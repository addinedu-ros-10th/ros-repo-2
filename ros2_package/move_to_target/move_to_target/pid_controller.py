import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # 구독자
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.position_callback, 10)
        self.create_subscription(String, '/control_mode', self.mode_callback, 10)
        self.create_subscription(PoseStamped, '/pid_target_pose', self.waypoint_goal_callback, 10)
        
        # 발행자
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_pid', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        
        # 상태 변수
        self.current_position = Point()
        self.current_yaw = 0.0
        self.current_mode = "nav2"
        self.target_position = None
        self.is_active = False
        self.control_timer = None
        
        # PID 게인 (더 부드러운 제어)
        self.Kp_linear = 0.3    # 감소
        self.Kp_angular = 1.5   # 감소
        
        # 제어 파라미터
        self.arrival_threshold = 0.08      # 8cm
        self.angle_threshold = math.radians(20)  # 20도
        self.max_linear_vel = 0.15         # 감소
        self.max_angular_vel = 0.7         # 감소
        
        self.get_logger().info("🎯 PID Controller Started")
        
        # 제어 주기 타이머 (나중에 시작)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def position_callback(self, msg: PoseWithCovarianceStamped):
        """로봇의 현재 위치 및 자세 수신"""
        try:
            # 위치 정보
            self.current_position.x = msg.pose.pose.position.x
            self.current_position.y = msg.pose.pose.position.y
            
            # 쿼터니언에서 yaw 각도 계산
            orientation = msg.pose.pose.orientation
            siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception as e:
            self.get_logger().warn(f"⚠️ 위치 정보 처리 오류: {e}")

    def mode_callback(self, msg):
        """제어 모드 변경 처리"""
        new_mode = msg.data.lower()
        if self.current_mode != new_mode:
            self.current_mode = new_mode
            self.is_active = (self.current_mode == "pid")
            
            if self.is_active:
                self.get_logger().info("🎯 PID 모드 활성화됨")
            else:
                # 비활성화 시 정지
                self.safe_stop()
                self.target_position = None
                self.get_logger().info("🔴 PID 모드 비활성화")

    def waypoint_goal_callback(self, msg):
        """PID 목표 위치 수신"""
        if self.current_mode == "pid":
            self.target_position = Point()
            self.target_position.x = msg.pose.position.x
            self.target_position.y = msg.pose.position.y
            self.get_logger().info(f"🎯 PID 목표 설정: ({self.target_position.x:.2f}, {self.target_position.y:.2f})")

    def safe_stop(self):
        """안전한 정지"""
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def normalize_angle(self, angle):
        """각도를 [-π, π] 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        """PID 제어 루프"""
        if not self.is_active or self.target_position is None:
            return
            
        try:
            dx = self.target_position.x - self.current_position.x
            dy = self.target_position.y - self.current_position.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # 도착 판단
            if distance < self.arrival_threshold:
                self.handle_arrival()
                return
                
            # 목표 각도 계산
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)
            
            # 속도 계산 (부드러운 제어)
            linear_vel = min(distance * self.Kp_linear, self.max_linear_vel)
            angular_vel = angle_error * self.Kp_angular
            
            # 각도 오차에 따른 제어 전략
            if abs(angle_error) > self.angle_threshold:
                # 방향 먼저 맞추기 (속도 감소)
                linear_vel *= 0.3
                angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)
            else:
                # 직진 위주 (각속도 제한)
                angular_vel = max(min(angular_vel, self.max_angular_vel * 0.6), 
                                -self.max_angular_vel * 0.6)
            
            # 명령 발행
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_pub.publish(cmd)
            
            self.get_logger().info(
                f"🎯 PID 제어 중: 거리 {distance:.3f}m, 각도오차 {math.degrees(angle_error):.1f}°", 
                throttle_duration_sec=2.0
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ PID 제어 오류: {e}")
            self.safe_stop()

    def handle_arrival(self):
        """목표 도착 처리"""
        self.get_logger().info("✅ PID 목표 도달! Nav2로 복귀 요청")
        
        # 정지 명령
        self.safe_stop()
        
        # 상태 초기화
        self.target_position = None 
        self.is_active = False
        
        # Nav2 모드로 복귀 알림
        mode_msg = String()
        mode_msg.data = "nav2"
        self.mode_pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()