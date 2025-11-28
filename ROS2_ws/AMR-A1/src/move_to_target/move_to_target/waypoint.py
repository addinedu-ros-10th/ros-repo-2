import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from collections import deque
import math
import tf_transformations
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def get_quaternion_from_yaw(yaw_deg: float):
    yaw = math.radians(yaw_deg)
    q = tf_transformations.quaternion_from_euler(0, 0, yaw)
    return q


class PID:
    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def step(self, error, current_time):
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9

        # P
        p = self.kp * error

        # I
        if dt > 0.0:
            self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i = self.ki * self.integral

        # D
        if dt > 0.0:
            d_err = (error - self.prev_error) / dt
        else:
            d_err = 0.0
        d = self.kd * d_err

        u = p + i + d

        if self.output_limit is not None:
            u = max(min(u, self.output_limit), -self.output_limit)

        self.prev_error = error
        self.prev_time = current_time
        return u


class WaypointRTRController(Node):
    def __init__(self):
        super().__init__('waypoint_rtr_controller')

        # Nav2 Simple Commander
        self.nav = BasicNavigator()

        # === Subscribers ===
        self.create_subscription(
            String,
            '/selected_waypoint',
            self.selected_waypoint_callback,
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # === Publishers ===
        self.current_wp_pub = self.create_publisher(String, '/current_waypoint', 10)
        self.arrival_pub = self.create_publisher(String, '/arrival_notification', 10)
        # PID 정밀 제어용 속도 출력
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === Load Waypoints ===
        package_dir = get_package_share_directory('move_to_target')
        # install 경로: .../share/move_to_target/config/my_map_waypoints.yaml
        default_waypoint_file = os.path.join(package_dir, 'config', 'my_map_waypoints.yaml')
        self.declare_parameter('waypoint_file', default_waypoint_file)
        yaml_path = self.get_parameter('waypoint_file').get_parameter_value().string_value

        self.get_logger().info(f"📂 waypoint_file: {yaml_path}")

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            # 이름 → 전체 waypoint dict
            self.waypoints = {wp['name']: wp for wp in data['waypoints']}
            self.get_logger().info(f"✅ Loaded waypoints: {list(self.waypoints.keys())}")
        except Exception as e:
            self.get_logger().error(f"❌ Waypoint load failed: {e}")
            self.waypoints = {}

        # === Graph 구조 만들기 ===
        self.graph = {}
        # arrival_yaw 정보만 따로 저장하는 딕셔너리
        self.arrival_yaws = {}

        for name, wp in self.waypoints.items():
            raw_neighbors = wp.get('neighbors', [])
            neighbor_names = []

            for nb in raw_neighbors:
                if isinstance(nb, str):
                    neighbor_names.append(nb)
                    # 기본 arrival_yaw = 0
                    self.arrival_yaws[(name, nb)] = 0
                elif isinstance(nb, dict) and 'name' in nb:
                    neighbor_names.append(nb['name'])
                    # arrival_yaw 정보 저장 (없으면 0)
                    arrival_yaw = nb.get('arrival_yaw', 0)
                    self.arrival_yaws[(name, nb['name'])] = arrival_yaw

            self.graph[name] = neighbor_names

        self.get_logger().info(f"🧠 Graph: {self.graph}")
        self.get_logger().info(f"🎯 Arrival yaws loaded: {len(self.arrival_yaws)} entries")

        # === Initial pose 설정 (원점 기준) ===
        self.current_wp_name = "원점"
        origin = self.waypoints.get('원점')
        if origin:
            px = origin['pose']['position']['x']
            py = origin['pose']['position']['y']
            yaw = origin['pose'].get('yaw', 0.0)
            self.set_initial_pose(px, py, yaw)
        else:
            self.get_logger().warn("⚠️ '원점' waypoint not found in YAML!")

        # Nav2 활성화 기다리기
        self.get_logger().info("⏳ Waiting for Nav2 to activate...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("✅ Nav2 is active!")

        # === State ===
        self.current_position = None
        self.current_pose = None  # orientation까지 포함
        self.current_path = deque()
        self.final_goal_name = None
        self.moving = False

        # === 정밀 제어(PID) 상태 ===
        self.precision_mode = False
        self.precision_target = None  # (x, y, yaw_deg) in map frame

        # PID 제어기 (초기값, 이후 튜닝 필요)
        self.lin_pid = PID(kp=0.8, ki=0.0, kd=0.0, output_limit=0.1)   # m/s
        self.ang_pid = PID(kp=1.5, ki=0.0, kd=0.0, output_limit=0.6)   # rad/s

        # 주기 상태 갱신 타이머
        self.create_timer(0.1, self.update)  # PID 반응 위해 0.1s로 약간 빠르게

    # ==================== 유틸 ====================

    @staticmethod
    def normalize_angle(a):
        """[-pi, pi] 범위로 각도 정규화"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ==================== 콜백들 ====================

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose
        self.current_position = msg.pose.pose.position

    def selected_waypoint_callback(self, msg: String):
        target_name = msg.data.strip()
        self.get_logger().info(f"🎯 Received target waypoint: '{target_name}'")

        if target_name not in self.waypoints:
            self.get_logger().warn(f"⚠️ Unknown target: {target_name}")
            self.get_logger().warn(f"   Available: {list(self.waypoints.keys())}")
            return

        if target_name == self.current_wp_name:
            self.get_logger().info(f"ℹ️ Already at {target_name}")
            return

        # BFS로 경로 계산
        path = self.bfs_path(self.current_wp_name, target_name)
        if not path:
            self.get_logger().warn(f"⚠️ No path from {self.current_wp_name} → {target_name}")
            return

        self.current_path = deque(path)
        self.final_goal_name = target_name
        self.moving = False
        self.precision_mode = False
        self.precision_target = None

        self.get_logger().info(f"🗺️ Planned path: {list(self.current_path)}")

    # ==================== Nav2 제어 ====================

    def set_initial_pose(self, x, y, yaw_deg):
        q = get_quaternion_from_yaw(yaw_deg)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.nav.setInitialPose(pose)
        self.get_logger().info(f"📍 Initial pose set: ({x:.2f}, {y:.2f}, yaw={yaw_deg:.1f})")

    def update(self):
        # 현재 위치한 waypoint 이름 퍼블리시
        self.current_wp_pub.publish(String(data=self.current_wp_name))

        # === 정밀 제어 모드 우선 처리 ===
        if self.precision_mode:
            self.run_precision_control()
            return

        # 아직 할 일이 없으면 리턴
        if not self.current_path and not self.moving:
            return

        # 이동 중이면 Nav2 상태 체크
        if self.moving:
            if not self.nav.isTaskComplete():
                return  # 아직 도착 안함

            result = self.nav.getResult()
            # 일단 현재 목표 waypoint 이름만 확인 (바로 popleft 하지 않음)
            arrived_wp = self.current_path[0]

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"✅ Reached (coarse) {arrived_wp}")
                self.current_wp_name = arrived_wp
            else:
                self.get_logger().warn(f"⚠️ Nav2 failed at {arrived_wp}")
                self.current_wp_name = arrived_wp  # 일단 도달한 것으로 처리

            # 이 waypoint는 coarse level에서 도달한 것으로 보고 path에서 제거
            self.current_path.popleft()
            self.moving = False

            # 최종 목적지 도착 처리 (coarse)
            if not self.current_path and arrived_wp == self.final_goal_name:
                self.get_logger().info(f"🏁 Coarse final destination reached: {arrived_wp}")
                # 여기서 바로 arrival_pub 하지 않고, PID 정밀 제어 모드로 진입
                self.start_precision_mode(arrived_wp)

            return

        # 여기까지 왔으면: 현재 이동 중은 아니고, current_path 에 다음 목표가 있음
        next_wp_name = self.current_path[0]
        next_wp = self.waypoints.get(next_wp_name)
        if not next_wp:
            self.get_logger().error(f"❌ Missing waypoint in dict: {next_wp_name}")
            self.current_path.clear()
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = next_wp['pose']['position']['x']
        goal.pose.position.y = next_wp['pose']['position']['y']

        # arrival_yaw 사용 (기존 yaw 대신)
        arrival_yaw_key = (self.current_wp_name, next_wp_name)
        if arrival_yaw_key in self.arrival_yaws:
            yaw_deg = self.arrival_yaws[arrival_yaw_key]
            self.get_logger().info(
                f"🎯 Using arrival_yaw: {yaw_deg}° for {self.current_wp_name} → {next_wp_name}"
            )
        else:
            yaw_deg = next_wp['pose'].get('yaw', 0.0)
            self.get_logger().info(f"ℹ️ Using default yaw: {yaw_deg}° for {next_wp_name}")

        q = get_quaternion_from_yaw(yaw_deg)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.get_logger().info(
            f"🚀 Sending Nav2 goal to '{next_wp_name}' "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}, yaw={yaw_deg:.1f}°)"
        )
        self.nav.goToPose(goal)
        self.moving = True

    # ==================== 정밀 제어 (PID) ====================

    def start_precision_mode(self, wp_name):
        wp = self.waypoints.get(wp_name)
        if not wp:
            self.get_logger().warn(f"⚠️ precision_mode: waypoint '{wp_name}' not found")
            return

        x = wp['pose']['position']['x']
        y = wp['pose']['position']['y']
        yaw_deg = wp['pose'].get('yaw', 0.0)

        self.precision_target = (x, y, yaw_deg)
        self.lin_pid.reset()
        self.ang_pid.reset()
        self.precision_mode = True

        self.get_logger().info(
            f"🎯 Start precision mode for '{wp_name}' at "
            f"({x:.3f}, {y:.3f}, yaw={yaw_deg:.1f}°)"
        )

    def run_precision_control(self):
        if self.current_pose is None or self.precision_target is None:
            return

        target_x, target_y, target_yaw_deg = self.precision_target

        # 현재 위치
        cur_x = self.current_pose.position.x
        cur_y = self.current_pose.position.y

        # 현재 yaw 추출 (rad)
        q = self.current_pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        _, _, cur_yaw = tf_transformations.euler_from_quaternion(quat)

        # 목표까지의 오차 (map 기준)
        dx = target_x - cur_x
        dy = target_y - cur_y

        # map -> base_link 좌표 변환 (로봇 기준 전/좌)
        cos_y = math.cos(cur_yaw)
        sin_y = math.sin(cur_yaw)
        ex =  cos_y * dx + sin_y * dy   # forward (+x)
        ey = -sin_y * dx + cos_y * dy   # left (+y)

        # 거리, 각도 오차
        dist = math.sqrt(ex * ex + ey * ey)
        target_yaw = math.radians(target_yaw_deg)
        yaw_err = self.normalize_angle(target_yaw - cur_yaw)  # [-pi, pi]

        # 정밀 기준 (튜닝해서 쓰면 됨)
        pos_tol = 0.02                  # 2 cm
        yaw_tol = math.radians(2.0)     # 2 deg

        # 충분히 가까우면 종료
        if dist < pos_tol and abs(yaw_err) < yaw_tol:
            twist = Twist()  # 0 속도
            self.cmd_vel_pub.publish(twist)
            self.precision_mode = False
            self.precision_target = None

            # 최종 도착 알림
            if self.final_goal_name is not None:
                self.arrival_pub.publish(String(data=self.final_goal_name))
                self.get_logger().info(
                    f"✅ Precision destination reached: {self.final_goal_name}"
                )
                self.final_goal_name = None
            else:
                self.get_logger().info("✅ Precision alignment done (no final_goal_name).")
            return

        now = self.get_clock().now()

        # 선속도: 전방 오차(ex)에 대해서만 PID
        v = self.lin_pid.step(ex, now)

        # 각속도: yaw_err에 대한 PID
        w = self.ang_pid.step(yaw_err, now)

        # 안전을 위한 추가 제한
        max_v = 0.1   # m/s
        max_w = 0.6   # rad/s
        v = max(min(v, max_v), -max_v)
        w = max(min(w, max_w), -max_w)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)

    # ==================== BFS 경로 계산 ====================

    def bfs_path(self, start_name, goal_name):
        """neighbors 안에 dict들이 있어서, 이름만 뽑아서 그래프를 만든 뒤 BFS 수행"""
        if start_name not in self.graph or goal_name not in self.graph:
            self.get_logger().warn(f"⚠️ bfs_path: '{start_name}' 또는 '{goal_name}' 이 graph에 없음")
            return []

        from collections import deque as dq
        visited = set()
        queue = dq([[start_name]])

        while queue:
            path = queue.popleft()
            node = path[-1]
            if node == goal_name:
                # start 자신은 빼고, 그 다음부터 리턴
                return path[1:]
            if node in visited:
                continue
            visited.add(node)
            for nb in self.graph.get(node, []):
                if nb not in visited:
                    queue.append(path + [nb])

        return []


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRTRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
