import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped


class SimRobotPublisher(Node):
    """
    GUI 테스트용 가짜 로봇 AMCL 좌표 퍼블리셔.
    /robot21/amcl_pose 에서 PoseWithCovarianceStamped 타입 발행.
    """

    def __init__(self):
        super().__init__('sim_robot_publisher')

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/robot21/amcl_pose',
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.t = 0.0

        self.get_logger().info(
            'SimRobotPublisher 시작: /robot21/amcl_pose 로 테스트 위치 발행'
        )

    def timer_callback(self):
        self.t += 0.1

        # 임의 원형 궤적
        center_x = 1.0
        center_y = -1.5
        radius = 1.0

        # AMCL 메시지 생성
        msg = PoseWithCovarianceStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = center_x + radius * math.cos(self.t)
        msg.pose.pose.position.y = center_y + radius * math.sin(self.t)
        msg.pose.pose.position.z = 0.0

        # orientation, covariance는 테스트이므로 기본값 그대로 둠

        self.publisher_.publish(msg)

        # 디버깅 출력
        self.get_logger().info(
            f"[SIM] /robot21/amcl_pose = ({msg.pose.pose.position.x:.2f}, "
            f"{msg.pose.pose.position.y:.2f})"
        )


def main():
    rclpy.init()
    node = SimRobotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
