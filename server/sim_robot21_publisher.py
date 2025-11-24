import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class SimRobotPublisher(Node):
    """
    고정된 AMCL 위치를 계속 발행하는 테스트 퍼블리셔
    /robot21/amcl_pose 에만 PoseWithCovarianceStamped 발행
    """

    def __init__(self):
        super().__init__('sim_robot_publisher')

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/robot22/amcl_pose',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)  # 10Hz

        # 고정 좌표 (원하는 값 넣으면 됨)
        self.fixed_x = -0.0
        self.fixed_y = -0.0
        self.fixed_yaw = 0.0  # 필요하면 yaw도 적용 가능

        self.get_logger().info(
            'SimRobotPublisher 시작: /robot22/amcl_pose 고정 위치 발행'
        )

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = self.fixed_x
        msg.pose.pose.position.y = self.fixed_y
        msg.pose.pose.position.z = 0.0

        # orientation은 기본값(0,0,0,1) 그대로

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"[SIM] /robot22/amcl_pose = ({self.fixed_x:.2f}, {self.fixed_y:.2f})"
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
