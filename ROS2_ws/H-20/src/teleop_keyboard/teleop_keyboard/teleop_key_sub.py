import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class TeleopKeySubscriber(Node):
    def __init__(self):
        super().__init__('teleop_key_subscriber')

        # QoS 설정 (teleop_keyboard 와 호환)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            String,
            'teleop/key',
            self.key_callback,
            qos
        )

        self.get_logger().info("teleop/key subscriber started.")

    def key_callback(self, msg: String):
        key = msg.data
        self.get_logger().info(f"Received Key: {key}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
