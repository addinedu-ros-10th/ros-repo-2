import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import keyboard  # pip install keyboard

KEYS = ['u', 'i', 'o', 'j', 'k', 'l', 'm', ',', '.']


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(String, 'teleop/key', 10)
        self.create_timer(0.01, self.check_key)

    def check_key(self):
        for key in KEYS:
            if keyboard.is_pressed(key):
                msg = String()
                msg.data = key
                self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
