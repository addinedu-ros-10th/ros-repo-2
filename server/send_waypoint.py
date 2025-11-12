#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.pub = self.create_publisher(String, '/selected_waypoint', 10)
        self.get_logger().info('WaypointPublisher ready on /selected_waypoint')

    def wait_for_subscribers(self, timeout_s: float = 2.0):
        # 구독자가 붙을 때까지 대기 (타임아웃)
        start = self.get_clock().now().nanoseconds
        timeout_ns = int(timeout_s * 1e9)
        while (self.get_clock().now().nanoseconds - start) < timeout_ns:
            if self.pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def send_command(self, waypoint_index: int, robot_id: int = None, wait_for_sub: bool = True):
        if robot_id is None:
            txt = f'{waypoint_index}번'
        else:
            txt = f'robot_{robot_id}:{waypoint_index}번'
        msg = String()
        msg.data = txt

        if wait_for_sub:
            ok = self.wait_for_subscribers(timeout_s=2.0)
            if not ok:
                self.get_logger().warning('No subscribers detected within timeout, publishing anyway.')

        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        # 안전하게 보내기 (구독자 확인)
        node.send_command(3)
        # publish 후 약간 유지해서 DDS가 메시지를 전파하도록 함
        rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
