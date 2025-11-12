import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class RobotPositionPublisher(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}_node')
        self.publisher_ = self.create_publisher(Point, f'/robot{robot_id}/pos', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.robot_id = robot_id
        self.get_logger().info(f'로봇 {robot_id} 위치 발행 시작')

    def timer_callback(self):
        pos = Point()
        pos.x = random.uniform(0.0, 10.0)
        pos.y = random.uniform(0.0, 10.0)
        pos.z = 0.0
        self.publisher_.publish(pos)
        print(f"[ROS2 발행] 로봇{self.robot_id} 위치: ({pos.x:.2f}, {pos.y:.2f})")

def main():
    rclpy.init()
    nodes = [RobotPositionPublisher(i) for i in range(1, 4)]
    try:
        rclpy.spin(nodes[0])  # 한 노드만 spin
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
