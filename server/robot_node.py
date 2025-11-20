import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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
        pos.x = random.uniform(7.0, 32.0)
        pos.y = random.uniform(6.0, 43.0)
        pos.z = 0.0
        self.publisher_.publish(pos)
        print(f"[ROS2 발행] 로봇{self.robot_id} 위치: ({pos.x:.2f}, {pos.y:.2f})")

def main():
    rclpy.init()

    # 로봇 3대 노드 생성
    nodes = [RobotPositionPublisher(i) for i in range(1, 4)]

    # MultiThreadedExecutor에 노드 등록
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()   # 모든 노드 동시에 실행
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
