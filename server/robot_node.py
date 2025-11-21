'''
로봇 좌표 임의로 주는것
'''

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped


class ManualPosePublisher(Node):
    """
    원하는 좌표(x,y)를 그대로 /robot{ID}/amcl_pose 로 퍼블리시하는 노드.
    """

    def __init__(self, robot_id, pose_list):
        super().__init__(f'manual_pose_pub_{robot_id}')
        self.robot_id = robot_id
        self.pose_list = pose_list   # [(x1,y1), (x2,y2), ...]
        self.index = 0

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/robot{robot_id}/amcl_pose',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"로봇{robot_id} 수동 좌표 발행 시작")

    def timer_callback(self):
        if not self.pose_list:
            return

        x, y = self.pose_list[self.index]
        self.index = (self.index + 1) % len(self.pose_list)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        self.publisher_.publish(msg)

        print(f"[PUB] robot{self.robot_id} -> ({x:.2f}, {y:.2f})")


def main():
    rclpy.init()

    # ★ 여기에 너가 원하는 좌표를 넣으면 됨 ★
    poses_robot21 = [(0.8559493965290068, 0.08201862789207194), (0.8559493965290068, 0.08201862789207194), (0.8559493965290068, 0.08201862789207194)]
    poses_robot22 = [(1.50, 0.0), (1.50, 1.0), (1.50, 2.0)]
    poses_robot23 = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]

    nodes = [
        ManualPosePublisher(21, poses_robot21),
        ManualPosePublisher(22, poses_robot22),
        ManualPosePublisher(23, poses_robot23),
    ]

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

