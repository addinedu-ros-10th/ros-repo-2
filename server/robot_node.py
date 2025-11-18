'''
robot 랜덤으로 자표 뿌려주는 코드
'''

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Point


class PinkyPoseRelay(Node):
    """
    Pinky(Nav2)에서 나오는 실제 위치(/amcl_pose)를 받아서
    GUI가 이미 사용 중인 /robot{robot_id}/pos 토픽(Point)로 중계하는 노드.
    """

    def __init__(self):
        super().__init__('pinky_pose_relay')

        # GUI에서 사용할 로봇 ID (기본 1)
        self.declare_parameter('robot_id', 1)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value

        # 입력: Nav2에서 나오는 pose 토픽
        input_topic = '/amcl_pose'  # 필요 시 여기만 실제 토픽 이름으로 변경

        # 출력: GUI에서 이미 구독 중인 토픽 (/robot1/pos, /robot2/pos, ...)
        output_topic = f'/robot{self.robot_id}/pos'

        # 구독자 생성 (/amcl_pose)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            input_topic,
            self.pose_callback,
            10
        )

        # 퍼블리셔 생성 (/robot{ID}/pos)
        self.publisher_ = self.create_publisher(Point, output_topic, 10)

        self.get_logger().info(
            f'PinkyPoseRelay 시작: input={input_topic}, output={output_topic}, robot_id={self.robot_id}'
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        /amcl_pose 들어올 때마다 GUI용 /robot{ID}/pos(Point)로 변환해서 발행.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = 0.0

        self.publisher_.publish(pt)
        # 디버깅 필요하면 주석 해제
        # self.get_logger().info(f"/robot{self.robot_id}/pos -> ({x:.3f}, {y:.3f})")


def main():
    rclpy.init()
    node = PinkyPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
