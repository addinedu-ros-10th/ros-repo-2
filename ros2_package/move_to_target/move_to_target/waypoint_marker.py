#home/lim/pinky/src/move_to_target/move_to_target/waypoint_marker.py

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os

class WaypointMarkerNode(Node):
    def __init__(self):
        super().__init__('waypoint_marker')

        # 퍼블리셔
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # YAML 경로 파라미터
        self.declare_parameter('waypoint_file', '')
        yaml_path = self.get_parameter('waypoint_file').get_parameter_value().string_value

        if not os.path.isfile(yaml_path):
            self.get_logger().error(f"❌ Waypoint file not found: {yaml_path}")
            rclpy.shutdown()
            return

        # YAML 로드
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']

        # 반복 퍼블리시: 1초마다
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints and publishing markers...")

    def publish_markers(self):
        markers = MarkerArray()

        for i, wp in enumerate(self.waypoints):
            x = wp['pose']['position']['x']
            y = wp['pose']['position']['y']
            name = str(wp.get('name', f'WP{i+1}')).strip()

            # 점 마커
            point_marker = Marker()
            point_marker.header.frame_id = 'map'
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.id = i
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = 0.05
            point_marker.scale.x = 0.1
            point_marker.scale.y = 0.1
            point_marker.scale.z = 0.1
            point_marker.color.r = 1.0
            point_marker.color.g = 0.0
            point_marker.color.b = 0.0
            point_marker.color.a = 0.8
            markers.markers.append(point_marker)

            # 텍스트 마커
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.4
            text_marker.scale.z = 0.2
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = name
            markers.markers.append(text_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Stopped by user.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
