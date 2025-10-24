import os, yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


class WaypointMarkers(Node):
    def __init__(self):
        super().__init__('waypoint_markers')

        # --- Params ---
        default_yaml = os.path.join(
            get_package_share_directory('waypoint_markers'),
            'config', 'waypoints.yaml'   # ← 네가 쓰는 경로에 맞춰 조정
        )
        self.declare_parameter('waypoints_yaml', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('dot_scale', 0.18)         # 점 크기(지름 m) 대략 18cm
        self.declare_parameter('draw_text', True)         # 인덱스 텍스트 보조표시
        self.declare_parameter('draw_line_strip', False)  # 경로 연결선

        yaml_path  = self.get_parameter('waypoints_yaml').value
        self.frame = self.get_parameter('frame_id').value
        self.dot_scale = float(self.get_parameter('dot_scale').value)
        self.draw_text = bool(self.get_parameter('draw_text').value)
        self.draw_line = bool(self.get_parameter('draw_line_strip').value)

        # --- Load YAML ---
        with open(yaml_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)

        # 기대 포맷: [{'x':..., 'y':..., 'yaw':...}, ...] 또는 {'waypoints':[...]}
        if isinstance(cfg, dict) and 'waypoints' in cfg:
            wp_list = cfg['waypoints']
        elif isinstance(cfg, list):
            wp_list = cfg
        else:
            wp_list = []

        self.waypoints = []
        for i, wp in enumerate(wp_list):
            # 다양한 키 이름 대응
            x = float(wp.get('x', wp.get('px', 0.0)))
            y = float(wp.get('y', wp.get('py', 0.0)))
            z = float(wp.get('z', 0.0))
            self.waypoints.append((x, y, z))

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints for RViz dot markers.")

        # --- QoS: Transient Local (RViz 나중에 켜도 받게) ---
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(MarkerArray, '/waypoints_markers', qos)

        # 최초 1회 + 주기적 재발행(1Hz)
        self.publish_markers()
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        arr = MarkerArray()

        # 1) 점(구) 리스트: SPHERE_LIST
        sphere_list = Marker()
        sphere_list.header.frame_id = self.frame
        sphere_list.header.stamp = self.get_clock().now().to_msg()
        sphere_list.ns = 'waypoints_dots'
        sphere_list.id = 0
        sphere_list.type = Marker.SPHERE_LIST
        sphere_list.action = Marker.ADD
        sphere_list.scale.x = self.dot_scale
        sphere_list.scale.y = self.dot_scale
        sphere_list.scale.z = self.dot_scale
        sphere_list.color.r = 0.10
        sphere_list.color.g = 0.75
        sphere_list.color.b = 1.00
        sphere_list.color.a = 1.0  # 투명도
        for (x, y, z) in self.waypoints:
            p = Point(x=x, y=y, z=z)
            sphere_list.points.append(p)
        arr.markers.append(sphere_list)

        # 2) 인덱스 텍스트 (선택)
        if self.draw_text:
            for i, (x, y, z) in enumerate(self.waypoints):
                txt = Marker()
                txt.header.frame_id = self.frame
                txt.header.stamp = self.get_clock().now().to_msg()
                txt.ns = 'waypoints_text'
                txt.id = i
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = x
                txt.pose.position.y = y
                txt.pose.position.z = z + self.dot_scale * 1.2
                txt.scale.z = self.dot_scale * 1.2   # 텍스트 크기(높이)
                txt.color.r = 1.0
                txt.color.g = 1.0
                txt.color.b = 1.0
                txt.color.a = 0.95
                txt.text = str(i)  # 번호만 심플하게
                arr.markers.append(txt)

        # 3) 연결선(선택)
        if self.draw_line and len(self.waypoints) >= 2:
            line = Marker()
            line.header.frame_id = self.frame
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = 'waypoints_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = max(0.02, self.dot_scale * 0.25)  # 선 두께
            line.color.r = 1.0
            line.color.g = 0.5
            line.color.b = 0.2
            line.color.a = 0.9
            for (x, y, z) in self.waypoints:
                line.points.append(Point(x=x, y=y, z=z))
            arr.markers.append(line)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkers()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
