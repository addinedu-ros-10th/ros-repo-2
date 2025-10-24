import os, math, yaml, json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

from tf_transformations import quaternion_from_euler
import math

# def quat_from_yaw_rad(yaw):
#     return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))

def quat_from_yaw_rad(yaw_rad) :
    # yaw_rad = math.radians(yaw_deg)  # 도 -> 라디안 변화
    q = quaternion_from_euler(0.0, 0.0, yaw_rad)
    return q

class WaypointDirector(Node):
    def __init__(self):
        super().__init__('waypoint_director')

        # ----- Parameters -----
        default_yaml = os.path.join(os.path.dirname(__file__), '..', 'config', 'waypoints.yaml')
        self.declare_parameter('waypoints_yaml', default_yaml)
        self.declare_parameter('base_frame', 'base_link')
        yaml_path = self.get_parameter('waypoints_yaml').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        with open(os.path.abspath(yaml_path), 'r') as f:
            cfg = yaml.safe_load(f)

        self.frame_id = cfg.get('frame_id', 'map')
        self.wp = {int(k): v for k, v in cfg.get('waypoints', {}).items()}

        # ----- Runtime state -----
        self.mode = 'stop'
        self.current_wp = None
        self.start_xy = None
        self.goal_xy = None

        # ===== TF (먼저 초기화: 콜백보다 앞) =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # ----- Action client -----
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None

        # ----- Publishers / Subscribers -----
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        # Marker publisher with TRANSIENT_LOCAL (latched-like)
        wanted_depth = max(10, len(self.wp) * 3 + 10)  # 라벨 2개/웨이포인트 + 여유
        marker_qos = QoSProfile(
            depth=wanted_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(Marker, 'waypoint_markers', marker_qos)

        # State publisher (for ros2 topic echo /state)
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.state_timer = self.create_timer(1.0, self._publish_state_periodic)

        # 마지막에 구독 생성(= 콜백 실행이 모든 준비 이후)
        self.sub = self.create_subscription(Int32, 'waypoint_id', self.on_waypoint_id, 10)

        # Keep last drawn markers to re-publish (timestamp refresh)
        self._last_start_marker = None
        self._last_goal_marker = None
        self._wp_label_markers = []  # TEXT markers
        self._wp_dot_markers = []    # SPHERE markers

        # 주기 재발행 타이머
        self.marker_timer = self.create_timer(1.0, self._republish_markers)

        # 초기: YAML 모든 웨이포인트 숫자 라벨을 표시(보관 포함)
        self._publish_all_wp_labels()

        self.get_logger().info('waypoint_director ready. Publish Int32 on /waypoint_id')

    # ===================== Command handling =====================
    def on_waypoint_id(self, msg: Int32):
        wp_id = int(msg.data)
        if wp_id not in self.wp:
            self.get_logger().warn(f'Unknown waypoint id: {wp_id}')
            return

        # Cancel current goal if exists
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        # Immediate stop
        self._publish_zero_once()

        # Start pose (map) & marker (TF가 아직 준비 안됐으면 스킵)
        start_pose = self._lookup_current_pose_in_map()
        if start_pose is not None:
            sx, sy = start_pose.pose.position.x, start_pose.pose.position.y
            self.start_xy = (sx, sy)
            self._publish_start_marker(sx, sy)
        else:
            self.start_xy = None

        # Build goal
        target = self.wp[wp_id]
        gx, gy = float(target['x']), float(target['y'])
        self.goal_xy = (gx, gy)
        self.current_wp = wp_id
        self.mode = 'run'

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        yaw_rad = math.radians(float(target.get('yaw_deg', 0.0)))
        qx, qy, qz, qw = quat_from_yaw_rad(yaw_rad)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        # Goal marker
        self._publish_goal_marker(gx, gy, yaw_rad)

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('navigate_to_pose action not available')
            self.mode = 'stop'
            return

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

        self.get_logger().info(f'Go to WP#{wp_id}: ({gx:.2f},{gy:.2f}) yaw={target.get("yaw_deg",0)}°')

    def _on_goal_sent(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')
            self._goal_handle = None
            self.mode = 'stop'
            return

        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._goal_handle = None
            self.mode = 'stop'
            return

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            _ = future.result().result
            self.get_logger().info('Goal finished.')
        except Exception as e:
            self.get_logger().warn(f'Goal ended with exception: {e}')
        finally:
            self._goal_handle = None
            self.mode = 'stop'

    # ===================== State =====================
    def _publish_state_periodic(self):
        msg = String()
        payload = {
            "mode": self.mode,
            "start": None if self.start_xy is None else {"wp": None, "x": self.start_xy[0], "y": self.start_xy[1]},
            "goal": None if self.goal_xy is None else {"wp": self.current_wp, "x": self.goal_xy[0], "y": self.goal_xy[1]},
        }
        msg.data = json.dumps(payload)
        self.state_pub.publish(msg)

    # ===================== TF =====================
    def _lookup_current_pose_in_map(self) -> PoseStamped or None:
        # 가드: init 순서/레이스 대비
        if not hasattr(self, 'tf_buffer'):
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, rclpy.time.Time())
            ps = PoseStamped()
            ps.header = tf.header
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation
            return ps
        except Exception as e:
            self.get_logger().warn(f'Cannot get TF {self.frame_id}->{self.base_frame}: {e}')
            return None

    # ===================== Markers =====================
    def _republish_markers(self):
        now = self.get_clock().now().to_msg()
        if self._last_start_marker:
            self._last_start_marker.header.stamp = now
            self.marker_pub.publish(self._last_start_marker)
        if self._last_goal_marker:
            self._last_goal_marker.header.stamp = now
            self.marker_pub.publish(self._last_goal_marker)

        for m in self._wp_label_markers:
            m.header.stamp = now
            self.marker_pub.publish(m)
        for m in self._wp_dot_markers:
            m.header.stamp = now
            self.marker_pub.publish(m)

    def _publish_start_marker(self, x, y):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'start_goal'
        m.id = 101
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 1.0
        self.marker_pub.publish(m)
        self._last_start_marker = m

    def _publish_goal_marker(self, x, y, yaw_rad=0.0):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'start_goal'
        m.id = 102
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        qx, qy, qz, qw = quat_from_yaw_rad(yaw_rad)
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        m.scale.x = 0.5
        m.scale.y = 0.12
        m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 1.0
        self.marker_pub.publish(m)
        self._last_goal_marker = m

    def _publish_all_wp_labels(self):
        """모든 WP 위치에 숫자(검정) + 작은 점(검정)을 항상 표시하고, 주기 재발행을 위해 보관."""
        self._wp_label_markers.clear()
        self._wp_dot_markers.clear()

        now = self.get_clock().now().to_msg()
        for k, v in self.wp.items():
            x = float(v['x']); y = float(v['y'])
            text = str(int(k))

            # 숫자 텍스트 (검정, 고정)
            t = Marker()
            t.header.frame_id = self.frame_id
            t.header.stamp = now
            t.ns = 'wp_labels'
            t.id = 1000 + int(k)
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = 0.75
            t.pose.orientation.w = 1.0
            t.scale.z = 0.45
            t.color.r, t.color.g, t.color.b, t.color.a = 0.0, 0.0, 0.0, 1.0  # 검정
            t.frame_locked = True
            t.text = text
            self.marker_pub.publish(t)
            self._wp_label_markers.append(t)

            # # 바닥 점 (검정)
            # d = Marker()
            # d.header.frame_id = self.frame_id
            # d.header.stamp = now
            # d.ns = 'wp_points'
            # d.id = 2000 + int(k)
            # d.type = Marker.SPHERE
            # d.action = Marker.ADD
            # d.pose.position.x = x
            # d.pose.position.y = y
            # d.pose.position.z = 0.05
            # d.pose.orientation.w = 1.0
            # d.scale.x = d.scale.y = d.scale.z = 0.06
            # d.color.r, d.color.g, d.color.b, d.color.a = 0.0, 0.0, 0.0, 1.0
            # d.frame_locked = True
            # self.marker_pub.publish(d)
            # self._wp_dot_markers.append(d)

    # ===================== Utils =====================
    def _publish_zero_once(self):
        self.cmd_pub.publish(Twist())

def main():
    rclpy.init()
    node = WaypointDirector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
