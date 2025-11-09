import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from collections import deque
import math
import tf_transformations
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def get_quaternion_from_yaw(yaw_deg: float):
    yaw = math.radians(yaw_deg)
    q = tf_transformations.quaternion_from_euler(0, 0, yaw)
    return q


class WaypointRTRController(Node):
    def __init__(self):
        super().__init__('waypoint_rtr_controller')
        self.nav = BasicNavigator()

        # === Subscribers ===
        self.create_subscription(String, '/selected_waypoint', self.selected_waypoint_callback, 10)
        self.create_subscription(String, '/control_mode', self.mode_change_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # === Publishers ===
        self.current_wp_pub = self.create_publisher(String, '/current_waypoint', 10)
        self.arrival_pub = self.create_publisher(String, '/arrival_notification', 10)
        self.control_mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.pid_target_pub = self.create_publisher(PoseStamped, '/pid_target_pose', 10)

        # === Load Waypoints ===
        package_dir = get_package_share_directory('move_to_target')
        default_waypoint_file = os.path.join(package_dir, 'my_map_waypoints.yaml')
        self.declare_parameter('waypoint_file', default_waypoint_file)
        yaml_path = self.get_parameter('waypoint_file').get_parameter_value().string_value

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            self.waypoints = {wp['name']: wp for wp in data['waypoints']}
            self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints.")
        except Exception as e:
            self.get_logger().error(f"❌ Waypoint load failed: {e}")
            self.waypoints = {}

        # === Initialize Pose (원점 기준) ===
        origin = self.waypoints.get('원점')
        if origin:
            self.set_initial_pose(origin['pose']['position']['x'],
                                  origin['pose']['position']['y'],
                                  origin['pose'].get('yaw', 0.0))
        else:
            self.get_logger().warn("⚠️ '원점' waypoint not found!")

        self.get_logger().info("⏳ Waiting for Nav2 to activate...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("✅ Nav2 active!")

        # === State Variables ===
        self.current_wp_name = "원점"
        self.current_position = None
        self.current_mode = "nav2"
        self.current_path = deque()
        self.final_goal_name = None
        self.moving = False
        self.checking_2_pass = False  # 2번 통과 감시 플래그

        # === Timer ===
        self.create_timer(0.3, self.update)

    # -----------------------
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_position = msg.pose.pose.position
        # 2번 통과 감지
        if self.checking_2_pass and self.current_position:
            wp2 = self.waypoints.get("2번", {}).get("pose", {}).get("position", {})
            wp2_x, wp2_y = wp2.get("x"), wp2.get("y")
            if wp2_x is None or wp2_y is None:
                return
            dx = self.current_position.x - wp2_x
            dy = self.current_position.y - wp2_y
            dist = math.hypot(dx, dy)
            if dist < 0.25:  # 25cm 근처 지나감
                self.get_logger().info("🎯 Passed near 2번 (no stop). Enabling PID transition.")
                self.checking_2_pass = False
                # Nav2 취소 후 PID로 전환
                self.nav.cancelTask()
                self.switch_to_pid_mode("1번")

    def mode_change_callback(self, msg: String):
        new_mode = msg.data.lower()
        if self.current_mode != new_mode:
            self.current_mode = new_mode
            self.control_mode_pub.publish(String(data=new_mode))
            self.get_logger().info(f"🔄 Mode changed to {new_mode}")

    # -----------------------
    def bfs_path(self, start_name, goal_name):
        if start_name not in self.waypoints or goal_name not in self.waypoints:
            return []
        neighbors = {name: wp.get('neighbors', []) for name, wp in self.waypoints.items()}
        visited = set()
        queue = deque([[start_name]])

        while queue:
            path = queue.popleft()
            node = path[-1]
            if node == goal_name:
                return path[1:]
            if node in visited:
                continue
            visited.add(node)
            for nb in neighbors.get(node, []):
                if nb not in visited:
                    queue.append(path + [nb])
        return []

    # -----------------------
    def selected_waypoint_callback(self, msg: String):
        target_name = msg.data.strip()
        if target_name not in self.waypoints:
            self.get_logger().warn(f"⚠️ Unknown target: {target_name}")
            return

        path = self.bfs_path(self.current_wp_name, target_name)
        if not path:
            self.get_logger().warn(f"⚠️ No path from {self.current_wp_name} → {target_name}")
            return

        self.current_path = deque(path)
        self.final_goal_name = target_name
        self.moving = False
        self.current_mode = "nav2"
        self.control_mode_pub.publish(String(data="nav2"))

        self.get_logger().info(f"🎯 Path planned: {list(self.current_path)}")

    # -----------------------
    def set_initial_pose(self, x, y, yaw_deg):
        q = get_quaternion_from_yaw(yaw_deg)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.nav.setInitialPose(pose)
        self.get_logger().info(f"📍 Initial pose: ({x:.2f}, {y:.2f}, yaw={yaw_deg:.1f})")

    # -----------------------
    def update(self):
        # 현재 위치 알림
        self.current_wp_pub.publish(String(data=self.current_wp_name))
        if self.current_mode == "pid":
            return

        # Nav2 완료 시
        if self.moving and self.nav.isTaskComplete():
            result = self.nav.getResult()
            wp_name = self.current_path.popleft()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"✅ Reached {wp_name}")
            else:
                self.get_logger().warn(f"⚠️ Nav2 failed at {wp_name}")

            self.current_wp_name = wp_name
            self.moving = False

            # 2번 → 1번 시퀀스 준비 (통과 감시 시작)
            if wp_name == "원점" and self.current_path and self.current_path[0] == "2번":
                self.checking_2_pass = True
                self.get_logger().info("👀 Will monitor for passing 2번 (no stop).")

            # 최종 목적지 도착
            if not self.current_path and wp_name == self.final_goal_name:
                self.arrival_pub.publish(String(data=wp_name))
                self.get_logger().info(f"🏁 Arrived final: {wp_name}")
                self.final_goal_name = None
            return

        # Nav2 중간 목표가 없으면 skip
        if self.moving or not self.current_path:
            return

        next_wp_name = self.current_path[0]
        next_wp = self.waypoints.get(next_wp_name)
        if not next_wp:
            self.get_logger().error(f"❌ Missing waypoint: {next_wp_name}")
            self.current_path.clear()
            return

        # 목표 Pose
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = next_wp['pose']['position']['x']
        goal.pose.position.y = next_wp['pose']['position']['y']

        # 2번은 yaw 무시 (통과용)
        yaw_deg = 0.0 if next_wp_name == "2번" else next_wp['pose'].get('yaw', 0.0)
        q = get_quaternion_from_yaw(yaw_deg)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.nav.goToPose(goal)
        self.moving = True
        self.get_logger().info(f"🚀 Nav2 moving to '{next_wp_name}' (yaw={yaw_deg:.1f}°)")

    # -----------------------
    def switch_to_pid_mode(self, next_target="1번"):
        self.get_logger().info("🔁 Switching to PID mode (after passing 2번)")
        self.current_mode = "pid"
        self.control_mode_pub.publish(String(data="pid"))

        wp = self.waypoints.get(next_target)
        if not wp:
            self.get_logger().error(f"❌ PID target not found: {next_target}")
            self.switch_to_nav2_mode()
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wp['pose']['position']['x']
        goal.pose.position.y = wp['pose']['position']['y']
        q = get_quaternion_from_yaw(wp['pose'].get('yaw', 0.0))
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.pid_target_pub.publish(goal)
        self.get_logger().info(f"🎯 PID target published for {next_target}")

    def switch_to_nav2_mode(self):
        self.current_mode = "nav2"
        self.control_mode_pub.publish(String(data="nav2"))
        self.get_logger().info("🔄 Back to Nav2 mode safely")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRTRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
