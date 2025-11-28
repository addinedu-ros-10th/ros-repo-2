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

        # Nav2 Simple Commander
        self.nav = BasicNavigator()

        # === Subscribers ===
        self.create_subscription(
            String,
            '/selected_waypoint',
            self.selected_waypoint_callback,
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # === Publishers ===
        self.current_wp_pub = self.create_publisher(String, '/current_waypoint', 10)
        self.arrival_pub = self.create_publisher(String, '/arrival_notification', 10)

        # === Load Waypoints ===
        package_dir = get_package_share_directory('move_to_target')
        # install 경로: .../share/move_to_target/config/my_map_waypoints.yaml
        default_waypoint_file = os.path.join(package_dir, 'config', 'my_map_waypoints.yaml')
        self.declare_parameter('waypoint_file', default_waypoint_file)
        yaml_path = self.get_parameter('waypoint_file').get_parameter_value().string_value

        self.get_logger().info(f"📂 waypoint_file: {yaml_path}")

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            # 이름 → 전체 waypoint dict
            self.waypoints = {wp['name']: wp for wp in data['waypoints']}
            self.get_logger().info(f"✅ Loaded waypoints: {list(self.waypoints.keys())}")
        except Exception as e:
            self.get_logger().error(f"❌ Waypoint load failed: {e}")
            self.waypoints = {}

        # === Graph 구조 만들기 (원래 방식 유지) ===
        self.graph = {}
        # arrival_yaw 정보만 따로 저장하는 딕셔너리 추가
        self.arrival_yaws = {}

        for name, wp in self.waypoints.items():
            raw_neighbors = wp.get('neighbors', [])
            neighbor_names = []

            for nb in raw_neighbors:
                if isinstance(nb, str):
                    neighbor_names.append(nb)
                    # 기본 arrival_yaw = 0
                    self.arrival_yaws[(name, nb)] = 0
                elif isinstance(nb, dict) and 'name' in nb:
                    neighbor_names.append(nb['name'])
                    # arrival_yaw 정보 저장 (없으면 0)
                    arrival_yaw = nb.get('arrival_yaw', 0)
                    self.arrival_yaws[(name, nb['name'])] = arrival_yaw

            self.graph[name] = neighbor_names

        self.get_logger().info(f"🧠 Graph: {self.graph}")
        self.get_logger().info(f"🎯 Arrival yaws loaded: {len(self.arrival_yaws)} entries")

        # === Initial pose 설정 (원점 기준) ===
        self.current_wp_name = "원점"
        origin = self.waypoints.get('원점')
        if origin:
            px = origin['pose']['position']['x']
            py = origin['pose']['position']['y']
            yaw = origin['pose'].get('yaw', 0.0)
            self.set_initial_pose(px, py, yaw)
        else:
            self.get_logger().warn("⚠️ '원점' waypoint not found in YAML!")

        # Nav2 활성화 기다리기
        self.get_logger().info("⏳ Waiting for Nav2 to activate...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("✅ Nav2 is active!")

        # === State ===
        self.current_position = None
        self.current_path = deque()
        self.final_goal_name = None
        self.moving = False

        # 주기 상태 갱신 타이머
        self.create_timer(0.3, self.update)

    # ==================== 콜백들 ====================

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_position = msg.pose.pose.position

    def selected_waypoint_callback(self, msg: String):
        target_name = msg.data.strip()
        self.get_logger().info(f"🎯 Received target waypoint: '{target_name}'")

        if target_name not in self.waypoints:
            self.get_logger().warn(f"⚠️ Unknown target: {target_name}")
            self.get_logger().warn(f"   Available: {list(self.waypoints.keys())}")
            return

        if target_name == self.current_wp_name:
            self.get_logger().info(f"ℹ️ Already at {target_name}")
            return

        # BFS로 경로 계산
        path = self.bfs_path(self.current_wp_name, target_name)
        if not path:
            self.get_logger().warn(f"⚠️ No path from {self.current_wp_name} → {target_name}")
            return

        self.current_path = deque(path)
        self.final_goal_name = target_name
        self.moving = False

        self.get_logger().info(f"🗺️ Planned path: {list(self.current_path)}")

    # ==================== Nav2 제어 ====================

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
        self.get_logger().info(f"📍 Initial pose set: ({x:.2f}, {y:.2f}, yaw={yaw_deg:.1f})")

    def update(self):
        # 현재 위치한 waypoint 이름 퍼블리시
        self.current_wp_pub.publish(String(data=self.current_wp_name))

        # 아직 할 일이 없으면 리턴
        if not self.current_path and not self.moving:
            return

        # 이동 중이면 Nav2 상태 체크
        if self.moving:
            if not self.nav.isTaskComplete():
                return  # 아직 도착 안함

            result = self.nav.getResult()
            arrived_wp = self.current_path.popleft()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"✅ Reached {arrived_wp}")
                self.current_wp_name = arrived_wp
            else:
                self.get_logger().warn(f"⚠️ Nav2 failed at {arrived_wp}")
                self.current_wp_name = arrived_wp  # 일단 도달한 것으로 처리

            self.moving = False

            # 최종 목적지 도착 처리
            if not self.current_path and arrived_wp == self.final_goal_name:
                self.arrival_pub.publish(String(data=arrived_wp))
                self.get_logger().info(f"🏁 Final destination reached: {arrived_wp}")
                self.final_goal_name = None

            return

        # 여기까지 왔으면: 현재 이동 중은 아니고, current_path 에 다음 목표가 있음
        next_wp_name = self.current_path[0]
        next_wp = self.waypoints.get(next_wp_name)
        if not next_wp:
            self.get_logger().error(f"❌ Missing waypoint in dict: {next_wp_name}")
            self.current_path.clear()
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = next_wp['pose']['position']['x']
        goal.pose.position.y = next_wp['pose']['position']['y']

        # 🔥 arrival_yaw 사용 (기존 yaw 대신)
        arrival_yaw_key = (self.current_wp_name, next_wp_name)
        if arrival_yaw_key in self.arrival_yaws:
            yaw_deg = self.arrival_yaws[arrival_yaw_key]
            self.get_logger().info(f"🎯 Using arrival_yaw: {yaw_deg}° for {self.current_wp_name} → {next_wp_name}")
        else:
            yaw_deg = next_wp['pose'].get('yaw', 0.0)
            self.get_logger().info(f"ℹ️ Using default yaw: {yaw_deg}° for {next_wp_name}")

        q = get_quaternion_from_yaw(yaw_deg)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.get_logger().info(f"🚀 Sending Nav2 goal to '{next_wp_name}' "
                               f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}, yaw={yaw_deg:.1f}°)")
        self.nav.goToPose(goal)
        self.moving = True

    # ==================== BFS 경로 계산 ====================

    def bfs_path(self, start_name, goal_name):
        """neighbors 안에 dict들이 있어서, 이름만 뽑아서 그래프를 만든 뒤 BFS 수행"""
        if start_name not in self.graph or goal_name not in self.graph:
            self.get_logger().warn(f"⚠️ bfs_path: '{start_name}' 또는 '{goal_name}' 이 graph에 없음")
            return []

        from collections import deque as dq
        visited = set()
        queue = dq([[start_name]])

        while queue:
            path = queue.popleft()
            node = path[-1]
            if node == goal_name:
                # start 자신은 빼고, 그 다음부터 리턴
                return path[1:]
            if node in visited:
                continue
            visited.add(node)
            for nb in self.graph.get(node, []):
                if nb not in visited:
                    queue.append(path + [nb])

        return []


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