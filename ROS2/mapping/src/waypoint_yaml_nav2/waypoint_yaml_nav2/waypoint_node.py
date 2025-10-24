import os, yaml, math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Quaternion

# Nav2 Simple Commander (sudo apt install ros-<distro>-nav2-simple-commander)
from nav2_simple_commander.robot_navigator import BasicNavigator

def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    # planar yaw 전용(roll=pitch=0). z,w만 설정해도 충분.
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q

def load_waypoints(nav_clock_node: Node, yaml_path: str):
    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)

    frame_id = cfg.get('frame_id', 'map')
    yaw_is_degrees = bool(cfg.get('yaw_is_degrees', True))
    goal_pose_list = []

    for wp in cfg['waypoints']:
        x = float(wp['x']); y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))
        if yaw_is_degrees:
            yaw = math.radians(yaw)

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = nav_clock_node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = quaternion_from_yaw(yaw)
        goal_pose_list.append(ps)
    return goal_pose_list

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_yaml_runner')

        # 파라미터: YAML 경로(패키지 설치본의 기본값)
        # 설치본 share 경로 사용 권장: launch에서 주입함
        self.declare_parameter('waypoints_yaml', '')
        self.yaml_path = self.get_parameter('waypoints_yaml').get_parameter_value().string_value

        if not self.yaml_path or not os.path.exists(self.yaml_path):
            self.get_logger().fatal(f'waypoints_yaml not found: {self.yaml_path}')
            raise FileNotFoundError(self.yaml_path)

        self.nav = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 lifecycle to be active...')
        self.nav.waitUntilNav2Active(localizer='amcl')  # 필요시 인자 조정

        self.goal_pose_list = load_waypoints(self.nav, self.yaml_path)
        self.get_logger().info(f'Loaded {len(self.goal_pose_list)} waypoints from YAML.')
        self.nav_start = self.nav.get_clock().now()

        self.nav.followWaypoints(self.goal_pose_list)
        self.timer = self.create_timer(0.2, self.spin_once)

    def spin_once(self):
        if self.nav.isTaskComplete():
            result = self.nav.getResult()
            self.get_logger().info(f'followWaypoints finished with result: {result}')
            rclpy.shutdown()
            return

        fb = self.nav.getFeedback()
        if fb:
            idx = fb.current_waypoint + 1
            total = len(self.goal_pose_list)
            # self.get_logger().info_throttle(2.0, f'Executing waypoint {idx}/{total}')
            now = self.nav.get_clock().now()
            if not hasattr(self, '_last_fb_log'):
                self._last_fb_log = now
            if (now - self._last_fb_log) > Duration(seconds = 2.0) :
                self.get_logger().info(f"Executing waypoint {idx}/{total}")
                self._last_fb_log = now
                
        # 타임아웃(예: 300초)
        if (self.nav.get_clock().now() - self.nav_start) > Duration(seconds=300.0):
            self.get_logger().warn('Timeout reached. Canceling task.')
            self.nav.cancelTask()

def main():
    rclpy.init()
    node = WaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
