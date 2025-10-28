from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('waypoint_yaml_nav2')
    yaml_path = os.path.join(pkg, 'waypoints.yaml')

    return LaunchDescription([
        Node(
            package='waypoint_yaml_nav2',
            executable='run_waypoints',
            name='waypoint_yaml_runner',
            output='screen',
            parameters=[{'waypoints_yaml': yaml_path}],
        )
    ])
