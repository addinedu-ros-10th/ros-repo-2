from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'waypoint_director'
    share = get_package_share_directory(pkg)
    yaml_path = os.path.join(share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        Node(
            package=pkg,
            executable='waypoint_director_node',
            name='waypoint_director',
            output='screen',
            parameters=[
                {'waypoints_yaml': yaml_path},
                {'base_frame': 'base_link'},
            ],
        )
    ])
