from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('waypoint_markers')
    yaml_path = os.path.join(pkg, 'waypoints.yaml')

    return LaunchDescription([
        Node(
            package='waypoint_markers',
            executable='show_waypoints',
            name='waypoint_markers',
            output='screen',
            parameters=[{'waypoints_yaml': yaml_path}],
            # namespace='pinky',  # 네임스페이스 쓰면 여기서 통일
        ),
        # RViz까지 같이 띄우고 싶다면 (rviz 파일 있으면 교체)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(pkg, 'rviz', 'waypoints.rviz')],
        #     output='screen'
        # ),
    ])
