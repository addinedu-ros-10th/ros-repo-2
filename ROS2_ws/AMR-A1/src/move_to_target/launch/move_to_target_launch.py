from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('move_to_target')
    waypoint_path = os.path.join(pkg_share, 'config', 'my_map_waypoints.yaml')

    return LaunchDescription([
        # === Waypoint 네비게이션 컨트롤러 (Nav2 사용) ===
        Node(
            package='move_to_target',
            executable='waypoint_rtr_controller',
            name='waypoint_rtr_controller',
            output='screen',
            parameters=[{'waypoint_file': waypoint_path}]
        ),

        # === 음성 → waypoint 선택 ===
        Node(
            package='move_to_target',
            executable='voice_nav_assistant',
            name='voice_nav_assistant',
            output='screen'
        ),

        # === Rviz 마커 표시 ===
        Node(
            package='move_to_target',
            executable='waypoint_marker',
            name='waypoint_marker',
            output='screen',
            parameters=[{'waypoint_file': waypoint_path}]
        ),
    ])
