from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 공유 디렉토리 가져오기
    package_dir = get_package_share_directory('move_to_target')
    
    # 설정 파일 경로
    config = os.path.join(package_dir, 'config', 'params.yaml')
    
    # 웨이포인트 파일 경로
    waypoint_file = os.path.join(package_dir, 'my_map_waypoints.yaml')
    
    # 노드 정의
    waypoint_marker_node = Node(
        package='move_to_target',
        executable='waypoint_marker',
        output='screen',
        parameters=[config, {'waypoint_file': waypoint_file}],
        name='waypoint_marker'
    )
    
    cmd_vel_switcher_node = Node(
        package='move_to_target',
        executable='cmd_vel_switcher',
        output='screen',
        parameters=[config],
        name='cmd_vel_switcher'
    )
    
    pid_controller_node = Node(
        package='move_to_target',
        executable='pid_controller',
        output='screen',
        parameters=[config],
        name='pid_controller'
    )
    
    waypoint_node = Node(
        package='move_to_target',
        executable='waypoint',
        output='screen',
        parameters=[config, {'waypoint_file': waypoint_file}],
        name='waypoint_rtr_controller'
    )
    
    voice_nav_assistant_node = Node(
        package='move_to_target',
        executable='voice_nav_assistant',
        output='screen',
        parameters=[config],
        name='voice_nav_assistant',
        respawn=True,
        respawn_delay=2.0
    )
    
    # 타이머를 사용한 지연 시작
    delayed_waypoint_marker = TimerAction(
        period=5.0,
        actions=[waypoint_marker_node]
    )
    
    delayed_cmd_vel_switcher = TimerAction(
        period=8.0,
        actions=[cmd_vel_switcher_node]
    )
    
    delayed_pid_controller = TimerAction(
        period=8.0,
        actions=[pid_controller_node]
    )
    
    delayed_waypoint = TimerAction(
        period=15.0,
        actions=[waypoint_node]
    )
    
    delayed_voice_nav = TimerAction(
        period=18.0,
        actions=[voice_nav_assistant_node]
    )
    
    return LaunchDescription([
        delayed_waypoint_marker,
        delayed_cmd_vel_switcher,
        delayed_pid_controller,
        delayed_waypoint,
        delayed_voice_nav,
    ])