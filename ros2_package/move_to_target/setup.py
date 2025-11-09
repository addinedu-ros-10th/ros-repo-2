from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'move_to_target'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일들
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # ✅ 추가: Python 런치 파일
        # 설정 파일들
        ('share/' + package_name + '/config', glob('config/*.yaml')),  # ✅ 추가: config 폴더
        # 웨이포인트 YAML 파일
        ('share/' + package_name, ['my_map_waypoints.yaml']),
    ],
    install_requires=[
        'setuptools',
        'SpeechRecognition',
        'gTTS',
        'playsound==1.2.2',
        'PyYAML'
    ],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='wlstnz3@gmail.com',
    description='Move robot to target position package',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint = move_to_target.waypoint:main',
            'waypoint_marker = move_to_target.waypoint_marker:main',
            'voice_nav_assistant = move_to_target.voice_nav_assistant:main',
            'pid_controller = move_to_target.pid_controller:main',
            'cmd_vel_switcher = move_to_target.cmd_vel_switcher:main',
            'nav2_goal_bridge = move_to_target.nav2_goal_bridge:main',  # ✅ 이 노드는 있나요?
        ],
    },
)