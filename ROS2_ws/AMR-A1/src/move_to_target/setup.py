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

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricky',
    maintainer_email='example@example.com',
    description='Move to target controller package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_rtr_controller = move_to_target.waypoint:main',
            'cmd_vel_switcher = move_to_target.cmd_vel_switcher:main',
            'pid_controller = move_to_target.pid_controller:main',
            'voice_nav_assistant = move_to_target.voice_nav_assistant:main',
            'waypoint_marker = move_to_target.waypoint_marker:main',
        ],
    },
)
