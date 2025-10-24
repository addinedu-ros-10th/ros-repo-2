from setuptools import find_packages, setup

package_name = 'waypoint_yaml_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/run_waypoints.launch.py']),
        (f'share/{package_name}', [f'{package_name}/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiming',
    maintainer_email='seojimni@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_waypoints = waypoint_yaml_nav2.waypoint_node:main',

        ],
    },
)
