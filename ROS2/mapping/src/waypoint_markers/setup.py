from setuptools import find_packages, setup

package_name = 'waypoint_markers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    package_data={
        'waypoint_markers': ['waypoints.yaml'],  # ← YAML을 site-packages 안에 설치
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/show_waypoints.launch.py']),
        # ↓↓↓ 이 줄은 제거 (share/..../waypoint_markers/waypoints.yaml로 가던 잘못된 설치 경로)
        # (f'share/{package_name}/{package_name}', [f'{package_name}/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiming',
    maintainer_email='seojimni@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'show_waypoints = waypoint_markers.markers_node:main',
    ]},
)
