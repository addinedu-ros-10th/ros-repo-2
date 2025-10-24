from setuptools import setup, find_packages
from glob import glob

package_name = 'waypoint_director'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Select waypoint by id; cancel previous goal; RViz markers; /state publisher.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_director_node = waypoint_director.waypoint_director_node:main',
        ],
    },
)
