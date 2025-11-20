from setuptools import setup

package_name = 'teleop_gui_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_gui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='Teleop GUI using PyQt',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'teleop_gui = teleop_gui_pkg.teleop_subscriber_gui:main',
            'teleop_keyboard = teleop_gui_pkg.teleop_keyboard_node:main',
        ],
    },
)
