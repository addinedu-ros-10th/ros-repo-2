from setuptools import find_packages, setup

package_name = 'teleop_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pjh',
    maintainer_email='wlstnz3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_keyboard_node = teleop_keyboard.teleop_keyboard_node:main',
            'teleop_key_sub = teleop_keyboard.teleop_key_sub:main',
        ],
    },
)
