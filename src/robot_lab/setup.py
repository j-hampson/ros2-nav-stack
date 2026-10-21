from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Without these, launch files and worlds are never installed and
        # `ros2 launch robot_lab ...` cannot find them.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='theunholycorruptor@gmail.com',
    description='Gazebo mobile-robot exercises: closed-loop control, sensors, navigation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_square = robot_lab.drive_square:main',
            'scan_inspector = robot_lab.scan_inspector:main',
            'go_to_goal = robot_lab.go_to_goal:main',
            'avoid_to_goal = robot_lab.avoid_to_goal:main',
            'patrol = robot_lab.patrol:main',
        ],
    },
)
