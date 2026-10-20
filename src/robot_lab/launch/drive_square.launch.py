"""Bring up Gazebo, the ROS<->gz bridge, and the drive_square controller.

    ros2 launch robot_lab drive_square.launch.py
    ros2 launch robot_lab drive_square.launch.py robot:=vehicle_green
    ros2 launch robot_lab drive_square.launch.py gz_args:='-s -r diff_drive.sdf'   # headless

Gazebo and ROS use separate transports, so the bridge in the middle is what
makes the robot reachable from ROS at all. The controller is written against
plain /cmd_vel and /odom and remapped onto this robot's topics here, which is
why it needs no changes to drive a different robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')

    declare_robot = DeclareLaunchArgument(
        'robot', default_value='vehicle_blue',
        description='Model name in the world; its topics are /model/<robot>/...')
    declare_gz_args = DeclareLaunchArgument(
        'gz_args', default_value='-r diff_drive.sdf',
        description="Passed to gz sim. '-r' runs immediately; add '-s' for headless.")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items())

    # Only these two topics are translated. Everything else Gazebo publishes
    # stays invisible to ROS until it is listed here.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            ['/model/', robot, '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            ['/model/', robot, '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        ])

    controller = Node(
        package='robot_lab', executable='drive_square', output='screen',
        remappings=[
            ('/cmd_vel', ['/model/', robot, '/cmd_vel']),
            ('/odom', ['/model/', robot, '/odometry']),
        ])

    return LaunchDescription(
        [declare_robot, declare_gz_args, gazebo, bridge, controller])
