"""ros2_control bringup for the real snake (or mock hardware for testing).

    # no hardware, controllers run against mock joints
    ros2 launch snakebot_description bringup.launch.py
    # real Dynamixels on the U2D2
    ros2 launch snakebot_description bringup.launch.py hw:=real usb_port:=/dev/ttyUSB0 baud_rate:=57600
    # add rviz:=true to watch it
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('snakebot_description')
    xacro_file = PathJoinSubstitution([FindPackageShare('snakebot_description'), 'urdf', 'snakebot.urdf.xacro'])
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' hw:=', LaunchConfiguration('hw'),
        ' usb_port:=', LaunchConfiguration('usb_port'),
        ' baud_rate:=', LaunchConfiguration('baud_rate'),
    ]), value_type=str)
    controllers = os.path.join(share, 'config', 'controllers.yaml')

    def spawner(name, *extra):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager', *extra])

    return LaunchDescription([
        DeclareLaunchArgument('hw', default_value='mock', description='mock | real'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='57600'),
        DeclareLaunchArgument('rviz', default_value='false'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers], output='screen',
             remappings=[('~/robot_description', '/robot_description')]),
        spawner('joint_state_broadcaster'),
        spawner('snake_position_controller'),
        spawner('snake_trajectory_controller', '--inactive'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'config', 'display.rviz')]),
    ])
