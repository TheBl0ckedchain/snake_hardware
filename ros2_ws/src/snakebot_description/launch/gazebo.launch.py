"""Gazebo Harmonic simulation (ROS 2 Jazzy).

    ros2 launch snakebot_description gazebo.launch.py
    ros2 launch snakebot_description gazebo.launch.py rviz:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('snakebot_description')
    xacro_file = PathJoinSubstitution([FindPackageShare('snakebot_description'), 'urdf', 'snakebot.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' hw:=gz']), value_type=str)
    world = os.path.join(share, 'worlds', 'snake_flat.sdf')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items())

    def spawner(name, *extra):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager', *extra])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share)),
        gz,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'snakebot', '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']),
        spawner('joint_state_broadcaster'),
        spawner('snake_position_controller'),
        spawner('snake_trajectory_controller', '--inactive'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'config', 'display.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
