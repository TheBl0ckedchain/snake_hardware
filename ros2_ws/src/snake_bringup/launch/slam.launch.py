"""2D SLAM with slam_toolbox (online async) on /scan.

    ros2 launch snake_bringup slam.launch.py [use_sim_time:=true]

There is no odometry yet, so odom -> base_link is a static identity and slam_toolbox's scan matcher
tracks the head alone (fine at snake speeds). Phase 3 replaces this with virtual-chassis odometry
fused with the IMU (robot_localization).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration('use_sim_time')
    cfg = PathJoinSubstitution([FindPackageShare('snake_bringup'), 'config', 'slam_toolbox.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='odom_to_base_placeholder',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
             parameters=[{'use_sim_time': sim}]),
        # The stock launch configures and activates the (lifecycle) slam_toolbox node.
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
            launch_arguments={'slam_params_file': cfg, 'use_sim_time': sim}.items()),
    ])
