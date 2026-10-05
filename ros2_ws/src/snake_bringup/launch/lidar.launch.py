"""Start the YDLidar G2 driver. With static_tf:=true (default, for standalone use) it publishes an
identity base_link -> laser transform; robot.launch.py turns that off because the URDF places the laser."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params')
    static_tf = LaunchConfiguration('static_tf')
    foxglove = LaunchConfiguration('foxglove')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('snake_bringup'), 'config', 'ydlidar_g2.yaml'])),
        DeclareLaunchArgument('static_tf', default_value='true'),
        # Headless Pi: view /scan from the laptop in Foxglove at ws://<pi-ip>:8765.
        DeclareLaunchArgument('foxglove', default_value='true'),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[params],
            # The SDK occasionally aborts during its startup intensity auto-detect; just restart it.
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'laser'],
            condition=IfCondition(static_tf),
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765}],
            condition=IfCondition(foxglove),
        ),
    ])
