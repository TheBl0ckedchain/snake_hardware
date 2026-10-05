"""Full real-robot bringup: URDF -> TF of every segment, Dynamixel driver (torque OFF), lidar, Foxglove.

    ros2 launch snake_bringup robot.launch.py [lidar:=false] [foxglove:=false]

Torque stays off until:  ros2 service call /dxl_node/enable std_srvs/srv/SetBool "{data: true}"
Emergency torque off:    python3 ~/snake_hardware/scripts/torque_off.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('snake_bringup')
    xacro_file = PathJoinSubstitution([FindPackageShare('snakebot_description'), 'urdf', 'snakebot.urdf.xacro'])
    # The ros2_control block is unused here (dxl_node drives the servos); mock keeps xacro free of hardware args.
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' hw:=mock']), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        IncludeLaunchDescription(PathJoinSubstitution([bringup, 'launch', 'servos.launch.py'])),
        # The URDF already places base_link -> lidar -> laser, so no static TF from the lidar launch.
        IncludeLaunchDescription(
            PathJoinSubstitution([bringup, 'launch', 'lidar.launch.py']),
            launch_arguments={'static_tf': 'false', 'foxglove': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('lidar'))),
        # Headless Pi: view everything from the laptop in Foxglove at ws://<pi-ip>:8765.
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             parameters=[{'port': 8765}], condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
