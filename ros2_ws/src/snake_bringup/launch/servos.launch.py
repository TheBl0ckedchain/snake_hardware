"""Start the Dynamixel joint driver. Torque stays OFF until:
    ros2 service call /dxl_node/enable std_srvs/srv/SetBool "{data: true}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('snake_driver'), 'config', 'dxl.yaml')
    return LaunchDescription([
        Node(package='snake_driver', executable='dxl_node', name='dxl_node',
             parameters=[cfg], output='screen'),
    ])
