import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rm_serial_driver')

    config_a = os.path.join(pkg, 'config', 'device_a_driver.yaml')
    config_b = os.path.join(pkg, 'config', 'device_b_driver.yaml')

    device_a = Node(
        package='rm_serial_driver',
        executable='device_a_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_a],
    )

    device_b = Node(
        package='rm_serial_driver',
        executable='device_b_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_b],
    )

    return LaunchDescription([device_a, device_b])
