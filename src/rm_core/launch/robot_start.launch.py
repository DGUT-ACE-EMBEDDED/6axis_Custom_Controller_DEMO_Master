import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_core'), 'config', 'robot_start.yaml')

    rm_chassis_board_node = Node(
        package='rm_core',
        executable='rm_core_chassis_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    

    return LaunchDescription([rm_chassis_board_node])
