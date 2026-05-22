import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #config = os.path.join(
       # get_package_share_directory('mecanum'), 'config', 'chassis_params.yaml')

    omni_solver_node = Node(
        package='omni',
        executable='omni_node',
        namespace='solver',
        output='screen',
        emulate_tty=True,
        #parameters=[config],
    )

    return LaunchDescription([omni_solver_node])
