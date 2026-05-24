import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('zyz_ik')
    urdf_file = os.path.join(pkg_dir, 'urdf', '6axis_arm.urdf')
    config = os.path.join(pkg_dir, 'config', 'arm.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    zyz_ik_solver_node = Node(
        package='zyz_ik',
        executable='zyz_ik_node',
        namespace='solver',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ),

       

        zyz_ik_solver_node,

      

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
