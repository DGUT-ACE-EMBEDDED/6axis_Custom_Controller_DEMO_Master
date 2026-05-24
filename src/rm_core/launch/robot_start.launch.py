import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # zyz_ik config
    zyz_ik_pkg = get_package_share_directory('zyz_ik')
    urdf_file = os.path.join(zyz_ik_pkg, 'urdf', '6axis_arm.urdf')
    arm_config = os.path.join(zyz_ik_pkg, 'config', 'arm.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # serial driver config
    serial_config = os.path.join(
        get_package_share_directory('rm_serial_driver'), 'config', 'serial_driver.yaml')

    # Serial driver node
    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[serial_config],
    )

    # IK solver node
    zyz_ik_solver_node = Node(
        package='zyz_ik',
        executable='zyz_ik_node',
        namespace='solver',
        output='screen',
        emulate_tty=True,
        parameters=[arm_config],
    )

    # Robot state publisher (URDF -> TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    return LaunchDescription([
        rm_serial_driver_node,
        zyz_ik_solver_node,
        robot_state_publisher_node,
        rviz_node,
    ])
