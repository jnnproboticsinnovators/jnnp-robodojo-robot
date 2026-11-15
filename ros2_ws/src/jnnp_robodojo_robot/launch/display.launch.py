import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('jnnp_robodojo_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'jnnp_robodojo_robot.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Publishes angles for the 4 continuous wheel joints and the back cabin
    # hinge. Without this node those 5 links have no TF and do not render.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
