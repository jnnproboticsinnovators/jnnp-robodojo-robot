# RViz-only drive mode: no Gazebo. Drive with `teleop`, press d to dump the back cabin.
# The robot starts on the start pad of the game field drawing (walls are not solid here).
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

START_X, START_Y, START_YAW = 6.318, 4.186, -1.5708   # same start pad as game_field.launch.py


def generate_launch_description():
    pkg = get_package_share_directory('jnnp_robodojo_robot')
    with open(os.path.join(pkg, 'urdf', 'jnnp_robodojo_robot.urdf')) as f:
        robot_description = f.read()

    c, s = math.cos(-START_YAW), math.sin(-START_YAW)
    tx, ty = c * -START_X - s * -START_Y, s * -START_X + c * -START_Y
    show_field = LaunchConfiguration('field')

    return LaunchDescription([
        DeclareLaunchArgument('field', default_value='true', description='Draw the game field under the robot'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='jnnp_robodojo_robot', executable='fake_robot.py', output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', output='log',
             arguments=['--x', f'{tx:.4f}', '--y', f'{ty:.4f}', '--yaw', f'{-START_YAW:.4f}',
                        '--frame-id', 'odom', '--child-frame-id', 'field'],
             condition=IfCondition(show_field)),
        Node(package='jnnp_robodojo_robot', executable='field_markers.py', output='log',
             parameters=[{'layout_file': os.path.join(pkg, 'config', 'field_layout.yaml')}],
             condition=IfCondition(show_field)),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'drive.rviz')]),
    ])
