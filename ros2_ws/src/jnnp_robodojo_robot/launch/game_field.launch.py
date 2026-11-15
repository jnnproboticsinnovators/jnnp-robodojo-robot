# Game field: robot on the start pad, random cylinders, dynamic barrier, RViz labels.
import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

START_X, START_Y, START_YAW = 6.318, 4.186, -1.5708     # start corner at the back-right, facing the loading zone (-Y)
CYLINDER_SPOTS = [(round(u * 6.70356, 3), round(v * 4.868, 3)) for v in (0.14, 0.30, 0.46) for u in (0.12, 0.22, 0.32)]  # 3x3 grid
CYL_COLOURS = ['0.22 0.22 0.24 1', '0.22 0.22 0.24 1', '0.22 0.22 0.24 1']   # dark tops, as in Figure 2


def cylinder_sdf(name, top_rgba):
    return f'''<sdf version="1.8"><model name="{name}"><static>true</static><link name="l">
  <collision name="c"><pose>0 0 0.21 0 0 0</pose><geometry><cylinder><radius>0.14</radius><length>0.42</length></cylinder></geometry></collision>
  <visual name="body"><pose>0 0 0.21 0 0 0</pose><geometry><cylinder><radius>0.14</radius><length>0.42</length></cylinder></geometry>
    <material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material></visual>
  <visual name="top"><pose>0 0 0.423 0 0 0</pose><geometry><cylinder><radius>0.141</radius><length>0.006</length></cylinder></geometry>
    <material><ambient>{top_rgba}</ambient><diffuse>{top_rgba}</diffuse></material></visual>
</link></model></sdf>'''


def sim_and_slam(context):
    pkg = get_package_share_directory('jnnp_robodojo_robot')
    use_slam = LaunchConfiguration('slam').perform(context).lower() in ('true', '1', 'yes')
    rviz_cfg = os.path.join(pkg, 'rviz', 'sim_slam.rviz' if use_slam else 'sim.rviz')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': 'game_field.sdf', 'x': str(START_X), 'y': str(START_Y),
                          'yaw': str(START_YAW), 'rviz_config': rviz_cfg}.items(),
    )]
    if use_slam:
        # start mapping once Gazebo, the bridge and the clock are up
        actions.append(TimerAction(period=8.0, actions=[
            LogInfo(msg='Starting SLAM (slam_toolbox) - RViz Fixed Frame is "map"'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')))]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('jnnp_robodojo_robot')
    sim = OpaqueFunction(function=sim_and_slam)

    # 3: judges place cylinders randomly -> pick 3 of the marked spots each launch
    spots = random.sample(CYLINDER_SPOTS, 3)
    spawn_cylinders = [
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-string', cylinder_sdf(f'cylinder_{i}', CYL_COLOURS[i]), '-name', f'cylinder_{i}',
                        '-x', str(x), '-y', str(y), '-z', '0.0'])
        for i, (x, y) in enumerate(spots)
    ]

    # 2: random time-based dynamic barrier
    barrier = Node(package='jnnp_robodojo_robot', executable='dynamic_barrier.py', output='screen',
                   parameters=[{'use_sim_time': True}])

    # 'field' frame (Gazebo world coordinates) relative to odom (= robot start pose)
    c, s = math.cos(-START_YAW), math.sin(-START_YAW)
    tx, ty = c * -START_X - s * -START_Y, s * -START_X + c * -START_Y
    field_tf = Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
                    arguments=['--x', f'{tx:.4f}', '--y', f'{ty:.4f}', '--yaw', f'{-START_YAW:.4f}',
                               '--frame-id', 'odom', '--child-frame-id', 'field'],
                    parameters=[{'use_sim_time': True}])

    markers = Node(package='jnnp_robodojo_robot', executable='field_markers.py', output='screen',
                   parameters=[{'layout_file': os.path.join(pkg, 'config', 'field_layout.yaml'),
                                'cylinders': [float(v) for xy in spots for v in xy],
                                'use_sim_time': True}])

    # point the Gazebo camera at the field from the same corner as Figure 2
    camera_view = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
             '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
             '--req', 'pose: {position: {x: 9.6, y: -4.2, z: 6.4}, orientation: {x: -0.2816, y: 0.1186, z: 0.8775, w: 0.3696}}'],
        output='log')

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='Start SLAM mapping too (true/false)'),
        LogInfo(msg=f'Cylinders placed at: {spots}'),
        sim,
        field_tf,
        markers,
        # give Gazebo time to load the world before adding cylinders and starting the barrier
        TimerAction(period=6.0, actions=spawn_cylinders + [barrier]),
        TimerAction(period=9.0, actions=[camera_view]),
    ])
