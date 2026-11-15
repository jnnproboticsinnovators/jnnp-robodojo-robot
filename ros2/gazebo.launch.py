"""
One command to open Gazebo with the official RoboDojo game field
(roboticsdojo/internship2026 -> gamefield_software/src/gamefield) and the
robot already placed in it.

    ros2 launch jnnp_robodojo_robot gazebo.launch.py

Optional:
    rviz:=true            also open RViz
    world:=empty.sdf      load an empty world instead of the field
    x:= y:= z:= yaw:=     where to place the robot (default: start bay)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'jnnp_robodojo_robot'


def find_robot_file(share_dir):
    """Use the first .urdf.xacro / .xacro / .urdf file in the package's urdf folder."""
    urdf_dir = os.path.join(share_dir, 'urdf')
    files = sorted(os.listdir(urdf_dir)) if os.path.isdir(urdf_dir) else []
    for ending in ('.urdf.xacro', '.xacro', '.urdf'):
        for name in files:
            if name.endswith(ending):
                return os.path.join(urdf_dir, name)
    raise FileNotFoundError(f'No .urdf or .xacro file found in {urdf_dir}')


def generate_launch_description():
    share_dir = get_package_share_directory(PACKAGE)
    robot_file = find_robot_file(share_dir)
    # Official RoboDojo field from github.com/roboticsdojo/internship2026
    gamefield_root = os.path.expanduser('~/internship2026/gamefield_software/src/gamefield')
    gamefield_models = os.path.join(gamefield_root, 'models')
    field_world = os.path.join(gamefield_root, 'worlds', 'gamefield.world')
    if not os.path.isfile(field_world):              # fall back to our own copy
        field_world = os.path.join(share_dir, 'worlds', 'robodojo_arena.sdf')

    # xacro also accepts plain .urdf files, so this works for both
    robot_description = ParameterValue(Command(['xacro ', robot_file]), value_type=str)

    # Let Gazebo find package://jnnp_robodojo_robot/... mesh files
    resource_path = os.pathsep.join([os.path.dirname(share_dir), gamefield_models,
                                     os.environ.get('GZ_SIM_RESOURCE_PATH', '')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Wait a few seconds so Gazebo is ready, then place the robot
    spawn_robot = TimerAction(period=5.0, actions=[Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robodojo',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('yaw')],
        output='screen',
    )])

    # ROS 2 <-> Gazebo bridge: clock, /cmd_vel, /odom, /tf, /joint_states, /scan, /camera, /dump_cmd
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   # ROS teleop (teleop_twist_keyboard) -> Gazebo drive plugin
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   # wheel odometry from Gazebo -> ROS (for RViz / navigation)
                   '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   # odom -> base_footprint transform from the drive plugin
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   # wheel + back cabin angles (robot_state_publisher uses these)
                   '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                   # simulated RPLIDAR
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   # simulated Pi Camera Module 3 (picture + camera settings)
                   '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   # ROS -> Gazebo: back cabin (dump) angle in radians
                   '/dump_cmd@std_msgs/msg/Float64]gz.msgs.Double'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=field_world),
        # START: corner bay of the official field (next to the start block),
        # measured from Gamefield.stl. Floor is 0.05 m high, robot faces +y
        # (out of the bay, towards the loading zone).
        DeclareLaunchArgument('x', default_value='-2.05'),
        DeclareLaunchArgument('y', default_value='-1.12'),
        DeclareLaunchArgument('z', default_value='0.12'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('rviz', default_value='false'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        rviz,
    ])
