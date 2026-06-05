"""Spawn solbot5 robot in Gazebo and set up ROS-Gazebo bridges."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _make_spawn_node(context, *args, **kwargs):
    """Evaluate URDF at launch time and spawn via -string to avoid TRANSIENT_LOCAL CycloneDDS issues."""
    desc_dir = get_package_share_directory('solbot5_description')

    use_simulator = context.launch_configurations.get('use_simulator', 'True')
    if use_simulator.lower() not in ('true', '1'):
        return []

    robot_name = context.launch_configurations.get('robot_name', 'solbot5')
    x = context.launch_configurations.get('x_pose', '0.00')
    y = context.launch_configurations.get('y_pose', '0.00')
    z = context.launch_configurations.get('z_pose', '0.01')
    R = context.launch_configurations.get('roll', '0.00')
    P = context.launch_configurations.get('pitch', '0.00')
    Y = context.launch_configurations.get('yaw', '0.00')

    urdf_path = os.path.join(desc_dir, 'urdf', 'solbot5_description.urdf')
    urdf_xml = subprocess.check_output(['xacro', urdf_path]).decode()

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-string', urdf_xml,
            '-x', x, '-y', y, '-z', z,
            '-R', R, '-P', P, '-Y', Y,
        ],
        parameters=[{'use_sim_time': False}],
    )
    return [spawn_model]


def generate_launch_description():
    sim_dir = get_package_share_directory('solbot5_gazebo_spawn')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True', description='Whether to start the simulator'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='solbot5', description='Name of the robot'
    )
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf', default_value='', description='Unused — URDF is read directly from solbot5_description'
    )
    declare_x_pose_cmd = DeclareLaunchArgument('x_pose', default_value='0.00')
    declare_y_pose_cmd = DeclareLaunchArgument('y_pose', default_value='0.00')
    declare_z_pose_cmd = DeclareLaunchArgument('z_pose', default_value='0.01')
    declare_roll_cmd = DeclareLaunchArgument('roll', default_value='0.00')
    declare_pitch_cmd = DeclareLaunchArgument('pitch', default_value='0.00')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.00')

    # ROS-Gazebo bridge — must NOT use sim time; it is the /clock source
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        namespace=namespace,
        parameters=[{
            'config_file': os.path.join(sim_dir, 'configs', 'solbot5_gz_bridge.yaml'),
            'use_sim_time': False,
        }],
        output='screen',
    )

    # Spawn robot — URDF evaluated at launch time and passed as -string to avoid
    # CycloneDDS AllowMulticast=false TRANSIENT_LOCAL delivery failure on -topic.
    spawn_model = OpaqueFunction(function=_make_spawn_node)

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(bridge)
    ld.add_action(spawn_model)

    return ld
