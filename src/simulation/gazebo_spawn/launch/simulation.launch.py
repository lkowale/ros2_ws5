"""Solbot4 Gazebo simulation launch file."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('gazebo_spawn')
    desc_dir = get_package_share_directory('description')
    launch_dir = os.path.join(sim_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    pose = {
        'x': LaunchConfiguration('x_pose', default='0.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.00'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw'),
    }

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start rviz',
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Run Gazebo headless (no GUI)'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'empty.sdf'),
        description='Full path to world model file to load',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='solbot5', description='Name of the robot'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(desc_dir, 'urdf', 'description.urdf'),
        description='Full path to robot URDF/xacro file',
    )

    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='0.00',
        description='Spawn yaw (rad): 0=East, pi/2=North, -pi/2=South'
    )

    # Robot state publisher
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'robot_description': Command(['xacro', ' ', robot_sdf])}
        ],
    )

    # RViz
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(desc_dir, 'rviz', 'config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gazebo world processing
    world_sdf = tempfile.mktemp(prefix='sim_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world]
    )

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_sdf]}.items(),
        condition=IfCondition(use_simulator)
    )

    # Cleanup temp file on shutdown
    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf) if os.path.exists(world_sdf) else None)
        ]))

    # Set Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(sim_dir, 'worlds')
    )

    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    # Spawn robot
    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_robot.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_simulator': use_simulator,
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'roll': pose['R'],
            'pitch': pose['P'],
            'yaw': pose['Y']
        }.items()
    )

    # Ackermann cmd_vel preprocessor - inverts angular.z when reversing
    # This fixes the Gazebo AckermannSteering plugin behavior for reverse motion
    ackermann_preprocessor = Node(
        package='gazebo_spawn',
        executable='ackermann_cmd_vel_preprocessor.py',
        name='ackermann_cmd_vel_preprocessor',
        namespace=namespace,
        output='screen',
    )

    # GPS odometry covariance injector - adds covariance to navsat_transform output
    # Needed because Gazebo GPS sensor may output zero covariance
    odom_covariance_injector = Node(
        package='gazebo_spawn',
        executable='odom_covariance_injector.py',
        name='odom_covariance_injector',
        namespace=namespace,
        output='screen',
        parameters=[{
            'pose_covariance_x': 0.0004,
            'pose_covariance_y': 0.0004,
        }],
    )

    # IMU covariance injector - adds covariance to Gazebo IMU output
    imu_covariance_injector = Node(
        package='gazebo_spawn',
        executable='imu_covariance_injector.py',
        name='imu_covariance_injector',
        namespace=namespace,
        output='screen',
        parameters=[{
            'orientation_covariance': 0.01,
            'angular_velocity_covariance': 0.01,
            'linear_acceleration_covariance': 0.1,
        }],
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add actions
    ld.add_action(set_env_vars_resources)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gz_robot)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(ackermann_preprocessor)
    ld.add_action(odom_covariance_injector)
    ld.add_action(imu_covariance_injector)

    return ld
