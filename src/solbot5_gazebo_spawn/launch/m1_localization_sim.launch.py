"""M1 — minimal localization sim for solbot5.

Brings up Gazebo + the robot + the localization path only (no Nav2). Validates
the dual-antenna heading pipeline end-to-end:

    Gazebo ground truth ─► sim_relposned_publisher ─► /ubx_nav_rel_pos_ned
                                                          │
                            relposned_heading ◄───────────┘
                                   │
                            /imu/gps_heading ─► EKF ◄─ /odometry/gps, /imu (yaw rate)

Drive the robot with teleop / cmd_vel and compare EKF odom yaw against Gazebo
ground truth to calibrate heading_offset_deg.

Usage:
    ros2 launch solbot5_gazebo_spawn m1_localization_sim.launch.py
    HEADLESS:=False  to show the Gazebo GUI
    heading_offset_deg:=<deg>  to apply a calibration offset
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('solbot5_gazebo_spawn')
    loc_dir = get_package_share_directory('solbot5_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    heading_offset_deg = LaunchConfiguration('heading_offset_deg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Run Gazebo without GUI')
    declare_heading_offset_cmd = DeclareLaunchArgument(
        'heading_offset_deg', default_value='0.0',
        description='Antenna-baseline mounting offset to calibrate')

    # Gazebo + robot + gz bridge + covariance injectors + ackermann preprocessor.
    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
        }.items(),
    )

    sim_wheel_speed_pub = Node(
        package='solbot5_gazebo_spawn',
        executable='sim_wheel_speed_publisher.py',
        name='sim_wheel_speed_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_radius': 0.20,
            'gear_ratio': 1.0,
        }],
    )

    # Fake dual-antenna RELPOSNED from Gazebo ground truth.
    sim_relposned = Node(
        package='solbot5_gazebo_spawn',
        executable='sim_relposned_publisher.py',
        name='sim_relposned_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sim_heading_offset_deg': 0.0,
            'heading_noise_deg': 0.0,
            'rate_hz': 8.0,
        }],
    )

    ackermann_odom_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='solbot5_control',
            executable='ackermann_odom',
            name='ackermann_odom',
            output='both',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wheelbase': 1.20,
                'track_width': 0.77,
                'wheel_diameter': 0.40,
                'gear_ratio': 1.0,
                'publish_rate': 20.0,
            }],
        )]
    )

    navsat_init_node = Node(
        package='solbot5_control',
        executable='navsat_init',
        name='navsat_init',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_speed_mps': 0.0,
        }],
    )

    # Localization (delayed to let Gazebo + bridge publish first messages).
    localization_cmd = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_dir, 'launch', 'ekf_relposned.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'heading_offset_deg': heading_offset_deg,
                'gps_odom_topic': 'odometry/gps_raw',
            }.items(),
        )]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_headless_cmd,
        declare_heading_offset_cmd,
        simulation_cmd,
        sim_wheel_speed_pub,
        sim_relposned,
        ackermann_odom_node,
        navsat_init_node,
        localization_cmd,
    ])
