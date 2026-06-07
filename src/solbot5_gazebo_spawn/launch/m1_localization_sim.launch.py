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

    # Correct /gps/fix from Gazebo ground truth (the gz NavSat sensor's lat/lon
    # is rotated vs world ENU, so it is not bridged — see solbot5_gz_bridge.yaml).
    sim_gps_fix = Node(
        package='solbot5_gazebo_spawn',
        executable='sim_gps_fix_publisher.py',
        name='sim_gps_fix_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'datum_lat': 53.5204991,
            'datum_lon': 17.8258532,
            'datum_alt': 100.0,
            # base_footprint now coincides with the front antenna (gps_link), so
            # the GPS is AT base_footprint — zero lever-arm.
            'antenna_x': 0.0,
            'antenna_y': 0.0,
            'rate_hz': 10.0,
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

    # Localization (delayed to let Gazebo + bridge publish first messages).
    # navsat_init now lives inside ekf_relposned.launch.py so it re-sets the
    # datum on a standalone localization restart.
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
        sim_gps_fix,
        sim_relposned,
        localization_cmd,
    ])
