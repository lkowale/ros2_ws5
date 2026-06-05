"""Dual EKF localization launch for solbot4.

Local EKF  (odom -> base_footprint): wheel odometry + IMU differential
Global EKF (map  -> odom):           GPS position + GPS velocity + local EKF output

Topic mapping:
    /odom              — local EKF output (Nav2, gps_vel_odom, etc. subscribe here)
    /odometry/global   — global EKF output
    /odometry/gps      — navsat_transform output (GPS in local frame)

This launch file is designed to be restartable independently of the core
hardware stack. All sensor nodes (drive, imu_bridge, ublox, gps_vel_odom,
ackermann_odom) run in pd600_core.launch.py and keep running when this
is restarted.

Usage:
    ros2 launch solbot5_localization dual_ekf.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('solbot5_localization')
    params_file = os.path.join(pkg_dir, 'config', 'dual_ekf_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    # NavSat Transform — converts GPS lat/lon to odometry in local frame
    # Reads from local EKF (/odom). Uses IMU orientation (not odometry yaw)
    # to establish ENU→local rotation.
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data', 'imu'),
            ('gps/fix', 'gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odom'),
        ],
    )

    # Local EKF — odom -> base_footprint (publishes /odom)
    # Fuses: wheel odometry (vx, vyaw) + IMU differential (yaw, vyaw)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom'),
                    ('set_pose', 'set_pose_local')],
    )

    # Global EKF — map -> odom
    # Fuses: GPS position + GPS velocity + local EKF output (vx, vyaw)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global'),
                    ('set_pose', 'set_pose_global')],
    )

    return LaunchDescription([
        declare_use_sim_time,
        navsat_transform_node,
        ekf_local_node,
        ekf_global_node,
    ])
