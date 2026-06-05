"""Dual EKF + NavSat localization launch for solbot4.

Uses robot_localization standard node names (ekf_filter_node_odom / ekf_filter_node_map)
matching the dual_ekf_navsat_example.yaml structure.

Local EKF  (odom -> base_footprint): wheel odometry + IMU yaw rate
Global EKF (map  -> odom):           wheel + IMU + GPS position + GPS velocity

Usage:
    ros2 launch solbot5_localization dual_ekf_navsat.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('solbot5_localization')
    params_file = os.path.join(pkg_dir, 'config', 'dual_ekf_navsat_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

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

    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom'),
                    ('set_pose', 'set_pose_local')],
    )

    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global'),
                    ('set_pose', 'set_pose_global'),
                    ('imu1/data', 'imu/gps_heading')],
    )

    gps_to_map_node = Node(
        package='solbot_control',
        executable='gps_to_map',
        name='gps_to_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        navsat_transform_node,
        ekf_odom_node,
        ekf_map_node,
        gps_to_map_node,
    ])
