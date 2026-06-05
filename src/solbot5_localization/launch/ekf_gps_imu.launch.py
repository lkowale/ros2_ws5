"""EKF localization launch file for solbot4.

Fuses GPS and IMU for odom->base_footprint transform using:
- navsat_transform_node: Converts GPS lat/lon to odometry in local frame
- ekf_node: Fuses GPS odometry and IMU data
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('solbot5_localization')
    params_file = os.path.join(pkg_dir, 'config', 'ekf_gps_imu_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # NavSat Transform Node - converts GPS to odometry coordinates
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

    # EKF Node - fuses GPS position + IMU yaw rate + GPS velocity heading
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odom'),
            ('imu1/data', 'imu/gps_heading'),
        ],
    )

    # Static identity transform map -> odom (single EKF has no global filter)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        navsat_transform_node,
        ekf_node,
        map_to_odom_tf,
    ])
