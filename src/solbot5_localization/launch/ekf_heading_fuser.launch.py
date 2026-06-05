"""EKF localization using heading_fuser as single IMU source.

heading_fuser publishes /imu/fused_heading — absolute yaw maintained by
integrating BNO080 yaw rate, with periodic correction from GPS heading.
The EKF uses this as a non-differential absolute source, so no imu1 needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('solbot5_localization')
    params_file = os.path.join(pkg_dir, 'config', 'ekf_heading_fuser_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    gps_roll_correction_node = Node(
        package='solbot5_localization',
        executable='gps_roll_correction_node.py',
        name='gps_roll_correction',
        output='screen',
        parameters=[{'antenna_height': 0.97, 'enable_roll_correction': True}],
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data', 'imu/fused_heading'),
            ('gps/fix', 'gps/fix_corrected'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odom'),
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odom'),
        ],
    )

    # Static identity transform map -> odom (single EKF, no global filter)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gps_roll_correction_node,
        navsat_transform_node,
        ekf_node,
        map_to_odom_tf,
    ])
