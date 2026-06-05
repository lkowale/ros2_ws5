"""solbot5 EKF localization — GPS position + dual-antenna heading.

Nodes:
- relposned_heading : UBXNavRelPosNED -> /imu/gps_heading (absolute yaw)
- navsat_transform  : /gps/fix -> /odometry/gps
- ekf_filter_node_odom : fuse GPS odom + IMU yaw-rate + dual-antenna heading
- static map->odom identity (single EKF, no global filter)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('solbot5_localization')
    params_file = os.path.join(pkg_dir, 'config', 'ekf_relposned_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    heading_offset_deg = LaunchConfiguration('heading_offset_deg')
    gps_odom_topic = LaunchConfiguration('gps_odom_topic')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_heading_offset_cmd = DeclareLaunchArgument(
        'heading_offset_deg', default_value='0.0',
        description='Antenna-baseline mounting offset, calibrated in sim')

    # In sim, navsat output is post-processed by odom_covariance_injector
    # (odometry/gps_raw -> odometry/gps), so navsat publishes to gps_raw there.
    # On the real robot the EKF reads odometry/gps directly.
    declare_gps_odom_topic_cmd = DeclareLaunchArgument(
        'gps_odom_topic', default_value='odometry/gps',
        description='navsat_transform odometry output topic (use odometry/gps_raw in sim)')

    relposned_heading_node = Node(
        package='solbot5_localization',
        executable='relposned_heading.py',
        name='relposned_heading',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'heading_offset_deg': heading_offset_deg,
        }],
    )

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
            ('odometry/gps', gps_odom_topic),
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
            ('imu1/data', 'imu/gps_heading'),
        ],
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_heading_offset_cmd,
        declare_gps_odom_topic_cmd,
        relposned_heading_node,
        navsat_transform_node,
        ekf_node,
        map_to_odom_tf,
    ])
