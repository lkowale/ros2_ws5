"""solbot5 EKF localization — GPS position + dual-antenna heading.

Self-contained: this launch owns everything needed to (re)start localization
without touching the hardware/sim layer, so it can be relaunched standalone
(e.g. run_localization.sh restart). In particular navsat_init lives here, not
in the core/sim launch — navsat_transform comes up with wait_for_datum=true and
needs /datum set on every fresh start, which a co-launched navsat_init handles.

Nodes:
- relposned_heading : UBXNavRelPosNED -> /imu/gps_heading (absolute yaw)
- navsat_transform  : /gps/fix -> /odometry/gps  (waits for datum)
- navsat_init       : sets /datum from first GPS fix (re-sets on each restart)
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
    pkg_dir = get_package_share_directory('localization')
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

    declare_gps_odom_topic_cmd = DeclareLaunchArgument(
        'gps_odom_topic', default_value='odometry/gps',
        description='navsat_transform odometry output topic (use odometry/gps_raw in sim)')

    relposned_heading_node = Node(
        package='localization',
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
            ('odometry/filtered', 'odom'),
            ('odometry/gps', gps_odom_topic),
        ],
    )

    navsat_init_node = Node(
        package='control',
        executable='navsat_init',
        name='navsat_init',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
        navsat_init_node,
        ekf_node,
        map_to_odom_tf,
    ])
