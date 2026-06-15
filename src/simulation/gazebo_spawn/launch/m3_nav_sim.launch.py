"""M3 — Reeds-Shepp planner sim for solbot5.

M1 localization sim + Nav2 with the custom ReedsSheppPlanner instead of
NavfnPlanner. Plans minimum-length forward+reverse arc/straight paths that
respect the Ackermann minimum turning radius.

Usage:
    ros2 launch gazebo_spawn m3_nav_sim.launch.py
    headless:=False  to show the Gazebo GUI

Send a goal (new terminal):
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0}, \\
       orientation: {z: 0.707, w: 0.707}}}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('gazebo_spawn')
    nav2_dir = get_package_share_directory('solbot5_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    heading_offset_deg = LaunchConfiguration('heading_offset_deg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Run Gazebo without GUI')
    declare_heading_offset_cmd = DeclareLaunchArgument(
        'heading_offset_deg', default_value='0.0')

    bt_xml = os.path.join(
        os.path.expanduser('~'), 'ros2_ws5', 'src',
        'navigation', 'nav2_bringup', 'behavior_trees', 'navigate_plan_once.xml')
    log_bt = LogInfo(msg=f'[M3] BT XML: {bt_xml}')

    m1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'm1_localization_sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'heading_offset_deg': heading_offset_deg,
        }.items(),
    )

    origin_publisher = Node(
        package='solbot5_nav2_bringup',
        executable='origin_publisher.py',
        name='origin_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_map_origin = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_transform_map_origin',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'origin'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav2_cmd = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_composition': 'True',
                'params_file': os.path.join(
                    nav2_dir, 'params', 'nav2_params_m3.yaml'),
            }.items(),
        )]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_headless_cmd,
        declare_heading_offset_cmd,
        log_bt,
        m1_cmd,
        origin_publisher,
        static_tf_map_origin,
        nav2_cmd,
    ])
