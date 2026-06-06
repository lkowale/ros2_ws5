"""M2 — Nav2 drive sim for solbot5.

M1 localization sim (Gazebo + dual-antenna localization) + the Nav2 stack, so
the robot can navigate to goals. Nav2 is delayed to let Gazebo, TF and the EKF
publish first.

cmd_vel chain: Nav2 controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel
-> ackermann_cmd_vel_preprocessor -> cmd_vel_ackermann -> Gazebo.

Usage:
    ros2 launch solbot5_gazebo_spawn m2_nav_sim.launch.py
    HEADLESS:=False  to show the Gazebo GUI

Send a goal (new terminal):
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0}, \\
       orientation: {w: 1.0}}}}"
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

    # M1 localization sim (Gazebo + robot + dual-antenna localization).
    m1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'm1_localization_sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'heading_offset_deg': heading_offset_deg,
        }.items(),
    )

    # Mapviz origin — solbot4's proven recipe (NOT swri initialize_origin):
    #  - origin_publisher publishes /local_xy_origin from the first GPS fix;
    #    Mapviz's tile_map plugin does the wgs84 conversion off that topic.
    #  - a static map->origin TF anchors the 'origin' frame Mapviz expects.
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

    # Nav2 — delayed to let Gazebo, TF and EKF come up and publish map->odom->base.
    nav2_cmd = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_composition': 'True',
            }.items(),
        )]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_headless_cmd,
        declare_heading_offset_cmd,
        m1_cmd,
        origin_publisher,
        static_tf_map_origin,
        nav2_cmd,
    ])
