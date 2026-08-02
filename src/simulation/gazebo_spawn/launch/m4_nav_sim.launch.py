"""M4 — Vision-guided tool slider sim for solbot5.

M1 localization sim + Nav2 (RS planner) + tool slider simulation nodes:
  - plant_row_detector_sim: fake row offset publisher
  - tool_slider_controller: commands toolbar position from row offset
  - tool_actuator_sim: first-order lag actuator + TF tool_bar link

Usage:
    ros2 launch gazebo_spawn m4_nav_sim.launch.py
    headless:=False  to show the Gazebo GUI
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FIELDS_DIR = os.path.join(os.path.expanduser('~'), 'ros2_ws5', 'src', 'fields')
FIELD_FILE  = os.path.join(
    FIELDS_DIR, 'house_short',
    'house_short_lines_0.77m_directed_turns.geojson')


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
    log_bt = LogInfo(msg=f'[M4] BT XML: {bt_xml}')

    m1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'm1_localization_sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'heading_offset_deg': heading_offset_deg,
            'world': os.path.join(sim_dir, 'worlds', 'house_short_rows.sdf'),
        }.items(),
    )

    origin_publisher = Node(
        package='solbot5_nav2_bringup',
        executable='origin_publisher.py',
        name='origin_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_pose_publisher = Node(
        package='solbot5_nav2_bringup',
        executable='tf_pose_publisher.py',
        name='tf_pose_publisher',
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
        period=20.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_composition': 'False',
                'params_file': os.path.join(
                    nav2_dir, 'params', 'nav2_params_m4.yaml'),
            }.items(),
        )]
    )

    plant_row_detector = Node(
        package='tool_slider',
        executable='plant_row_detector_sim',
        name='plant_row_detector_sim',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mode': 'sine',
            'sine_amplitude': 0.04,
            'sine_period_s': 8.0,
        }],
    )

    tool_slider_controller = Node(
        package='tool_slider',
        executable='tool_slider_controller',
        name='tool_slider_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cal_offset_m': 0.0,
            'slider_limit_m': 0.10,
            'row_lost_timeout_s': 0.5,
        }],
    )

    tool_actuator_sim = Node(
        package='tool_slider',
        executable='tool_actuator_sim',
        name='tool_actuator_sim',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'time_constant_s': 0.3,
            'slider_limit_m': 0.10,
            'update_rate_hz': 50.0,
        }],
    )

    pause_manager = Node(
        package='pause_manager',
        executable='pause_manager',
        name='pause_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': 8080,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn all crop row boxes at startup, keep them for the whole run.
    # Delayed 15 s to let /fromLL and EKF settle.
    crop_row_spawner = TimerAction(
        period=15.0,
        actions=[Node(
            package='gazebo_crop_rows',
            executable='crop_row_spawner',
            name='crop_row_spawner',
            output='screen',
            parameters=[{
                'field_file':        FIELD_FILE,
                'world_name':        'house_short_crop_rows',
                'spawn_all':         True,
                'row_offset_m':      0.18,
                'row_width_m':       0.06,
                'segment_length_m':  2.0,
            }],
        )]
    )

    gz_rl_recorder = TimerAction(
        period=15.0,
        actions=[Node(
            package='gazebo_crop_rows',
            executable='gz_rl_recorder',
            name='gz_rl_recorder',
            output='screen',
            parameters=[{
                'out_csv': '/tmp/gz_rl_record17.csv',
                'img_dir': '/tmp/gz_rl_frames17',
                'rate_hz': 4.0,
            }],
        )]
    )

    # Send run_field goal after Nav2 is fully active (Nav2 timer=20s + 50s for activation)
    send_field_goal = TimerAction(
        period=70.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'action', 'send_goal', '/run_field',
                'solbot5_msgs/action/RunField',
                '{field_name: house_short, start_line_index: 0}',
            ],
            output='screen',
        )]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_headless_cmd,
        declare_heading_offset_cmd,
        log_bt,
        m1_cmd,
        origin_publisher,
        tf_pose_publisher,
        static_tf_map_origin,
        pause_manager,
        nav2_cmd,
        plant_row_detector,
        tool_slider_controller,
        tool_actuator_sim,
        web_video_server,
        # crop_row_spawner disabled: boxes occlude QR floor from downward camera
        gz_rl_recorder,
        send_field_goal,
    ])
