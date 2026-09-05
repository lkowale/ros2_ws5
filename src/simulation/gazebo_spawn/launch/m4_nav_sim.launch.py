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
from launch.conditions import IfCondition, UnlessCondition
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
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Run Gazebo without GUI')
    declare_heading_offset_cmd = DeclareLaunchArgument(
        'heading_offset_deg', default_value='0.0')
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.00',
        description='Gazebo world-frame spawn X (== map/EKF frame in sim)')
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.00',
        description='Gazebo world-frame spawn Y (== map/EKF frame in sim)')
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='0.00',
        description='Gazebo world-frame spawn yaw [rad]')
    use_vision_detector = LaunchConfiguration('use_vision_detector')
    declare_use_vision_detector_cmd = DeclareLaunchArgument(
        'use_vision_detector', default_value='false',
        description='true: crop_row_vision_node (real camera-based row/implement '
                     'detection). false (default, M4): plant_row_detector_sim '
                     '(fake sine-wave offset).')
    vision_debug_publish = LaunchConfiguration('vision_debug_publish')
    declare_vision_debug_publish_cmd = DeclareLaunchArgument(
        'vision_debug_publish', default_value='false',
        description='crop_row_vision_node: publish annotated debug image on '
                     'crop_row_vision/debug_image')

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
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
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
        condition=UnlessCondition(use_vision_detector),
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

    crop_row_vision = Node(
        condition=IfCondition(use_vision_detector),
        package='crop_row_vision',
        executable='crop_row_vision_node',
        name='crop_row_vision_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_height_m': 0.80,
            'camera_hfov_rad': 1.414,
            'debug_publish': vision_debug_publish,
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
            # PID gains tuned live against crop_row_vision_node + M5 field nav
            # (see tool_slider_logger data): kp=0.5 (old feedforward-equivalent
            # default) saturated the slider every tick and self-oscillated.
            # kp=0.15/ki=0.02/kd=0.02 tracked a real ~0.09m offset swing with
            # zero limit-saturation and zero oscillation; kp=0.4 reproduced the
            # oscillation (16 sign flips / 200 samples, 44 at the limit), so
            # this is a deliberately conservative margin below that. Tune live
            # with `ros2 param set /tool_slider_controller kp <val>` etc.
            'kp': 0.15,
            'ki': 0.02,
            'kd': 0.02,
            'integral_limit_m': 0.05,
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
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_yaw_cmd,
        declare_use_vision_detector_cmd,
        declare_vision_debug_publish_cmd,
        log_bt,
        m1_cmd,
        origin_publisher,
        tf_pose_publisher,
        static_tf_map_origin,
        pause_manager,
        nav2_cmd,
        plant_row_detector,
        crop_row_vision,
        tool_slider_controller,
        tool_actuator_sim,
        web_video_server,
        # crop_row_spawner disabled: boxes occlude QR floor from downward camera
        gz_rl_recorder,
        send_field_goal,
    ])
