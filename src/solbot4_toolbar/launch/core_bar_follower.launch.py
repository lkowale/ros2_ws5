"""solbot4 (pd600) core stack + sliding toolbar + OAK-D + bar_follower, one launch.

  core_toolbar.launch.py   pd600_core (ros2_ws4) + toolbar_service + toolbar_outer_bridge
  oakd.launch.py           OAK-D RGB (depthai_ros_driver)            use_camera:=false to skip
  domain_bridge            camera JPEG + bar_follower overlay/status -> domain 1 (laptop)
  bar_follower             red bar -> /tool_bar_position_cmd          use_bar_follower:=false to skip
                           (config/toolbar_cam.yaml, starts dry_run; toolbar_service starts disabled)

Started by ros2_ws5/run_solbot4_bar_follower.sh (via ros2_ws4/run_core_um982.sh), which
exports CAMERA_BRIDGE_CYCLONEDDS_URI (domain 0 loopback <-> domain 1 tailscale0).
All core_toolbar / pd600_core launch arguments pass through.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('solbot4_toolbar')
    launch_dir = os.path.join(share, 'launch')
    use_camera = LaunchConfiguration('use_camera')
    use_bar_follower = LaunchConfiguration('use_bar_follower')

    args = [
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_bar_follower', default_value='true'),
        DeclareLaunchArgument('camera_model', default_value='OAK-D-LITE'),
        DeclareLaunchArgument(
            'bar_follower_params', default_value=os.path.join(share, 'config', 'toolbar_cam.yaml')),
    ]

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'core_toolbar.launch.py')))

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'oakd.launch.py')),
        launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items(),
        condition=IfCondition(use_camera))

    camera_bridge = Node(
        package='domain_bridge', executable='domain_bridge', name='camera_domain_bridge',
        arguments=[os.path.join(share, 'config', 'camera_bridge.yaml')],
        additional_env={'CYCLONEDDS_URI': EnvironmentVariable('CAMERA_BRIDGE_CYCLONEDDS_URI')},
        output='screen', condition=IfCondition(use_camera))

    bar_follower = Node(
        package='solbot4_toolbar', executable='bar_follower.py', name='bar_follower',
        parameters=[LaunchConfiguration('bar_follower_params')],
        output='screen', condition=IfCondition(use_bar_follower))

    return LaunchDescription(args + [core, camera, camera_bridge, bar_follower])
