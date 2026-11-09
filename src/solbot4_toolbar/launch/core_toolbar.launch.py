"""solbot4 (pd600) core stack + sliding toolbar.

Everything in solbot4_hw_bringup/pd600_core.launch.py (from the ros2_ws4 underlay;
all its launch arguments pass straight through) plus toolbar_service from the
ros2_ws5 overlay, configured from solbot4_toolbar/config/toolbar.yaml.

Started by ros2_ws5/run_solbot4_toolbar.sh (via ros2_ws4/run_core_um982.sh).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('solbot4_hw_bringup'), 'launch', 'pd600_core.launch.py')))

    # also declared (same default) by pd600_core; declared here so toolbar_service
    # resolves it regardless of include scoping
    declare_broker_ip = DeclareLaunchArgument(
        'broker_ip', default_value=EnvironmentVariable('BROKER_IP', default_value='t630'),
        description='MQTT broker hostname or IP address')
    declare_toolbar_params = DeclareLaunchArgument(
        'toolbar_params',
        default_value=os.path.join(
            get_package_share_directory('solbot4_toolbar'), 'config', 'toolbar.yaml'),
        description='toolbar_service parameter file')

    toolbar_service = Node(
        package='toolbar', executable='toolbar_service', name='toolbar_service',
        output='screen',
        parameters=[LaunchConfiguration('toolbar_params'),
                    {'broker_ip_address': LaunchConfiguration('broker_ip')}])

    # phone MQTT app via HiveMQ cloud: outer/toolbar_pos {"data": mm}, outer/toolbar_cmd
    toolbar_outer_bridge = Node(
        package='toolbar', executable='toolbar_outer_bridge', name='toolbar_outer_bridge',
        output='screen')

    return LaunchDescription([
        declare_broker_ip,
        declare_toolbar_params,
        core,
        toolbar_service,
        toolbar_outer_bridge,
    ])
