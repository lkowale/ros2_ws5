"""toolbar_service alone (default: solbot5 config). params:=<file> to override."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(
            get_package_share_directory('toolbar'), 'config', 'toolbar_solbot5.yaml')),
        Node(package='toolbar', executable='toolbar_service', name='toolbar_service',
             output='screen', parameters=[LaunchConfiguration('params')]),
    ])
