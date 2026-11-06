"""OAK-D camera on solbot4 (pd600): depthai_ros_driver, RGB only (config/oakd.yaml).

Topics: /oakd/rgb/image_raw, /oakd/rgb/image_raw/compressed, /oakd/rgb/camera_info.
Started by ros2_ws5/run_solbot4_camera.sh (pd600.sh camera), together with a
domain_bridge that forwards the compressed stream to the laptop.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('solbot4_toolbar')
    declare_model = DeclareLaunchArgument(
        'camera_model', default_value='OAK-D-LITE', description='OAK-D, OAK-D-LITE, ...')
    declare_params = DeclareLaunchArgument(
        'camera_params', default_value=os.path.join(share, 'config', 'oakd.yaml'))

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')),
        launch_arguments={
            'name': 'oakd',
            'camera_model': LaunchConfiguration('camera_model'),
            'params_file': LaunchConfiguration('camera_params'),
        }.items())

    return LaunchDescription([declare_model, declare_params, driver])
