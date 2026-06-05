from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'solbot5_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='lkowale@gmail.com',
    description='Control nodes for solbot robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # M1 minimal-localization set. Other solbot4 nodes (mqtt_op,
            # lift_service, planter_service, heading_fuser, gps_vel_odom, …)
            # are intentionally omitted until later milestones add them back
            # along with solbot5_msgs.
            'drive = solbot5_control.drive:main',
            'steering = solbot5_control.steering:main',
            'imu_bridge = solbot5_control.imu_bridge:main',
            'navsat_init = solbot5_control.navsat_init:main',
        ],
    },
)
