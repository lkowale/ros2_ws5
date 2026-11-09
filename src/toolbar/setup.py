import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'toolbar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lkowale',
    maintainer_email='lkowale@gmail.com',
    description='Sliding toolbar controller node (solbot5 and solbot4 firmware)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'toolbar_service = toolbar.toolbar_service:main',
            'toolbar_outer_bridge = toolbar.toolbar_outer_bridge:main',
        ],
    },
)
