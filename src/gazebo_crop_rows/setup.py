from setuptools import setup

package_name = 'gazebo_crop_rows'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'crop_row_spawner = gazebo_crop_rows.crop_row_spawner:main',
        ],
    },
)
