from setuptools import find_packages, setup

package_name = 'pause_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='lkowale@gmail.com',
    description='Pause manager node - central pause state supervision with severity-based behavior',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pause_manager = pause_manager.pause_manager:main',
        ],
    },
)
