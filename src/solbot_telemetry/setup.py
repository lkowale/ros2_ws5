from setuptools import find_packages, setup

package_name = 'solbot_telemetry'

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
    maintainer='Lukasz Kowalewski',
    maintainer_email='lkowale@gmail.com',
    description='Diagnostic reporting mixin shared by components that report conditions to PauseManager',
    license='Apache-2.0',
    tests_require=['pytest'],
)
