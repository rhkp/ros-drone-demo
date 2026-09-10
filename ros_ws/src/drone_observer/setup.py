from glob import glob
from setuptools import setup

package_name = 'drone_observer'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'kinematic_drone = drone_observer.kinematic_drone:main',
            'observer_node = drone_observer.observer_node:main',
            'truth_publisher = drone_observer.truth_publisher:main',
            'farm_map_publisher = drone_observer.farm_map_publisher:main',
            'dynamic_obstacle_bridge = drone_observer.dynamic_obstacle_bridge:main',
            'nav2_cmd_vel_adapter = drone_observer.nav2_cmd_vel_adapter:main',
        ],
    },
)
