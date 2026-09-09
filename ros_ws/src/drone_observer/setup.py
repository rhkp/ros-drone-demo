from glob import glob
from setuptools import setup

package_name = 'drone_observer'

setup(
    name=package_name,
    version='0.1.2',
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
        ],
    },
)
