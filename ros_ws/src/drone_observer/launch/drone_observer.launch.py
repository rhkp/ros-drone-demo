from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    scenario_file = LaunchConfiguration('scenario_file')
    artifact_dir = LaunchConfiguration('artifact_dir')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='all'),
        DeclareLaunchArgument('scenario_file', default_value='/opt/drone-demo/config/scenarios.yaml'),
        DeclareLaunchArgument('artifact_dir', default_value='/tmp/drone-artifacts'),
        Node(package='drone_observer', executable='kinematic_drone', name='kinematic_drone', output='screen'),
        Node(package='drone_observer', executable='camera_simulator', name='camera_simulator', output='screen'),
        Node(package='drone_observer', executable='truth_publisher', name='truth_publisher', output='screen',
             parameters=[{'scenario_file': scenario_file}]),
        Node(package='drone_observer', executable='observer_node', name='observer_node', output='screen',
             parameters=[{'scenario': scenario}, {'scenario_file': scenario_file}, {'artifact_dir': artifact_dir}]),
    ])
