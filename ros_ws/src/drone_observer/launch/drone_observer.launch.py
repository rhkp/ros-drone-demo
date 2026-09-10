from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    scenario_file = LaunchConfiguration('scenario_file')
    artifact_dir = LaunchConfiguration('artifact_dir')
    home_x = LaunchConfiguration('home_x')
    home_y = LaunchConfiguration('home_y')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude')
    landing_altitude = LaunchConfiguration('landing_altitude')
    acceleration_mps2 = LaunchConfiguration('acceleration_mps2')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='all'),
        DeclareLaunchArgument('scenario_file', default_value='/opt/drone-demo/config/scenarios.yaml'),
        DeclareLaunchArgument('artifact_dir', default_value='/tmp/drone-artifacts'),
        DeclareLaunchArgument('home_x', default_value='10.0'),
        DeclareLaunchArgument('home_y', default_value='16.0'),
        DeclareLaunchArgument('takeoff_altitude', default_value='12.0'),
        DeclareLaunchArgument('landing_altitude', default_value='0.6'),
        DeclareLaunchArgument('acceleration_mps2', default_value='2.5'),
        Node(package='drone_observer', executable='kinematic_drone', name='kinematic_drone', output='screen',
             parameters=[{'home_x': ParameterValue(home_x, value_type=float)},
                         {'home_y': ParameterValue(home_y, value_type=float)},
                         {'landing_altitude': ParameterValue(landing_altitude, value_type=float)},
                         {'acceleration_mps2': ParameterValue(acceleration_mps2, value_type=float)}]),
        Node(package='drone_observer', executable='truth_publisher', name='truth_publisher', output='screen',
             parameters=[{'scenario_file': scenario_file}]),
        Node(package='drone_observer', executable='observer_node', name='observer_node', output='screen',
             parameters=[{'scenario': scenario}, {'scenario_file': scenario_file}, {'artifact_dir': artifact_dir},
                         {'home_x': ParameterValue(home_x, value_type=float)},
                         {'home_y': ParameterValue(home_y, value_type=float)},
                         {'takeoff_altitude': ParameterValue(takeoff_altitude, value_type=float)},
                         {'landing_altitude': ParameterValue(landing_altitude, value_type=float)}]),
    ])
