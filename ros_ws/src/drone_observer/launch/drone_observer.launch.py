from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    scenario_file = LaunchConfiguration('scenario_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    artifact_dir = LaunchConfiguration('artifact_dir')
    home_x = LaunchConfiguration('home_x')
    home_y = LaunchConfiguration('home_y')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude')
    landing_altitude = LaunchConfiguration('landing_altitude')
    acceleration_mps2 = LaunchConfiguration('acceleration_mps2')
    nav2_startup_delay = LaunchConfiguration('nav2_startup_delay')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='all'),
        DeclareLaunchArgument('scenario_file', default_value='/opt/drone-demo/config/scenarios.yaml'),
        DeclareLaunchArgument('nav2_params_file', default_value='/opt/drone-demo/config/nav2_params.yaml'),
        DeclareLaunchArgument('artifact_dir', default_value='/tmp/drone-artifacts'),
        DeclareLaunchArgument('home_x', default_value='10.0'),
        DeclareLaunchArgument('home_y', default_value='16.0'),
        DeclareLaunchArgument('takeoff_altitude', default_value='12.0'),
        DeclareLaunchArgument('landing_altitude', default_value='0.6'),
        DeclareLaunchArgument('acceleration_mps2', default_value='2.5'),
        DeclareLaunchArgument('nav2_startup_delay', default_value='5.0'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom', output='screen',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(package='drone_observer', executable='farm_map_publisher', name='farm_map_publisher', output='screen',
             parameters=[{'scenario_file': scenario_file}]),
        Node(package='drone_observer', executable='dynamic_obstacle_bridge', name='dynamic_obstacle_bridge', output='screen',
             parameters=[{'scenario_file': scenario_file}]),
        Node(package='drone_observer', executable='kinematic_drone', name='kinematic_drone', output='screen',
             parameters=[{'home_x': ParameterValue(home_x, value_type=float)},
                         {'home_y': ParameterValue(home_y, value_type=float)},
                         {'landing_altitude': ParameterValue(landing_altitude, value_type=float)},
                         {'acceleration_mps2': ParameterValue(acceleration_mps2, value_type=float)}]),
        Node(package='drone_observer', executable='nav2_cmd_vel_adapter', name='nav2_cmd_vel_adapter', output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py',
            ])),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'false',
                'params_file': nav2_params_file,
            }.items(),
        ),
        TimerAction(
            period=nav2_startup_delay,
            actions=[ExecuteProcess(
                cmd=[FindExecutable(name='ros2'), 'service', 'call',
                     '/lifecycle_manager_navigation/manage_nodes',
                     'nav2_msgs/srv/ManageLifecycleNodes', '{command: 0}'],
                output='screen',
            )],
        ),
        Node(package='drone_observer', executable='truth_publisher', name='truth_publisher', output='screen',
             parameters=[{'scenario_file': scenario_file}]),
        Node(package='drone_observer', executable='observer_node', name='observer_node', output='screen',
             parameters=[{'scenario': scenario}, {'scenario_file': scenario_file}, {'artifact_dir': artifact_dir},
                         {'home_x': ParameterValue(home_x, value_type=float)},
                         {'home_y': ParameterValue(home_y, value_type=float)},
                         {'takeoff_altitude': ParameterValue(takeoff_altitude, value_type=float)},
                         {'landing_altitude': ParameterValue(landing_altitude, value_type=float)}]),
    ])
