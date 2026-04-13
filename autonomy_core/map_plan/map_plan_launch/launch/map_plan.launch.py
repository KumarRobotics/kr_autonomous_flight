from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    real_robot = LaunchConfiguration('real_robot')
    odom = LaunchConfiguration('odom')
    lidar_frame = LaunchConfiguration('lidar_frame')
    mapper_config = LaunchConfiguration('mapper_config')
    planner_config = LaunchConfiguration('planner_config')
    cloud = LaunchConfiguration('cloud')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    default_planner_config = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])
    mapper_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'mapper.launch.py'
    ])
    mp_planner_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'mp_planner.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value='quadrotor'),
        ),
        DeclareLaunchArgument('real_robot', default_value='false'),
        DeclareLaunchArgument('use_mp', default_value='true'),
        DeclareLaunchArgument('odom', default_value='odom'),
        DeclareLaunchArgument('lidar_frame', default_value=''),
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('planner_config', default_value=default_planner_config),
        DeclareLaunchArgument('cloud', default_value='lidar'),

        GroupAction(actions=[
            PushRosNamespace(robot),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([mapper_launch]),
                launch_arguments={
                    'odom_frame': [robot, '/odom'],
                    'map_frame': [robot, '/map'],
                    'lidar_frame': lidar_frame,
                    'real_robot': real_robot,
                    'mapper_config': mapper_config,
                    'cloud': cloud,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([mp_planner_launch]),
                launch_arguments={
                    'planner_config': planner_config,
                    'odom': odom,
                }.items(),
            ),
        ]),
    ])
