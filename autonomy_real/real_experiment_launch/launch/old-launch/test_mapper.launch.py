"""Legacy test_mapper launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapper_config = LaunchConfiguration('mapper_config')
    robot = LaunchConfiguration('robot')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    map_plan_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'map_plan.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('robot', default_value='quadrotor'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([map_plan_launch]),
            launch_arguments={
                'robot': robot,
                'cloud': 'os1_cloud_node/points',
                'mapper_config': mapper_config,
            }.items(),
        ),
    ])
