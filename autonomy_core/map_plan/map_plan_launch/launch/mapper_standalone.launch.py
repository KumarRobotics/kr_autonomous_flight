from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapper_config = LaunchConfiguration('mapper_config')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])

    mapper_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'mapper.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([mapper_launch]),
            launch_arguments={
                'odom_frame': 'map',
                'map_frame': 'map',
                'mapper_config': mapper_config,
                'cloud': 'cloud',
            }.items(),
        ),
    ])
