from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_horizon = LaunchConfiguration('max_horizon')

    return LaunchDescription([
        DeclareLaunchArgument('max_horizon', default_value='5'),

        Node(
            package='state_machine',
            executable='path_replanner',
            name='replanner',
            output='screen',
            parameters=[{'max_horizon': max_horizon}],
            remappings=[
                ('~/epoch', 'epoch'),
                ('~/plan_path', 'tpplanner/plan_path'),
                ('~/execute_path', 'trackers_manager/execute_path'),
                ('~/position_cmd', 'position_cmd'),
            ],
        ),
    ])
