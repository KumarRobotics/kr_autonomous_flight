from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prefix = LaunchConfiguration('prefix')
    odom = LaunchConfiguration('odom')
    planner_config = LaunchConfiguration('planner_config')

    default_planner_config = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])

    return LaunchDescription([
        # use taskset to hard-assign threads to avoid computation spikes
        DeclareLaunchArgument('prefix', default_value='taskset -c 1,2,3'),
        DeclareLaunchArgument('odom', default_value='odom'),
        DeclareLaunchArgument('planner_config', default_value=default_planner_config),

        # planning server
        Node(
            package='action_planner',
            executable='global_plan_server',
            name='global_plan_server',
            output='screen',
            prefix=prefix,
            parameters=[planner_config],
            remappings=[
                ('~/odom', odom),
                ('~/global_voxel_map', 'mapper/global_voxel_map'),
            ],
        ),
        Node(
            package='action_planner',
            executable='local_plan_server',
            name='local_plan_server',
            output='screen',
            prefix=prefix,
            respawn=True,
            respawn_delay=10.0,
            parameters=[planner_config],
            remappings=[
                ('~/local_voxel_map', 'mapper/local_voxel_map'),
                ('~/local_noinfla_voxel_map', 'mapper/local_voxel_no_inflation_map'),
            ],
        ),
    ])
