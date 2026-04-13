from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch file for ouster OS2-64 LiDAR (copied from faster-lio repo)
    rviz = LaunchConfiguration('rviz')
    prefix = LaunchConfiguration('prefix')

    faster_lio_config = PathJoinSubstitution([
        FindPackageShare('faster_lio'), 'config', 'ouster64.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('faster_lio'), 'rviz_cfg', 'loam_livox.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('prefix', default_value='nice -n 0'),

        Node(
            package='faster_lio',
            executable='run_mapping_online',
            name='laserMapping',
            output='screen',
            prefix=prefix,
            parameters=[
                faster_lio_config,
                {
                    'feature_extract_enable': False,
                    'point_filter_num_': 4,
                    'max_iteration': 3,
                    'filter_size_surf': 0.5,
                    'filter_size_map': 0.5,
                    'cube_side_length': 1000.0,
                    'runtime_pos_log_enable': False,
                },
            ],
        ),

        GroupAction(
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz',
                    arguments=['-d', rviz_config],
                    prefix='nice',
                ),
            ],
            condition=IfCondition(rviz),
        ),
    ])
