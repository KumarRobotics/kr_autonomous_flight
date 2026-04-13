from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cloud = LaunchConfiguration('cloud')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    real_robot = LaunchConfiguration('real_robot')
    mapper_config = LaunchConfiguration('mapper_config')
    prefix = LaunchConfiguration('prefix')

    return LaunchDescription([
        DeclareLaunchArgument('cloud', default_value='lidar'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        # lidar_frame arg is only used when real_robot is true, otherwise the mapper
        # will just use the same frame as the point cloud in simulator
        DeclareLaunchArgument('lidar_frame', default_value=''),
        DeclareLaunchArgument('real_robot', default_value='false'),
        DeclareLaunchArgument('mapper_config'),
        # hard assigning threads to avoid computation spikes
        DeclareLaunchArgument('prefix', default_value='taskset -c 0'),

        # use nice to set low priority for mapper
        Node(
            package='mapper',
            executable='local_global_mapper',
            name='mapper',
            output='screen',
            prefix=prefix,
            parameters=[
                mapper_config,
                {
                    'real_robot': real_robot,
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'lidar_frame': lidar_frame,
                },
            ],
            remappings=[
                ('~/cloud', cloud),
                ('~/voxel_map', 'voxel_map'),
                ('~/local_cloud', 'local_cloud'),
            ],
        ),
    ])
