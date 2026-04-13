"""Legacy ouster OS1 viz-only launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    udp_dest = LaunchConfiguration('udp_dest')
    lidar_port = LaunchConfiguration('lidar_port')
    imu_port = LaunchConfiguration('imu_port')
    replay = LaunchConfiguration('replay')
    lidar_mode = LaunchConfiguration('lidar_mode')
    timestamp_mode = LaunchConfiguration('timestamp_mode')
    metadata = LaunchConfiguration('metadata')
    viz = LaunchConfiguration('viz')
    rviz_config = LaunchConfiguration('rviz_config')
    tf_prefix = LaunchConfiguration('tf_prefix')

    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('ouster_ros'), 'viz.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('sensor_hostname', default_value='192.168.100.12'),
        DeclareLaunchArgument('udp_dest', default_value='192.168.100.1'),
        DeclareLaunchArgument('lidar_port', default_value='7502'),
        DeclareLaunchArgument('imu_port', default_value='7503'),
        DeclareLaunchArgument('replay', default_value='true'),
        DeclareLaunchArgument('lidar_mode', default_value='2048x10'),
        DeclareLaunchArgument('timestamp_mode', default_value=''),
        DeclareLaunchArgument('metadata', default_value='ouster_metadata.json'),
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('tf_prefix', default_value=''),

        Node(
            package='ouster_ros',
            executable='os_node',
            name='os_node',
            output='screen',
            parameters=[{
                'lidar_mode': lidar_mode,
                'timestamp_mode': timestamp_mode,
                'replay': replay,
                'sensor_hostname': sensor_hostname,
                'udp_dest': udp_dest,
                'lidar_port': lidar_port,
                'imu_port': imu_port,
                'metadata': metadata,
            }],
        ),

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='ouster_ros',
                executable='os_cloud_node',
                name='os_cloud_node',
                output='screen',
                parameters=[{'tf_prefix': tf_prefix}],
                remappings=[
                    ('~/os_config', '/os_node/os_config'),
                    ('~/lidar_packets', '/os_node/lidar_packets'),
                    ('~/imu_packets', '/os_node/imu_packets'),
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(viz),
            ),
            Node(
                package='ouster_ros',
                executable='img_node',
                name='img_node',
                output='screen',
                condition=IfCondition(viz),
                remappings=[
                    ('~/os_config', '/os_node/os_config'),
                    ('~/points', '/os_cloud_node/points'),
                ],
            ),
            Node(
                package='topic_tools',
                executable='relay',
                name='relay_lidar',
                arguments=['/os1_node/lidar_packets', '/os_node/lidar_packets'],
            ),
            Node(
                package='topic_tools',
                executable='relay',
                name='relay_imu',
                arguments=['/os1_node/imu_packets', '/os_node/imu_packets'],
            ),
        ]),
    ])
