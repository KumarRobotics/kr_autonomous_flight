from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    decoder_launch = PathJoinSubstitution([
        FindPackageShare('ouster_decoder'), 'launch', 'decoder.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('sensor_hostname', default_value='192.168.100.12'),
        DeclareLaunchArgument('udp_dest', default_value='192.168.100.1'),
        DeclareLaunchArgument('lidar_port', default_value='7502'),
        DeclareLaunchArgument('imu_port', default_value='7503'),
        DeclareLaunchArgument('replay', default_value='false'),
        DeclareLaunchArgument('lidar_mode', default_value='1024x10'),
        DeclareLaunchArgument('timestamp_mode', default_value=''),
        DeclareLaunchArgument('metadata', default_value='ouster_metadata.json'),
        DeclareLaunchArgument('viz', default_value='false'),
        DeclareLaunchArgument('tf_prefix', default_value=''),

        Node(
            package='ouster_decoder',
            executable='ouster_driver',
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([decoder_launch]),
            launch_arguments={'lidar_ns': robot, 'replay': 'false'}.items(),
        ),
    ])
