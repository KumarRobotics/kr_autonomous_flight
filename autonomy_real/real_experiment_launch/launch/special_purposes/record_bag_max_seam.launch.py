from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_dir = LaunchConfiguration('dir')

    topics = [
        '/quadrotor/vio/odom',
        '/tf',
        '/tf_static',
        '/rosout',
        '/rosout_agg',
        '/os_node/imu_packets',
        '/os_node/lidar_packets',
        '/os_node/metadata',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('dir', default_value='/home/dcist/bags/falcon_pennovation_max_seam'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_dir, *topics],
            output='screen',
        ),
    ])
