from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_dir = LaunchConfiguration('dir')

    topics = [
        '/Odometry',
        '/os_node/imu_packets',
        '/os_node/lidar_packets',
        '/os_node/metadata',
        '/ovc/vectornav/imu',
        '/ovc/left/image_raw',
        '/ovc/right/image_raw',
        '/ovc/rgb/image_raw',
        '/tf',
        '/tf_static',
        '/quadrotor/odom',
        '/diagnostics',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('dir', default_value='/home/dcist/bags/atl_falcon_pennovation'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_dir, *topics],
            output='screen',
        ),
    ])
