from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='drone'),
        Node(
            package='topic_tools',
            executable='relay',
            name='ovc_relay',
            arguments=['/ovc/left/image_raw', [robot, '/cam_left/image_raw']],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='ovc_relay2',
            arguments=['/ovc/right/image_raw', [robot, '/cam_right/image_raw']],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='ovc_relay3',
            arguments=['/ovc/vectornav', [robot, '/imu']],
        ),
    ])
