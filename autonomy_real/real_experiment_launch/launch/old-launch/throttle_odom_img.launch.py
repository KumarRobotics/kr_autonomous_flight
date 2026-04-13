from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    odom_topic = LaunchConfiguration('odom_topic')
    throttled_topic_high = LaunchConfiguration('throttled_topic_high')
    throttled_hz_high = LaunchConfiguration('throttled_hz_high')
    throttled_topic_low = LaunchConfiguration('throttled_topic_low')
    throttled_hz_low = LaunchConfiguration('throttled_hz_low')
    image_topic = LaunchConfiguration('image_topic')
    throttled_image_topic = LaunchConfiguration('throttled_image_topic')
    throttled_image_hz = LaunchConfiguration('throttled_image_hz')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('throttled_topic_high', default_value='odom_throttled_high'),
        DeclareLaunchArgument('throttled_hz_high', default_value='100'),
        DeclareLaunchArgument('throttled_topic_low', default_value='odom_throttled_low'),
        DeclareLaunchArgument('throttled_hz_low', default_value='0.2'),
        DeclareLaunchArgument('image_topic', default_value='image_processor/debug_stereo_image'),
        DeclareLaunchArgument('throttled_image_topic', default_value='image_processor/debug_stereo_image_throttled'),
        DeclareLaunchArgument('throttled_image_hz', default_value='10'),

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='topic_tools',
                executable='throttle',
                name='odom_throttler_high',
                arguments=['messages', odom_topic, throttled_hz_high, throttled_topic_high],
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                name='odom_throttler_low',
                arguments=['messages', odom_topic, throttled_hz_low, throttled_topic_low],
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                name='image_throttler',
                arguments=['messages', image_topic, throttled_image_hz, throttled_image_topic],
            ),
        ]),
    ])
