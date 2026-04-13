from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    odom_topic = LaunchConfiguration('odom_topic')
    throttled_odom_topic = LaunchConfiguration('throttled_odom_topic')
    throttled_odom_hz = LaunchConfiguration('throttled_odom_hz')
    global_map_topic = LaunchConfiguration('global_map_topic')
    local_map_topic = LaunchConfiguration('local_map_topic')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('throttled_odom_topic', default_value='odom_throttled'),
        DeclareLaunchArgument('throttled_odom_hz', default_value='60'),
        DeclareLaunchArgument('global_map_topic', default_value='mapper/global_voxel_map'),
        DeclareLaunchArgument('local_map_topic', default_value='mapper/local_voxel_map'),

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='topic_tools',
                executable='throttle',
                name='debug_stereo_throttler',
                arguments=['messages', 'image_processor/debug_stereo_image', '5', 'image_processor/debug_stereo_image_throttled'],
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                name='odom_throttler',
                arguments=['messages', odom_topic, throttled_odom_hz, throttled_odom_topic],
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                name='local_map_throttler',
                arguments=['messages', local_map_topic, '0.5', [local_map_topic, '_throttled']],
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                name='global_map_throttler',
                arguments=['messages', global_map_topic, '0.1', [global_map_topic, '_throttled']],
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                name='rgb_throttler',
                arguments=['messages', '/ovc/rgb/image_raw', '10', '/ovc/rgb/image_raw_throttled'],
            ),
        ]),
    ])
