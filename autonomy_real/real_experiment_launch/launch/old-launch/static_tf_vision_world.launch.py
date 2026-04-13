from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='stereo_rig_link_broadcaster',
            arguments=['0', '0', '0', '0', '0', '-0.7071068', '0.7071068', 'world', 'vision'],
        ),
    ])
