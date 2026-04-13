import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    coverage_utils_share = get_package_share_directory('coverage_utils')
    default_input = os.path.join(coverage_utils_share, 'config', 'input.txt')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    )

    coverage_node = Node(
        package='coverage_utils',
        executable='coverage_utils_node',
        name='coverage_utils',
        output='screen',
        parameters=[{'input_file_path': default_input}],
    )

    return LaunchDescription([static_tf, coverage_node])
