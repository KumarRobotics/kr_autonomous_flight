import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('multi_mav_launch'), 'rviz', 'config.rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_config)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([rviz_config_arg, rviz_node])
