import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mapper')
    rviz_config = os.path.join(share, 'launch', 'test.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([rviz_node])
