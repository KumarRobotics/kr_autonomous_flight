import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    viz_ns = LaunchConfiguration('viz_ns')

    perspective_file = PathJoinSubstitution([
        FindPackageShare('client_launch'), 'config', 'client_gui.perspective'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('client_launch'), 'rviz', 'client.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value='quadrotor'),
        ),
        # remap topic ns to viz_ns arg to enable display with variable robot names
        # if this is changed from default, you will also have to change the corresponding .rviz
        DeclareLaunchArgument('viz_ns', default_value='robot'),

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='rqt_gui',
                executable='rqt_gui',
                name='rqt_gui_buttons',
                output='screen',
                arguments=['--perspective-file', perspective_file],
                remappings=[
                    ('robot', robot),
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rviz_config, '-f', [robot, '/map']],
                respawn=False,
                remappings=[
                    (['/', viz_ns, '/voxel_map'], 'voxel_map'),
                    (['/', viz_ns, '/trackers_manager/sg'], 'trackers_manager/sg'),
                    (['/', viz_ns, '/tpplanner/path_array'], 'tpplanner/path_array'),
                    (['/', viz_ns, '/odom'], 'odom'),
                ],
            ),
        ]),
    ])
