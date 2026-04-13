"""Legacy LLOL lidar odometry launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug = LaunchConfiguration('debug')
    tbb = LaunchConfiguration('tbb')
    log = LaunchConfiguration('log')
    vis = LaunchConfiguration('vis')
    rigid = LaunchConfiguration('rigid')
    odom_frame = LaunchConfiguration('odom_frame')

    llol_cfg = PathJoinSubstitution([
        FindPackageShare('llol'), 'config', 'llol.yaml'
    ])
    debug_conf = PathJoinSubstitution([
        FindPackageShare('llol'), 'launch', 'debug.conf'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('tbb', default_value='0'),
        DeclareLaunchArgument('log', default_value='0'),
        DeclareLaunchArgument('vis', default_value='false'),
        DeclareLaunchArgument('rigid', default_value='true'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),

        SetEnvironmentVariable(
            name='ROSCONSOLE_CONFIG_FILE',
            value=debug_conf,
            condition=IfCondition(debug),
        ),

        Node(
            package='llol',
            executable='sv_node_llol',
            name='llol_odom',
            namespace='quadrotor',
            output='screen',
            parameters=[
                llol_cfg,
                {
                    'tbb': tbb,
                    'log': log,
                    'vis': vis,
                    'rigid': rigid,
                    'odom_frame': odom_frame,
                },
            ],
            remappings=[
                ('~/imu', 'imu'),
                ('~/image', 'image'),
                ('~/camera_info', 'camera_info'),
            ],
        ),
    ])
