from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    takeoff_height = LaunchConfiguration('takeoff_height')
    node_start_delay = LaunchConfiguration('node_start_delay')

    tracker_params = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('takeoff_height', default_value='3'),
        DeclareLaunchArgument('node_start_delay', default_value='20'),

        Node(
            package='state_machine',
            executable='main_state_machine.py',
            name='main_state_machine',
            output='screen',
            # Delay startup with bash prefix to mimic the ROS1 launch-prefix
            prefix=['bash -c "sleep ', node_start_delay, '; $0 $@" '],
            parameters=[
                tracker_params,
                {'takeoff_height': takeoff_height},
            ],
            remappings=[
                ('~/odom', 'odom'),
                ('waypoints_done', 'done'),
                ('~/height', 'mavros/distance_sensor/lidarlite_pub'),
                ('~/goal_pose', 'goal_pose'),
            ],
        ),
    ])
