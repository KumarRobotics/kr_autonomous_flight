from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    takeoff_height = LaunchConfiguration('takeoff_height')
    max_horizon = LaunchConfiguration('max_horizon')

    smach_launch = PathJoinSubstitution([
        FindPackageShare('state_machine_launch'), 'launch', 'smach.launch.py'
    ])
    tracker_params = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('takeoff_height', default_value='5'),
        DeclareLaunchArgument('max_horizon', default_value='5'),

        GroupAction(actions=[
            PushRosNamespace(robot),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([smach_launch]),
                launch_arguments={'takeoff_height': takeoff_height}.items(),
            ),
            # motion primitive replanner
            Node(
                package='state_machine',
                executable='local_global_replan_server',
                name='local_global_replan_server',
                output='screen',
                parameters=[tracker_params],
                remappings=[
                    ('~/plan_local_trajectory', 'local_plan_server/plan_local_trajectory'),
                    ('~/plan_global_path', 'global_plan_server/plan_global_path'),
                    ('~/execute_trajectory', 'trackers_manager/execute_trajectory'),
                    ('~/global_path', 'global_plan_server/path'),
                    ('~/epoch', 'trackers_manager/epoch'),
                    ('~/position_cmd', 'position_cmd'),
                    ('~/local_voxel_map', 'mapper/local_voxel_map'),
                    ('~/replan_state_trigger', 'replan_state_trigger'),
                ],
            ),
        ]),
    ])
