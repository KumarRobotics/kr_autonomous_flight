from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    mass = LaunchConfiguration('mass')
    simulation = LaunchConfiguration('simulation')
    gains_file = LaunchConfiguration('gains_file')
    manager = LaunchConfiguration('manager')

    default_gains_file = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'so3_control_gains.yaml'
    ])
    mav_manager_params = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'mav_manager_params.yaml'
    ])
    trackers_mp_yaml = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'trackers_mp.yaml'
    ])
    tracker_params_mp_yaml = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])
    real_trackers_yaml = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'trackers.yaml'
    ])
    real_tracker_params_yaml = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'tracker_params.yaml'
    ])
    real_gains_yaml = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'so3_control_gains.yaml'
    ])

    # Nodelet manager converted to ComposableNodeContainer hosting trackers + so3_control
    sim_container = ComposableNodeContainer(
        name=manager,
        namespace=robot,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='kr_trackers_manager',
                plugin='kr_trackers_manager::TrackersManager',
                name='trackers_manager',
                parameters=[trackers_mp_yaml, tracker_params_mp_yaml, {'mass': mass}],
                remappings=[
                    ('~/odom', 'odom'),
                    ('~/cmd', 'position_cmd'),
                    ('~/des_max', 'des_max'),
                ],
            ),
            ComposableNode(
                package='kr_mav_controllers',
                plugin='kr_mav_controllers::SO3ControlComponent',
                name='so3_control',
                parameters=[gains_file, {'mass': mass}],
                remappings=[
                    ('~/odom', 'odom'),
                    ('~/position_cmd', 'position_cmd'),
                    ('~/so3_cmd', 'so3_cmd'),
                    ('~/motors', 'motors'),
                ],
            ),
        ],
        output='screen',
        condition=IfCondition(simulation),
    )

    real_container = ComposableNodeContainer(
        name=manager,
        namespace=robot,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='kr_trackers_manager',
                plugin='kr_trackers_manager::TrackersManager',
                name='trackers_manager',
                parameters=[real_trackers_yaml, real_tracker_params_yaml, {'mass': mass}],
                remappings=[
                    ('~/odom', 'odom'),
                    ('~/cmd', 'position_cmd'),
                    ('~/des_max', 'des_max'),
                ],
            ),
            ComposableNode(
                package='kr_mav_controllers',
                plugin='kr_mav_controllers::SO3ControlComponent',
                name='so3_control',
                parameters=[real_gains_yaml, {'mass': mass}],
                remappings=[
                    ('~/odom', 'odom'),
                    ('~/position_cmd', 'position_cmd'),
                    ('~/so3_cmd', 'so3_cmd'),
                    ('~/motors', 'motors'),
                ],
            ),
        ],
        output='screen',
        condition=UnlessCondition(simulation),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value='quadrotor'),
        ),
        DeclareLaunchArgument('mass', default_value='1.74'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('max_prop_force', default_value='7'),
        DeclareLaunchArgument('gains_file', default_value=default_gains_file),
        DeclareLaunchArgument('start_nodelet_manager', default_value='true'),
        DeclareLaunchArgument('manager', default_value=[robot, '_manager_control']),

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='kr_mav_manager',
                executable='mav_services',
                name='mav_services',
                output='screen',
                parameters=[mav_manager_params, {'mass': mass}],
            ),
        ]),

        sim_container,
        real_container,

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='kr_trackers',
                executable='twist_to_velocity_goal.py',
                name='twist_to_velocity',
            ),
        ]),
    ])
