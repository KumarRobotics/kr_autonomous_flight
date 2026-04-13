from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapper_config = LaunchConfiguration('mapper_config')
    onboard_sensing = LaunchConfiguration('onboard_sensing')
    use_rgbd = LaunchConfiguration('use_rgbd')
    robot = LaunchConfiguration('robot')
    depth_cam = LaunchConfiguration('depth_cam')
    takeoff_height = LaunchConfiguration('takeoff_height')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    estimation_launch = PathJoinSubstitution([
        FindPackageShare('estimation_launch'), 'launch', 'estimation.launch.py'
    ])
    control_launch = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'launch', 'control.launch.py'
    ])
    depth_to_cloud_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'depth_to_cloud.launch.py'
    ])
    map_plan_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'map_plan.launch.py'
    ])
    state_machine_launch = PathJoinSubstitution([
        FindPackageShare('state_machine_launch'), 'launch', 'state_machine.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('onboard_sensing', default_value='false'),
        DeclareLaunchArgument('use_rgbd', default_value='false'),
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('depth_cam', default_value='DepthCamera'),
        DeclareLaunchArgument('takeoff_height', default_value='5'),

        # S-MSCKF VIO
        GroupAction(
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource([estimation_launch]))],
            condition=IfCondition(onboard_sensing),
        ),

        # Controller
        IncludeLaunchDescription(PythonLaunchDescriptionSource([control_launch])),

        # Use RGBD
        GroupAction(
            actions=[
                GroupAction(actions=[
                    PushRosNamespace([robot, '/', depth_cam]),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource([depth_to_cloud_launch])),
                ]),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([map_plan_launch]),
                    launch_arguments={
                        'cloud': [depth_cam, '/points'],
                        'mapper_config': mapper_config,
                        'robot': robot,
                    }.items(),
                ),
            ],
            condition=IfCondition(use_rgbd),
        ),

        # Use LIDAR
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([map_plan_launch]),
                    launch_arguments={
                        'cloud': 'lidar',
                        'mapper_config': mapper_config,
                        'robot': robot,
                    }.items(),
                ),
            ],
            condition=UnlessCondition(use_rgbd),
        ),

        # State machine
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_machine_launch]),
            launch_arguments={'takeoff_height': takeoff_height}.items(),
        ),
    ])
