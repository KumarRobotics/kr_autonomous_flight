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
    # NOTE: the upstream ROS1 system.launch included
    # control_launch/launch/control.launch — a generic (non-motion-primitive)
    # controller launch that was DELETED in Dec 2021 (commit f8c2f1b "clean
    # up control_launch") along with its companion configs trackers.yaml and
    # tracker_params.yaml. The ROS2 port inherits this broken reference.
    #
    # control_launch today only ships control_mp.launch.py, which uses the
    # motion-primitive tracker configs (trackers_mp.yaml, tracker_params_mp.yaml).
    # Silently redirecting the include to control_mp.launch.py is a semantic
    # change (non-MP controller -> MP controller) that the original author
    # did not intend, so the include below is commented out rather than
    # auto-substituted. If you want the MP controller, uncomment and change
    # the filename to control_mp.launch.py; if you want the original non-MP
    # controller, you will need to resurrect control.launch + trackers.yaml +
    # tracker_params.yaml from the pre-f8c2f1b tree.
    control_launch = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'launch', 'control_mp.launch.py'
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
        # IncludeLaunchDescription(PythonLaunchDescriptionSource([control_launch])),
        # ^ disabled: the original 'control.launch' (non-motion-primitive) no
        # longer exists; see the control_launch note above. Uncomment and
        # change the variable definition to point at control_mp.launch.py
        # if you want the motion-primitive controller loaded automatically.

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
