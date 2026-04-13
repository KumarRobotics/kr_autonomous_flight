"""Legacy sensing-only launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapper_config = LaunchConfiguration('mapper_config')
    onboard_sensing = LaunchConfiguration('onboard_sensing')
    robot = LaunchConfiguration('robot')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    os1_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'old-launch', 'os1_robot_ns.launch.py'
    ])
    estimation_launch = PathJoinSubstitution([
        FindPackageShare('estimation_launch'), 'launch', 'estimation.launch.py'
    ])
    relay_ovc_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'relay_ovc.launch.py'
    ])
    map_plan_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'map_plan.launch.py'
    ])
    publish_tf_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'publish_tf.launch.py'
    ])
    msckf_calib = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'msckf_calib.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('onboard_sensing', default_value='true'),
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('vio_frame_id', default_value='vio/odom'),
        DeclareLaunchArgument('mass', default_value='2.85'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os1_launch]),
            launch_arguments={'robot': robot}.items(),
        ),

        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([estimation_launch]),
                    launch_arguments={'calibration_file': msckf_calib, 'robot': robot}.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([relay_ovc_launch]),
                    launch_arguments={'robot': robot}.items(),
                ),
            ],
            condition=IfCondition(onboard_sensing),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([map_plan_launch]),
            launch_arguments={
                'robot': robot,
                'cloud': 'os1_cloud_node/points',
                'mapper_config': mapper_config,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([publish_tf_launch]),
            launch_arguments={'robot': robot}.items(),
        ),
    ])
