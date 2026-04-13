from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapper_config = LaunchConfiguration('mapper_config')
    onboard_sensing = LaunchConfiguration('onboard_sensing')
    robot = LaunchConfiguration('robot')
    mass = LaunchConfiguration('mass')
    odom_topic = LaunchConfiguration('odom_topic')
    record_bag = LaunchConfiguration('record_bag')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    drivers_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'drivers_for_faster_lio.launch.py'
    ])
    px4_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'px4.launch.py'
    ])
    publish_tf_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'publish_tf.launch.py'
    ])
    system_mp_launch = PathJoinSubstitution([
        FindPackageShare('state_machine_launch'), 'launch', 'system_mp.launch.py'
    ])
    so3_to_mavros_launch = PathJoinSubstitution([
        FindPackageShare('px4_interface_launch'), 'launch', 'SO3_command_to_mavros.launch.py'
    ])
    throttle_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'throttle_map_odom.launch.py'
    ])
    record_bag_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'record_bag.launch.py'
    ])
    msckf_calib = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'msckf_calib.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('onboard_sensing', default_value='true'),
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('vio_frame_id', default_value='vio/odom'),
        DeclareLaunchArgument('mass', default_value='4.2'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('poll_period', default_value='1.0'),

        # drivers - use LIDAR odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([drivers_launch]),
            launch_arguments={
                'robot': robot,
                'sensor_hostname': '192.168.100.12',
                'udp_dest': '192.168.100.1',
                'lidar_port': '7502',
                'imu_port': '7503',
                'replay': 'false',
                'lidar_mode': '1024x10',
                'metadata': 'ouster_metadata.json',
            }.items(),
        ),

        # pixhawk
        GroupAction(actions=[
            PushRosNamespace(robot),
            IncludeLaunchDescription(PythonLaunchDescriptionSource([px4_launch])),
        ]),

        # publish_tf
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([publish_tf_launch]),
            launch_arguments={
                'robot': robot,
                'robot_frame_id': [robot, '/base_link'],
                'vio_imu_frame_id': [robot, '/ovc_camera_link'],
            }.items(),
        ),

        # system: mapper/planner/state_machine/controller/state_estimation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([system_mp_launch]),
            launch_arguments={
                'onboard_sensing': onboard_sensing,
                'takeoff_height': '3.0',
                'mass': mass,
                'calibration_file': msckf_calib,
                'robot': robot,
                'output_odom': 'odom',
                'cam0': '/ovc/left',
                'cam1': '/ovc/right',
                'robot_frame_id': [robot, '/base_link'],
                'vio_imu_frame_id': [robot, '/ovc_camera_link'],
                'imu': '/ovc/vectornav/imu',
                'mag': '/ovc/vectornav/mag',
                'publish_body_camera_tf': 'false',
                'lidar_cloud_topic': 'os_cloud_node/points',
                'lidar_frame': [robot, '/lidar'],
                'publish_odom_tf': 'true',
                'real_robot': 'true',
                'mapper_config': mapper_config,
            }.items(),
        ),

        # mavros pixhawk interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([so3_to_mavros_launch]),
            launch_arguments={'robot': robot, 'odom': odom_topic}.items(),
        ),

        # throttle map and odom topics
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([throttle_launch]),
            launch_arguments={'robot': robot}.items(),
        ),

        # bag record
        GroupAction(
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource([record_bag_launch]))],
            condition=IfCondition(record_bag),
        ),
    ])
