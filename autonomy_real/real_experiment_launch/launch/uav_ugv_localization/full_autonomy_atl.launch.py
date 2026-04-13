from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    record_bag = LaunchConfiguration('record_bag')

    throttle_imu_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'throttle_imu.launch.py'
    ])
    drivers_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'drivers_for_faster_lio.launch.py'
    ])
    publish_tf_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'publish_tf.launch.py'
    ])
    faster_lio_launch = PathJoinSubstitution([
        FindPackageShare('faster_lio'), 'launch', 'mapping_ouster64.launch.py'
    ])
    record_bag_atl_launch = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'launch', 'uav_ugv_localization', 'record_bag_atl.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mapper_config', default_value=PathJoinSubstitution([
            FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
        ])),
        DeclareLaunchArgument('onboard_sensing', default_value='true'),
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('vio_frame_id', default_value='vio/odom'),
        DeclareLaunchArgument('mass', default_value='4.2'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('poll_period', default_value='1.0'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource([throttle_imu_launch])),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([publish_tf_launch]),
            launch_arguments={
                'robot': robot,
                'robot_frame_id': [robot, '/base_link'],
                'vio_imu_frame_id': [robot, '/ovc_camera_link'],
            }.items(),
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource([faster_lio_launch])),

        GroupAction(
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource([record_bag_atl_launch]))],
            condition=IfCondition(record_bag),
        ),
    ])
