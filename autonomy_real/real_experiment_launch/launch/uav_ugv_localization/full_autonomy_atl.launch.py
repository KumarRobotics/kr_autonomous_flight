from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    record_bag = LaunchConfiguration('record_bag')

    # NOTE: throttle_imu.launch.py was referenced in the upstream ROS1 source
    # but the file never existed in the real_experiment_launch package on
    # master or on feature/integrate_lidar_3d_planner_default. The ROS2 port
    # faithfully preserves this gap; the include is commented out below so
    # that `ros2 launch` can at least resolve this file. If someone needs
    # IMU throttling, port the original ROS1 logic (topic_tools throttle
    # node on the IMU topic) into a new throttle_imu.launch.py and re-add
    # the include.
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

        # IncludeLaunchDescription(PythonLaunchDescriptionSource([throttle_imu_launch])),
        # ^ disabled: throttle_imu.launch does not exist in the package; see
        # the note where throttle_imu_launch was defined (now removed).

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
