from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    imu = LaunchConfiguration('imu')
    mag = LaunchConfiguration('mag')
    output_odom = LaunchConfiguration('output_odom')
    robot_frame_id = LaunchConfiguration('robot_frame_id')
    vio_imu_frame_id = LaunchConfiguration('vio_imu_frame_id')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    vio_ref_frame_id = LaunchConfiguration('vio_ref_frame_id')
    ukf_ref_frame_id = LaunchConfiguration('ukf_ref_frame_id')
    publish_body_camera_tf = LaunchConfiguration('publish_body_camera_tf')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    enable_vio_odom = LaunchConfiguration('enable_vio_odom')
    enable_lidar = LaunchConfiguration('enable_lidar')

    default_calibration_file = PathJoinSubstitution([
        FindPackageShare('estimation_launch'), 'config', 'msckf_calib.yaml'
    ])
    ukf_params = PathJoinSubstitution([
        FindPackageShare('estimation_launch'), 'config', 'ukf_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value='quadrotor'),
        ),
        DeclareLaunchArgument('cam0', default_value='cam_left'),
        DeclareLaunchArgument('cam1', default_value='cam_right'),
        DeclareLaunchArgument('image', default_value='image_raw'),
        DeclareLaunchArgument('imu', default_value='imu'),
        DeclareLaunchArgument('mag', default_value='mag'),
        DeclareLaunchArgument('output_odom', default_value='ukf_odom'),
        DeclareLaunchArgument('robot_frame_id', default_value=robot),
        # This frame_id should be the frame_id of the imu used for S-MSCKF
        DeclareLaunchArgument('vio_imu_frame_id', default_value=[robot, '/stereo_rig_imu']),
        DeclareLaunchArgument('lidar_frame_id', default_value=[robot, '/lidar']),
        DeclareLaunchArgument('vio_ref_frame_id', default_value=[robot, '/odom']),
        DeclareLaunchArgument('ukf_ref_frame_id', default_value=[robot, '/odom']),
        # This should be set true for simulation
        DeclareLaunchArgument('publish_body_camera_tf', default_value='false'),
        # This should be set false for simulation
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('enable_vio_odom', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='false'),
        DeclareLaunchArgument('calibration_file', default_value=default_calibration_file),

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='fla_ukf',
                executable='fla_ukf_node',
                name='fla_ukf',
                output='screen',
                parameters=[
                    ukf_params,
                    {
                        'world_frame_id': ukf_ref_frame_id,
                        'robot_frame_id': robot_frame_id,
                        'cam_frame_id': vio_imu_frame_id,
                        'lidar_frame_id': lidar_frame_id,
                        'enable_vio_odom': enable_vio_odom,
                        'enable_lidar': enable_lidar,
                        'vision_frame_id': vio_ref_frame_id,
                        'enable_laser': False,
                        'enable_gps': False,
                        'enable_height': False,
                        'enable_mag': False,
                        'enable_yaw': False,
                        'publish_tf': publish_odom_tf,
                    },
                ],
                remappings=[
                    ('~/imu', imu),
                    ('~/mag', mag),
                    ('~/height', 'mavros/distance_sensor/lidarlite_pub'),
                    ('~/vio_odom', '/Odometry'),
                    ('~/pose_lidar', '/quadrotor/llol_odom/pose_cov'),
                    ('~/odom_out', output_odom),
                ],
            ),
        ]),

        # stereo-inertial system imu frame to body frame transform (simulation only)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[robot, '_imu_body_tf'],
            arguments=['0', '0', '0', '0', '0', '0', robot_frame_id, vio_imu_frame_id],
            condition=IfCondition(publish_body_camera_tf),
        ),
    ])
