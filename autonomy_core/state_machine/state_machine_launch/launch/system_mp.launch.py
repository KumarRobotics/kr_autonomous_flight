from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapper_config = LaunchConfiguration('mapper_config')
    planner_config = LaunchConfiguration('planner_config')
    onboard_sensing = LaunchConfiguration('onboard_sensing')
    use_rgbd = LaunchConfiguration('use_rgbd')
    robot = LaunchConfiguration('robot')
    lidar_frame = LaunchConfiguration('lidar_frame')
    lidar_cloud_topic = LaunchConfiguration('lidar_cloud_topic')
    depth_cam = LaunchConfiguration('depth_cam')
    takeoff_height = LaunchConfiguration('takeoff_height')
    mass = LaunchConfiguration('mass')
    gains_file = LaunchConfiguration('gains_file')
    real_robot = LaunchConfiguration('real_robot')
    cam0 = LaunchConfiguration('cam0')
    cam1 = LaunchConfiguration('cam1')
    imu = LaunchConfiguration('imu')
    mag = LaunchConfiguration('mag')
    output_odom = LaunchConfiguration('output_odom')
    robot_frame_id = LaunchConfiguration('robot_frame_id')
    vio_imu_frame_id = LaunchConfiguration('vio_imu_frame_id')
    calibration_file = LaunchConfiguration('calibration_file')
    publish_body_camera_tf = LaunchConfiguration('publish_body_camera_tf')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    default_planner_config = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])
    default_gains_file = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'so3_control_gains.yaml'
    ])

    estimation_launch = PathJoinSubstitution([
        FindPackageShare('estimation_launch'), 'launch', 'estimation.launch.py'
    ])
    control_mp_launch = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'launch', 'control_mp.launch.py'
    ])
    depth_to_cloud_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'depth_to_cloud.launch.py'
    ])
    map_plan_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'map_plan.launch.py'
    ])
    state_machine_mp_launch = PathJoinSubstitution([
        FindPackageShare('state_machine_launch'), 'launch', 'state_machine_mp.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('planner_config', default_value=default_planner_config),
        DeclareLaunchArgument('onboard_sensing', default_value='false'),
        DeclareLaunchArgument('use_rgbd', default_value='false'),
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('lidar_frame', default_value=''),
        DeclareLaunchArgument('lidar_cloud_topic', default_value='lidar'),
        DeclareLaunchArgument('mag', default_value='mag'),
        DeclareLaunchArgument('depth_cam', default_value='DepthCamera'),
        DeclareLaunchArgument('takeoff_height', default_value='5'),
        DeclareLaunchArgument('mass', default_value='1.74'),
        DeclareLaunchArgument('gains_file', default_value=default_gains_file),
        DeclareLaunchArgument('real_robot', default_value='false'),
        DeclareLaunchArgument('cam0', default_value='cam_left'),
        DeclareLaunchArgument('cam1', default_value='cam_right'),
        DeclareLaunchArgument('imu', default_value='imu'),
        DeclareLaunchArgument('output_odom', default_value='ukf_odom'),
        DeclareLaunchArgument('robot_frame_id', default_value=robot),
        DeclareLaunchArgument('vio_imu_frame_id', default_value=[robot, '/stereo_rig_imu']),
        DeclareLaunchArgument('calibration_file', default_value='msckf_calib.yaml'),
        DeclareLaunchArgument('publish_body_camera_tf', default_value='true'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),

        # S-MSCKF VIO
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([estimation_launch]),
                    launch_arguments={
                        'publish_body_camera_tf': publish_body_camera_tf,
                        'vio_imu_frame_id': vio_imu_frame_id,
                        'lidar_frame_id': lidar_frame,
                        'publish_odom_tf': publish_odom_tf,
                        'cam0': cam0,
                        'cam1': cam1,
                        'imu': imu,
                        'mag': mag,
                        'enable_vio_odom': 'true',
                        'enable_lidar': 'false',
                        'output_odom': output_odom,
                        'robot_frame_id': robot_frame_id,
                        'calibration_file': calibration_file,
                    }.items(),
                ),
            ],
            condition=IfCondition(onboard_sensing),
        ),

        # Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([control_mp_launch]),
            launch_arguments={
                'mass': mass,
                'gains_file': gains_file,
            }.items(),
        ),

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
                        'planner_config': planner_config,
                        'robot': robot,
                        'lidar_frame': lidar_frame,
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
                        'real_robot': real_robot,
                        'cloud': lidar_cloud_topic,
                        'mapper_config': mapper_config,
                        'planner_config': planner_config,
                        'robot': robot,
                        'lidar_frame': lidar_frame,
                    }.items(),
                ),
            ],
            condition=UnlessCondition(use_rgbd),
        ),

        # State machine
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_machine_mp_launch]),
            launch_arguments={'takeoff_height': takeoff_height}.items(),
        ),
    ])
