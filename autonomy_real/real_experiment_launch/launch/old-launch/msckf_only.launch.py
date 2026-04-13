"""Legacy MSCKF-only launch (kept for reference)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    estimation_launch = PathJoinSubstitution([
        FindPackageShare('estimation_launch'), 'launch', 'estimation.launch.py'
    ])
    calibration_file = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'msckf_calib.yaml'
    ])

    robot = LaunchConfiguration('robot')
    publish_body_camera_tf = LaunchConfiguration('publish_body_camera_tf')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('takeoff_height', default_value='3'),
        DeclareLaunchArgument('publish_body_camera_tf', default_value='false'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([estimation_launch]),
            launch_arguments={
                'publish_body_camera_tf': publish_body_camera_tf,
                'vio_imu_frame_id': [robot, '/ovc_camera_link'],
                'publish_odom_tf': publish_odom_tf,
                'cam0': '/ovc/left',
                'cam1': '/ovc/right',
                'imu': '/ovc/vectornav',
                'output_odom': 'odom',
                'robot_frame_id': [robot, '/base_link'],
                'calibration_file': calibration_file,
            }.items(),
        ),
    ])
