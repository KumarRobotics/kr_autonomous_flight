from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fixed_frame_id = LaunchConfiguration('fixed_frame_id')
    child_frame_id = LaunchConfiguration('child_frame_id')
    imu = LaunchConfiguration('imu')
    calibration_file = LaunchConfiguration('calibration_file')

    default_calibration_file = PathJoinSubstitution([
        FindPackageShare('estimation_launch'), 'config', 'msckf_calib.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('fixed_frame_id', default_value='odom'),
        DeclareLaunchArgument('child_frame_id', default_value='base_link'),
        DeclareLaunchArgument('imu', default_value='imu'),
        DeclareLaunchArgument('calibration_file', default_value=default_calibration_file),

        Node(
            package='msckf_vio',
            executable='msckf_vio_node',
            name='vio',
            output='screen',
            parameters=[
                calibration_file,
                {
                    'publish_tf': False,
                    'frame_rate': 30,
                    'fixed_frame_id': fixed_frame_id,
                    'child_frame_id': child_frame_id,
                    'max_cam_state_size': 20,
                    'position_std_threshold': 8.0,

                    'rotation_threshold': 0.2618,
                    'translation_threshold': 0.4,
                    'tracking_rate_threshold': 0.5,

                    'feature/config/translation_threshold': -1.0,

                    'noise/gyro': 0.01,
                    'noise/acc': 0.1,
                    'noise/gyro_bias': 0.005,
                    'noise/acc_bias': 0.05,
                    'noise/feature': 0.03,

                    'initial_state/velocity/x': 0.0,
                    'initial_state/velocity/y': 0.0,
                    'initial_state/velocity/z': 0.0,

                    'initial_covariance/velocity': 0.25,
                    'initial_covariance/gyro_bias': 0.0001,
                    'initial_covariance/acc_bias': 0.01,
                    'initial_covariance/extrinsic_rotation_cov': 3.0462e-4,
                    'initial_covariance/extrinsic_translation_cov': 2.5e-5,
                },
            ],
            remappings=[
                ('~/imu', imu),
                ('~/features', 'image_processor/features'),
            ],
        ),
    ])
