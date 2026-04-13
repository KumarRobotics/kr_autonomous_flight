from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu = LaunchConfiguration('imu')
    cam0 = LaunchConfiguration('cam0')
    cam1 = LaunchConfiguration('cam1')
    image = LaunchConfiguration('image')
    calibration_file = LaunchConfiguration('calibration_file')

    return LaunchDescription([
        DeclareLaunchArgument('cam0', default_value='cam_left'),
        DeclareLaunchArgument('cam1', default_value='cam_right'),
        DeclareLaunchArgument('image', default_value='image_raw'),
        DeclareLaunchArgument('imu', default_value='imu'),
        # don't use a default, user must provide one
        DeclareLaunchArgument('calibration_file'),

        Node(
            package='msckf_vio',
            executable='image_processor_node',
            name='image_processor',
            output='screen',
            parameters=[
                calibration_file,
                {
                    'grid_row': 5,
                    'grid_col': 6,
                    'grid_min_feature_num': 1,
                    'grid_max_feature_num': 3,
                    'pyramid_levels': 3,
                    'patch_size': 15,
                    'fast_threshold': 10,
                    'max_iteration': 30,
                    'track_precision': 0.01,
                    'ransac_threshold': 3,
                    'stereo_threshold': 5,
                },
            ],
            remappings=[
                ('~/imu', imu),
                ('~/cam0_image', [cam0, '/', image]),
                ('~/cam1_image', [cam1, '/', image]),
            ],
        ),
    ])
