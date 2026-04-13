from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    robot_frame_id = LaunchConfiguration('robot_frame_id')
    vio_imu_frame_id = LaunchConfiguration('vio_imu_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='quadrotor'),
        DeclareLaunchArgument('robot_frame_id', default_value=[robot, '/base_link']),
        DeclareLaunchArgument('vio_imu_frame_id', default_value=[robot, '/ovc_camera_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_odom_map_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', [robot, '/map'], [robot, '/odom']],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_map_world_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_map_map_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', [robot, '/map']],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_odom_world_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', [robot, '/map']],
        ),
        # ovc-imu to body transform (case: 27 deg pitch)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[robot, 'body_ovc_tf'],
            arguments=['0.1', '-0.05', '0', '1.5707963', '0.0', '-1.27', robot_frame_id, vio_imu_frame_id],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[robot, 'ovc_to_imu_tf'],
            arguments=['0', '0', '0', '0', '0', '0', vio_imu_frame_id, 'ovc_vnav_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[robot, 'body_lidar_tf'],
            arguments=['0', '0', '0.1', '0', '0', '0', robot_frame_id, [robot, '/lidar']],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_lidar_tf',
            arguments=['0', '0', '0.0', '0', '0', '0', [robot, '/lidar'], 'os_lidar'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_odom_lidarodom_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_init', [robot, '/map']],
        ),
    ])
