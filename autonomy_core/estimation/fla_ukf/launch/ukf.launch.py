"""ROS 2 launch file for the FLA UKF composable node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot', default_value=os.environ.get('EMU_ROBOT_NAME', ''))
    world_frame_id_arg = DeclareLaunchArgument(
        'world_frame_id', default_value='world')
    robot_frame_id_arg = DeclareLaunchArgument(
        'robot_frame_id', default_value='base_link')
    vio_imu_frame_id_arg = DeclareLaunchArgument(
        'vio_imu_frame_id', default_value='imu')
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id', default_value='lidar')
    odom_out_arg = DeclareLaunchArgument('odom_out', default_value='odom')
    publish_tf_arg = DeclareLaunchArgument('publish_tf', default_value='true')

    params_file = os.path.join(
        get_package_share_directory('fla_ukf'), 'config', 'params.yaml')

    container = ComposableNodeContainer(
        name='fla_ukf_container',
        namespace=LaunchConfiguration('robot'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='fla_ukf',
                plugin='fla_ukf::FLAUKFNodelet',
                name='fla_ukf',
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    params_file,
                    {
                        'world_frame_id': LaunchConfiguration('world_frame_id'),
                        'robot_frame_id': LaunchConfiguration('robot_frame_id'),
                        'cam_frame_id': LaunchConfiguration('vio_imu_frame_id'),
                        'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
                        'enable_vio_odom': True,
                        'enable_lidar': True,
                        'enable_laser': False,
                        'enable_gps': False,
                        'enable_height': False,
                        'enable_mag': False,
                        'enable_yaw': False,
                        'publish_tf': LaunchConfiguration('publish_tf'),
                    },
                ],
                remappings=[
                    ('imu', 'sync/imu/imu'),
                    ('height', 'mavros/distance_sensor/lidarlite_pub'),
                    ('vio_odom', 'vio/odom'),
                    ('odom_out', LaunchConfiguration('odom_out')),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        world_frame_id_arg,
        robot_frame_id_arg,
        vio_imu_frame_id_arg,
        lidar_frame_id_arg,
        odom_out_arg,
        publish_tf_arg,
        container,
    ])
