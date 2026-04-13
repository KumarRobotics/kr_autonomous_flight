from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    image_raw = LaunchConfiguration('image_raw')
    points = LaunchConfiguration('points')

    # Nodelet manager -> ComposableNodeContainer hosting depth_image_proc components
    container = ComposableNodeContainer(
        name='nodelet_manager',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::ConvertMetricNode',
                name='metric_rect',
                remappings=[
                    ('image_raw', image_raw),
                    ('image', 'image_rect'),
                ],
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='cloudify',
                remappings=[
                    ('points', points),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_raw', default_value='image_raw'),
        DeclareLaunchArgument('points', default_value='points'),
        container,
    ])
