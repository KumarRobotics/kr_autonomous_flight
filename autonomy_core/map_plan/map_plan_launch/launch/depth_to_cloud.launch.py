from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Arguments
    image_raw = LaunchConfiguration('image_raw', default='image_raw')
    points = LaunchConfiguration('points', default='points')

    # Create a component container
    container = ComposableNodeContainer(
        name='depth_image_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
    )

    # Metric conversion component
    metric_rect = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::ConvertMetricNode',
        name='metric_rect',
        remappings=[
            ('image_raw', image_raw),
            ('image', 'image_rect'),
        ],
    )

    # Point cloud conversion component
    cloudify = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        name='cloudify',
        remappings=[
            ('points', points),
        ],
    )

    return LaunchDescription([
        container,
        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[metric_rect, cloudify],
        )
    ])