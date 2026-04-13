"""Launch depth_image_proc composable nodes to convert a depth image into a
point cloud for a single camera namespace."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction


def generate_launch_description():
    camera = LaunchConfiguration("camera")
    camera_info = LaunchConfiguration("camera_info")
    image_raw_input = LaunchConfiguration("image_raw_input")
    image_rect_input = LaunchConfiguration("image_rect_input")

    container = ComposableNodeContainer(
        name="depth_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::ConvertMetricNode",
                name="convert_metric",
                remappings=[
                    ("image_raw", image_raw_input),
                    ("image", image_rect_input),
                ],
            ),
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzNode",
                name="point_cloud_xyz",
                remappings=[
                    ("camera_info", camera_info),
                    ("image_rect", image_rect_input),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera"),
            DeclareLaunchArgument("camera_info", default_value="camera_info"),
            DeclareLaunchArgument("image_raw_input", default_value="image_raw"),
            DeclareLaunchArgument("image_rect_input", default_value="image_rect"),
            GroupAction([PushRosNamespace(camera), container]),
        ]
    )
