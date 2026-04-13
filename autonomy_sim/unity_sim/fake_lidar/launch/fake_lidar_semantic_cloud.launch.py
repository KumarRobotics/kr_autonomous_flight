"""Launch fake_lidar_node (semantic variant) which consumes depth + semantic
images from four DepthCameras and publishes a merged pseudo-LiDAR cloud."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    camera = LaunchConfiguration("camera")
    save_as_images = LaunchConfiguration("save_as_images")
    save_path_prefix = LaunchConfiguration("save_path_prefix")
    save_image_interval = LaunchConfiguration("save_image_interval")

    fake_lidar_share = get_package_share_directory("fake_lidar")
    os1_64_yaml = os.path.join(fake_lidar_share, "launch", "os1-64.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="/quadrotor"),
            DeclareLaunchArgument("camera", default_value="DepthCamera"),
            DeclareLaunchArgument("save_as_images", default_value="false"),
            DeclareLaunchArgument(
                "save_path_prefix", default_value="/tmp/range_images"
            ),
            DeclareLaunchArgument("save_image_interval", default_value="2"),
            Node(
                package="fake_lidar",
                executable="fake_lidar_node",
                name="fake_lidar",
                namespace=robot,
                output="screen",
                parameters=[
                    os1_64_yaml,
                    {
                        "save_as_images": save_as_images,
                        "save_path_prefix": save_path_prefix,
                        "save_image_interval": save_image_interval,
                    },
                ],
                remappings=[
                    ("image1", [camera, "1/image_raw"]),
                    ("cinfo1", [camera, "1/camera_info"]),
                    ("semantic1", [camera, "1/semantic_image"]),
                    ("image2", [camera, "2/image_raw"]),
                    ("cinfo2", [camera, "2/camera_info"]),
                    ("semantic2", [camera, "2/semantic_image"]),
                    ("image3", [camera, "3/image_raw"]),
                    ("cinfo3", [camera, "3/camera_info"]),
                    ("semantic3", [camera, "3/semantic_image"]),
                    ("image4", [camera, "4/image_raw"]),
                    ("cinfo4", [camera, "4/camera_info"]),
                    ("semantic4", [camera, "4/semantic_image"]),
                ],
            ),
        ]
    )
