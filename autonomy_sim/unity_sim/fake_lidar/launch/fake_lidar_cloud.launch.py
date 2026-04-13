"""Launch fake_lidar_node2 and a container of depth_image_proc composable
nodes that convert each depth camera into a point cloud, then merge them."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    camera = LaunchConfiguration("camera")
    parallel = LaunchConfiguration("par")

    fake_lidar_share = get_package_share_directory("fake_lidar")
    depth2cloud_launch = os.path.join(
        fake_lidar_share, "launch", "depth2cloud.launch.py"
    )

    def include_depth2cloud(idx):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depth2cloud_launch),
            launch_arguments={
                "camera": [robot, "/", camera, str(idx)],
            }.items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument("camera", default_value="DepthCamera"),
            DeclareLaunchArgument("par", default_value="false"),
            include_depth2cloud(1),
            include_depth2cloud(2),
            include_depth2cloud(3),
            include_depth2cloud(4),
            Node(
                package="fake_lidar",
                executable="fake_lidar_node2",
                name="fake_lidar",
                namespace=robot,
                output="screen",
                parameters=[{"parallel": parallel}],
                remappings=[
                    ("points1", [camera, "1/points"]),
                    ("points2", [camera, "2/points"]),
                    ("points3", [camera, "3/points"]),
                    ("points4", [camera, "4/points"]),
                ],
            ),
        ]
    )
