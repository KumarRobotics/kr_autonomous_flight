"""Launch msckf_calib_gen for generating a msckf_vio calib.yaml from
published camera_info and tf."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    cam0 = LaunchConfiguration("cam0")
    cam1 = LaunchConfiguration("cam1")
    imu = LaunchConfiguration("imu")
    output = LaunchConfiguration("output")

    dcist_share = get_package_share_directory("dcist_utils")
    default_output = os.path.join(
        dcist_share, "config", "msckf_calib_auto_generated.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument("cam0", default_value="cam_left"),
            DeclareLaunchArgument("cam1", default_value="cam_right"),
            DeclareLaunchArgument("imu", default_value="imu"),
            DeclareLaunchArgument("output", default_value=default_output),
            Node(
                package="dcist_utils",
                executable="msckf_calib_gen",
                name="msckf_calib_gen",
                namespace=robot,
                output="screen",
                parameters=[{"output": output}],
                remappings=[
                    ("cam0/camera_info", [cam0, "/camera_info"]),
                    ("cam1/camera_info", [cam1, "/camera_info"]),
                    ("imu", imu),
                ],
            ),
        ]
    )
