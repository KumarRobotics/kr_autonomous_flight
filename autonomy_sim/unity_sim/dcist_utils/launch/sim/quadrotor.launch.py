"""Spawn a quadrotor in the arl_unity_ros simulator.

NOTE: Relies on arl_unity_ros(_air) packages being ported to ROS 2 Jazzy.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    description = LaunchConfiguration("description")

    dcist_share = get_package_share_directory("dcist_utils")
    default_description = os.path.join(
        dcist_share, "config", "emu_stereo_lidar.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot"),
            DeclareLaunchArgument("x", default_value="50.0"),
            DeclareLaunchArgument("y", default_value="-30.0"),
            DeclareLaunchArgument("z", default_value="1.0"),
            DeclareLaunchArgument(
                "description", default_value=default_description
            ),
            Node(
                package="arl_unity_ros",
                executable="spawn_yaml_robot",
                name=["spawn_yaml_", robot],
                output="screen",
                parameters=[description],
            ),
        ]
    )
