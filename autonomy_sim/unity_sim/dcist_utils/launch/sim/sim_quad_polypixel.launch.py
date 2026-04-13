"""Spawn a quadrotor in the polypixel (large-world) Unity environment.

NOTE: Depends on arl_unity_ros(_air), rosflight, kr_rosflight_interface ports.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    odom = LaunchConfiguration("odom")

    dcist_share = get_package_share_directory("dcist_utils")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument("launch_unity", default_value="true"),
            DeclareLaunchArgument(
                "odom",
                default_value=["/unity_command/ground_truth/", robot, "/odom"],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        dcist_share, "launch", "sim", "quadrotor.launch.py"
                    )
                ),
                launch_arguments={
                    "robot": robot,
                    "description": os.path.join(
                        dcist_share, "config", "emu_stereo_rgbd_360_FOV.yaml"
                    ),
                    "x": "1000.0",
                    "y": "950.0",
                    "z": "50.0",
                }.items(),
            ),
            GroupAction(
                [
                    PushRosNamespace(robot),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            os.path.join(
                                dcist_share, "launch", "rosflight.launch.py"
                            )
                        )
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            os.path.join(
                                dcist_share,
                                "launch",
                                "rosflight_interface.launch.py",
                            )
                        )
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="tf_map_odom",
                        arguments=[
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            [robot, "/map"],
                            [robot, "/odom"],
                        ],
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            os.path.join(
                                dcist_share, "launch", "odom2tf.launch.py"
                            )
                        ),
                        launch_arguments={
                            "odom": odom,
                            "robot": robot,
                        }.items(),
                    ),
                ]
            ),
        ]
    )
