"""Spawn a quadrotor in the overpasscity Unity environment with custom
TrueState propagation.

NOTE: Depends on arl_unity_ros, rosflight, kr_rosflight_interface ports.
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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    odom = LaunchConfiguration("odom")

    dcist_share = get_package_share_directory("dcist_utils")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value=EnvironmentVariable(
                    "ROBOT_NAME", default_value="quadrotor"
                ),
            ),
            DeclareLaunchArgument(
                "odom",
                default_value="/unity_command/ground_truth/quadrotor/odom",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("arl_unity_ros"),
                            "launch",
                            "simulator.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "param_file": os.path.join(
                        dcist_share, "config", "overpasscity_demo.yaml"
                    ),
                    "launch_unity": "true",
                }.items(),
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
                        dcist_share, "config", "emu_stereo_rgbd.yaml"
                    ),
                    "x": "-40.0",
                    "y": "5.0",
                    "z": "3.0",
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_TrueState_propagate",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    ["ground_truth/", robot, "/base_link"],
                    [robot, "/TrueState"],
                ],
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
