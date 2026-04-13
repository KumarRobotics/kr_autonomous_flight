"""Launch the full sim_quad pipeline: unity simulator, quadrotor spawn,
rosflight bridge, odom relay.

NOTE: Depends on arl_unity_ros, arl_unity_ros_air, rosflight, and
kr_rosflight_interface — these still need ROS 2 Jazzy ports.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    launch_unity = LaunchConfiguration("launch_unity")
    odom = LaunchConfiguration("odom")
    param_file = LaunchConfiguration("param_file")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    dcist_share = get_package_share_directory("dcist_utils")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument("launch_unity", default_value="true"),
            DeclareLaunchArgument(
                "odom",
                default_value=["/unity_command/ground_truth/", robot, "/odom"],
            ),
            DeclareLaunchArgument("param_file", default_value=""),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
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
                    "param_file": param_file,
                    "launch_unity": launch_unity,
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
                        dcist_share, "config", "emu_stereo_lidar.yaml"
                    ),
                    "x": x,
                    "y": y,
                    "z": z,
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
