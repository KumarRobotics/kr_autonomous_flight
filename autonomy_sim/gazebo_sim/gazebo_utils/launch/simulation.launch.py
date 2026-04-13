"""Gazebo simulation launch.

NOTE: Uses mrsl_quadrotor_launch which depends on Gazebo classic.
Jazzy's default Gazebo is Gazebo Harmonic (ros_gz_sim) — flagged as:
needs Gazebo Harmonic follow-up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    robot_frame = LaunchConfiguration("robot_frame")
    world_model = LaunchConfiguration("world_model")
    world_frame = LaunchConfiguration("world_frame")
    odom = LaunchConfiguration("odom")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument(
                "robot_frame", default_value=[robot, "/base_link"]
            ),
            DeclareLaunchArgument(
                "world_model",
                default_value=EnvironmentVariable("FLA_ENV", default_value="D60"),
            ),
            DeclareLaunchArgument("world_frame", default_value="map"),
            DeclareLaunchArgument("odom", default_value="ground_truth/odom"),
            DeclareLaunchArgument("x", default_value="0.5"),
            DeclareLaunchArgument("y", default_value="0.5"),
            DeclareLaunchArgument("z", default_value="0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            # Gazebo classic launch from mrsl_quadrotor_launch
            # needs Gazebo Harmonic follow-up
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("mrsl_quadrotor_launch"),
                            "launch",
                            "gazebo.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "world": world_model,
                    "gui": gazebo_gui,
                }.items(),
            ),
            GroupAction(
                [
                    PushRosNamespace(robot),
                    Node(
                        package="mrsl_quadrotor_utils",
                        executable="change_header",
                        name="change_header",
                        output="screen",
                        parameters=[
                            {
                                "robot_frame": robot_frame,
                                "world_frame": world_frame,
                            }
                        ],
                        remappings=[
                            ("odom_in", odom),
                            ("odom_out", "raw_odom"),
                        ],
                    ),
                    Node(
                        package="mrsl_quadrotor_utils",
                        executable="change_odom",
                        name="change_odom",
                        parameters=[{"dt_thr": 0.5}],
                        remappings=[
                            ("odom_in", "raw_odom"),
                            ("odom_out", "odom"),
                        ],
                    ),
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("mrsl_quadrotor_launch"),
                            "launch",
                            "spawn.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "mav_name": robot,
                    "mav_type": "falcon4_os1_16_256",
                    "x": x,
                    "y": y,
                    "z": z,
                    "Y": yaw,
                }.items(),
            ),
        ]
    )
