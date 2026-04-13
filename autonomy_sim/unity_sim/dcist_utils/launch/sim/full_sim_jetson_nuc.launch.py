"""Jetson / NUC configuration variant of full_sim (onboard sensing enabled).

NOTE: Depends on state_machine_launch, client_launch, arl_unity_ros(_air)
ports.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    launch_unity = LaunchConfiguration("launch_unity")
    onboard_sensing = LaunchConfiguration("onboard_sensing")
    param_file = LaunchConfiguration("param_file")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    dcist_share = get_package_share_directory("dcist_utils")

    odom = PythonExpression(
        [
            '"ukf_odom" if "',
            onboard_sensing,
            '" == "true" else "/unity_command/ground_truth/" + "',
            robot,
            '" + "/odom"',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "takeoff_node_start_delay", default_value="15.0"
            ),
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument(
                "min_dispersion_planner", default_value="false"
            ),
            DeclareLaunchArgument("launch_unity", default_value="true"),
            DeclareLaunchArgument("onboard_sensing", default_value="true"),
            DeclareLaunchArgument(
                "param_file",
                default_value=PathJoinSubstitution(
                    [
                        get_package_share_directory("arl_unity_ros"),
                        "config",
                        "overpasscity.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("x", default_value="2.0"),
            DeclareLaunchArgument("y", default_value="2.0"),
            DeclareLaunchArgument("z", default_value="1.0"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        dcist_share, "launch", "sim", "sim_quad.launch.py"
                    )
                ),
                launch_arguments={
                    "robot": robot,
                    "odom": odom,
                    "launch_unity": launch_unity,
                    "param_file": param_file,
                    "x": x,
                    "y": y,
                    "z": z,
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("client_launch"),
                            "launch",
                            "client.launch.py",
                        ]
                    )
                ),
                launch_arguments={"robot": robot}.items(),
            ),
            Node(
                package="arl_unity_ros_air",
                executable="rosflight_offboard",
                name="robot",
                namespace=robot,
                output="screen",
            ),
        ]
    )
