"""Top-level launch for the full Unity simulation experiment.

NOTE: Depends on state_machine_launch, client_launch, and
arl_unity_ros(_air) packages that still need ROS 2 Jazzy ports.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    takeoff_node_start_delay = LaunchConfiguration("takeoff_node_start_delay")
    robot = LaunchConfiguration("robot")
    min_dispersion_planner = LaunchConfiguration("min_dispersion_planner")
    launch_unity = LaunchConfiguration("launch_unity")
    onboard_sensing = LaunchConfiguration("onboard_sensing")
    param_file = LaunchConfiguration("param_file")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    dcist_share = get_package_share_directory("dcist_utils")

    # odom default depends on onboard_sensing
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
            DeclareLaunchArgument("onboard_sensing", default_value="false"),
            DeclareLaunchArgument(
                "param_file",
                default_value=PathJoinSubstitution(
                    [
                        get_package_share_directory("arl_unity_ros"),
                        "config",
                        "basic_environment.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
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
            GroupAction(
                [
                    PushRosNamespace(robot),
                    Node(
                        package="motion_primitives",
                        executable="planning_server",
                        name="local_plan_server",
                        output="screen",
                        parameters=[
                            PathJoinSubstitution(
                                [
                                    get_package_share_directory(
                                        "control_launch"
                                    ),
                                    "config",
                                    "tracker_params_mp.yaml",
                                ]
                            )
                        ],
                        remappings=[("voxel_map", "mapper/local_voxel_map")],
                        condition=IfCondition(min_dispersion_planner),
                    ),
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("state_machine_launch"),
                            "launch",
                            "system_mp.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "onboard_sensing": onboard_sensing,
                    "robot": robot,
                    "lidar_frame": "lidar",
                    "takeoff_height": "10",
                    "min_dispersion_planner": min_dispersion_planner,
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
