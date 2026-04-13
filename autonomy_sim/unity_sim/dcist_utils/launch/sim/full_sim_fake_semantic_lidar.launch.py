"""Top-level launch for the Unity sim with fake semantic LiDAR.

NOTE: Depends on state_machine_launch, client_launch, arl_unity_ros(_air),
graphslam, and polygon_coverage_ros ports.
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
    run_graph_slam = LaunchConfiguration("run_graph_slam")
    launch_unity = LaunchConfiguration("launch_unity")
    coverage_planner = LaunchConfiguration("coverage_planner")
    save_range_images = LaunchConfiguration("save_range_images")
    onboard_sensing = LaunchConfiguration("onboard_sensing")
    sim_binary_name = LaunchConfiguration("sim_binary_name")
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
                "takeoff_node_start_delay", default_value="10.0"
            ),
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument(
                "min_dispersion_planner", default_value="false"
            ),
            DeclareLaunchArgument("run_graph_slam", default_value="false"),
            DeclareLaunchArgument("launch_unity", default_value="true"),
            DeclareLaunchArgument("coverage_planner", default_value="false"),
            DeclareLaunchArgument("save_range_images", default_value="false"),
            DeclareLaunchArgument("onboard_sensing", default_value="false"),
            DeclareLaunchArgument(
                "sim_binary_name", default_value="forest.x86_64"
            ),
            DeclareLaunchArgument(
                "param_file",
                default_value=os.path.join(
                    dcist_share, "config", "forest.yaml"
                ),
            ),
            DeclareLaunchArgument("x", default_value="170.0"),
            DeclareLaunchArgument("y", default_value="25.0"),
            DeclareLaunchArgument("z", default_value="9.0"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("fake_lidar"),
                            "launch",
                            "fake_lidar_semantic_cloud.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot": robot,
                    "save_as_images": save_range_images,
                    "save_image_interval": "5",
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        dcist_share,
                        "launch",
                        "sim",
                        "sim_quad_fake_semantic_lidar.launch.py",
                    )
                ),
                launch_arguments={
                    "sim_binary_name": sim_binary_name,
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
                    "lidar_frame": "DepthCamera1",
                    "robot": robot,
                    "takeoff_height": "4",
                    "min_dispersion_planner": min_dispersion_planner,
                    "lidar_cloud_topic": "fake_lidar/cloud",
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
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("graphslam"),
                            "launch",
                            "run_for_unity.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(run_graph_slam),
            ),
            Node(
                package="fake_sloam",
                executable="fake_sloam_node",
                name="fake_sloam",
                output="screen",
                condition=IfCondition(run_graph_slam),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("polygon_coverage_ros"),
                            "launch",
                            "coverage_planner.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(coverage_planner),
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
