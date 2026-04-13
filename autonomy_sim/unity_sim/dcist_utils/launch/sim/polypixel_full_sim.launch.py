"""Top-level polypixel (large world) sim.

NOTE: Depends on state_machine_launch, client_launch, arl_unity_ros(_air),
map_plan_launch, control_launch ports.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    use_3d = LaunchConfiguration("use_3d")
    odom = LaunchConfiguration("odom")
    min_dispersion_planner = LaunchConfiguration("min_dispersion_planner")
    launch_unity = LaunchConfiguration("launch_unity")

    dcist_share = get_package_share_directory("dcist_utils")
    control_launch_share = get_package_share_directory("control_launch")
    map_plan_launch_share = get_package_share_directory("map_plan_launch")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "takeoff_node_start_delay", default_value="10.0"
            ),
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument("use_3d", default_value="false"),
            DeclareLaunchArgument(
                "odom",
                default_value=[
                    "/unity_command/ground_truth/",
                    robot,
                    "/odom",
                ],
            ),
            DeclareLaunchArgument(
                "min_dispersion_planner", default_value="false"
            ),
            DeclareLaunchArgument("launch_unity", default_value="true"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        dcist_share,
                        "launch",
                        "sim",
                        "sim_quad_polypixel.launch.py",
                    )
                ),
                launch_arguments={
                    "robot": robot,
                    "odom": odom,
                    "launch_unity": launch_unity,
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
                            os.path.join(
                                control_launch_share,
                                "config",
                                "tracker_params_mp.yaml",
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
                    "robot": robot,
                    "takeoff_height": "8",
                    "min_dispersion_planner": min_dispersion_planner,
                    "lidar_cloud_topic": "fake_lidar/points",
                    "mapper_config": os.path.join(
                        map_plan_launch_share, "config", "mapper_3d.yaml"
                    ),
                    "planner_config": os.path.join(
                        control_launch_share,
                        "config",
                        "tracker_params_mp_3d.yaml",
                    ),
                }.items(),
                condition=IfCondition(use_3d),
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
                    "robot": robot,
                    "takeoff_height": "8",
                    "min_dispersion_planner": min_dispersion_planner,
                    "lidar_cloud_topic": "fake_lidar/points",
                    "mapper_config": os.path.join(
                        map_plan_launch_share, "config", "mapper.yaml"
                    ),
                    "planner_config": os.path.join(
                        control_launch_share,
                        "config",
                        "tracker_params_mp.yaml",
                    ),
                }.items(),
                condition=UnlessCondition(use_3d),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("fake_lidar"),
                            "launch",
                            "fake_lidar_cloud.launch.py",
                        ]
                    )
                ),
                launch_arguments={"robot": robot}.items(),
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
