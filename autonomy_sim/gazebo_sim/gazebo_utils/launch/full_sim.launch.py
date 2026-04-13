"""Full simulation launch for Gazebo-based sim.

NOTE: This launch uses mrsl_quadrotor_* packages which rely on Gazebo classic.
Jazzy's default Gazebo is Gazebo Harmonic — these includes need upstream
Harmonic migration before this launch will function end-to-end.
Flagged as: needs Gazebo Harmonic follow-up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    robot_base_link = LaunchConfiguration("robot_base_link")
    lidar_frame = LaunchConfiguration("lidar_frame")
    mass = LaunchConfiguration("mass")
    lidar_cloud_topic = LaunchConfiguration("lidar_cloud_topic")
    gt_odom = LaunchConfiguration("gt_odom")
    onboard_sensing = LaunchConfiguration("onboard_sensing")
    launch_sim = LaunchConfiguration("launch_sim")
    takeoff_height = LaunchConfiguration("takeoff_height")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    gains_file = LaunchConfiguration("gains_file")
    world_model = LaunchConfiguration("world_model")

    gazebo_utils_share = get_package_share_directory("gazebo_utils")
    default_gains_file = os.path.join(
        gazebo_utils_share, "config", "falcon4_os1_so3_gains.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            DeclareLaunchArgument(
                "robot_base_link", default_value=[robot, "/base_link"]
            ),
            DeclareLaunchArgument("lidar_frame", default_value="os1_lidar"),
            DeclareLaunchArgument("mass", default_value="1.83"),
            DeclareLaunchArgument(
                "lidar_cloud_topic",
                default_value="/falcon4/os1_cloud_node/points",
            ),
            DeclareLaunchArgument("gt_odom", default_value="ground_truth/odom"),
            DeclareLaunchArgument("onboard_sensing", default_value="false"),
            DeclareLaunchArgument("launch_sim", default_value="true"),
            DeclareLaunchArgument("takeoff_height", default_value="1.0"),
            DeclareLaunchArgument("x", default_value="10.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="1.0"),
            DeclareLaunchArgument("yaw", default_value="-0.0"),
            DeclareLaunchArgument("gains_file", default_value=default_gains_file),
            DeclareLaunchArgument("world_model", default_value="forest0"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [gazebo_utils_share, "launch", "simulation.launch.py"]
                    )
                ),
                condition=IfCondition(launch_sim),
                launch_arguments={
                    "odom": gt_odom,
                    "robot": robot,
                    "world_model": world_model,
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                }.items(),
            ),
            GroupAction(
                [
                    PushRosNamespace(robot),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="map_to_world",
                        arguments=[
                            "0.0",
                            "0.0",
                            "0.0",
                            "0.0",
                            "0.0",
                            "0.0",
                            [robot, "/map"],
                            "map",
                        ],
                    ),
                    # msg_to_tf node from mrsl_quadrotor_utils:
                    # needs Gazebo Harmonic follow-up
                    Node(
                        package="mrsl_quadrotor_utils",
                        executable="msg_to_tf",
                        name="odom_to_tf",
                        output="screen",
                        remappings=[("msg", "odom")],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="base_link_tf",
                        arguments=[
                            "0.0",
                            "0.0",
                            "0.0",
                            "0.0",
                            "0.0",
                            "0.0",
                            robot_base_link,
                            robot,
                        ],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="lidar_tf",
                        arguments=[
                            "0.0",
                            "0.0",
                            "0.1",
                            "0.0",
                            "0.0",
                            "0.0",
                            robot,
                            lidar_frame,
                        ],
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
                    "lidar_frame": lidar_frame,
                    "lidar_cloud_topic": lidar_cloud_topic,
                    "mass": mass,
                    "takeoff_height": takeoff_height,
                    "gains_file": gains_file,
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
        ]
    )
