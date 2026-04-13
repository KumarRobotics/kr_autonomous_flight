"""Launch odom2tf node which relays odometry and publishes a world->map tf."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom = LaunchConfiguration("odom")
    robot = LaunchConfiguration("robot")
    world_frame = LaunchConfiguration("world_frame")
    ground_truth_robot_frame = LaunchConfiguration("ground_truth_robot_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom"),
            DeclareLaunchArgument("robot"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument(
                "ground_truth_robot_frame",
                default_value=["ground_truth/", robot, "/", robot],
            ),
            Node(
                package="dcist_utils",
                executable="odom2tf",
                name="odom2tf",
                output="screen",
                parameters=[
                    {
                        "world_frame": world_frame,
                        "robot": robot,
                        "ground_truth_robot_frame": ground_truth_robot_frame,
                    }
                ],
                remappings=[
                    ("odom_in", odom),
                    ("odom_out", "odom"),
                ],
            ),
        ]
    )
