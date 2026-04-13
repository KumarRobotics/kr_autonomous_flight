"""Launch the SO3CmdToRosflight component from kr_rosflight_interface.

NOTE: In ROS 1 this was a nodelet. In ROS 2 we load it as a regular node;
if the upstream port exposes it as a composable component, this can be
swapped to a ComposableNodeContainer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_prop_force = LaunchConfiguration("max_prop_force")

    return LaunchDescription(
        [
            DeclareLaunchArgument("max_prop_force", default_value="7"),
            Node(
                package="kr_rosflight_interface",
                executable="so3cmd_to_rosflight",
                name="so3cmd_to_rosflight",
                output="screen",
                parameters=[
                    {
                        "num_props": 4,
                        "max_prop_force": max_prop_force,
                    }
                ],
                remappings=[
                    ("odom", "odom"),
                    ("so3_cmd", "so3_cmd"),
                    ("imu", "rosflight/imu/data"),
                    ("command", "rosflight/command"),
                ],
            ),
        ]
    )
