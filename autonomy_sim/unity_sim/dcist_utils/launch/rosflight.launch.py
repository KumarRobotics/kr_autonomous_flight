"""Launch rosflight_io, unity_rosflight, and rc_sim + calibrate_imu helpers.

NOTE: Depends on external packages (rosflight, arl_unity_ros_air) that still
need to be ported to ROS 2 Jazzy before this launch file will run.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavlink_port = LaunchConfiguration("mavlink_port")
    firmware_bind_port = LaunchConfiguration("firmware_bind_port")
    firmware_remote_port = LaunchConfiguration("firmware_remote_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mavlink_port", default_value="14560"),
            DeclareLaunchArgument("firmware_bind_port", default_value="14525"),
            DeclareLaunchArgument(
                "firmware_remote_port", default_value="14520"
            ),
            Node(
                package="rosflight",
                executable="rosflight_io",
                name="rosflight_io",
                namespace="rosflight",
                output="screen",
                parameters=[
                    {
                        "udp": True,
                        "bind_port": firmware_remote_port,
                        "remote_port": firmware_bind_port,
                    }
                ],
            ),
            Node(
                package="arl_unity_ros_air",
                executable="unity_rosflight",
                name="unity_rosflight",
                output="screen",
                parameters=[
                    {
                        "mavlink_port": mavlink_port,
                        "firmware_bind_port": firmware_bind_port,
                        "firmware_remote_port": firmware_remote_port,
                    }
                ],
            ),
            Node(
                package="arl_unity_ros_air",
                executable="rc_sim",
                name="rc_sim",
                output="screen",
                remappings=[("RC", "unity_rosflight/RC")],
            ),
            Node(
                package="arl_unity_ros_air",
                executable="calibrate_imu",
                name="calibrate_imu",
            ),
        ]
    )
