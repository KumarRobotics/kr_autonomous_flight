"""Launch rqt_robot_steering in the robot namespace for gain tuning."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="quadrotor"),
            Node(
                package="rqt_robot_steering",
                executable="rqt_robot_steering",
                name="quadrotor_vel_control",
                namespace=robot,
            ),
        ]
    )
