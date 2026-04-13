"""Record a rosbag2 containing the SLOAM-relevant topics plus raw images."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    out_dir = LaunchConfiguration("dir")
    topics = [
        "/clicked_point",
        "/clock",
        "/quadrotor/cam_left/camera_info",
        "/quadrotor/cam_left/image_raw",
        "/quadrotor/cam_right/detection",
        "/quadrotor/cam_right/image_raw",
        "/all_input_points",
        "/quadrotor/coverage_path_visualization",
        "/quadrotor/state_trigger",
        "/quadrotor/waypoints",
        "/quadrotor/image_processor/debug_stereo_image",
        "/quadrotor/vio/feature_point_cloud",
        "/quadrotor/vio/odom",
        "/graph_slam/odom",
        "/initialpose",
        "/move_base_simple/goal",
        "/quadrotor/IMU",
        "/quadrotor/cmd_vel",
        "/quadrotor/diagnostics",
        "/quadrotor/estop",
        "/quadrotor/fake_lidar/cloud",
        "/quadrotor/fake_lidar/trees_cloud",
        "/quadrotor/heartbeat",
        "/quadrotor/imu",
        "/quadrotor/lidar",
        "/quadrotor/lidar_real_sparse",
        "/quadrotor/odom",
        "/quadrotor/position_cmd",
        "/quadrotor/so3_cmd",
        "/quadrotor/so3_control/cmd_viz",
        "/quadrotor/so3_control/corrections",
        "/quadrotor/unity_rosflight/RC",
        "/rosout",
        "/tf",
        "/tf_static",
        "/timing/mapper",
        "/timing/replanner/global_replan",
        "/timing/replanner/local_replan",
        "/unity_command/command_topic",
        "/unity_command/ground_truth/quadrotor/odom",
        "/unity_command/ground_truth/quadrotor/pose",
        "/unity_command/ground_truth/quadrotor/twist",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("dir", default_value="/tmp/bags/unity_sloam"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "--compression-mode",
                    "file",
                    "--compression-format",
                    "zstd",
                    "-o",
                    out_dir,
                    *topics,
                ],
                output="screen",
            ),
        ]
    )
