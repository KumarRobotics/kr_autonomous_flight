"""Record a rosbag2 containing the SLOAM-relevant topics."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


TOPICS = [
    "/clicked_point",
    "/clock",
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
    "/quadrotor/global_plan_server/path",
    "/quadrotor/heartbeat",
    "/quadrotor/imu",
    "/quadrotor/lidar",
    "/quadrotor/lidar_real_sparse",
    "/quadrotor/local_global_replan_server/local_global_server/global_path_wrt_map",
    "/quadrotor/local_global_replan_server/local_global_server/global_path_wrt_odom",
    "/quadrotor/local_plan_server/expanded_cloud",
    "/quadrotor/local_plan_server/start_goal",
    "/quadrotor/local_plan_server/traj",
    "/quadrotor/local_plan_server/trajectory",
    "/quadrotor/make_marker",
    "/quadrotor/mav_services/status",
    "/quadrotor/mavros/distance_sensor/lidarlite_pub",
    "/quadrotor/motors",
    "/quadrotor/odom",
    "/quadrotor/position_cmd",
    "/quadrotor/quad_decode_msg/output_data",
    "/quadrotor/quadrotor_manager_control/trajectory",
    "/quadrotor/reset",
    "/quadrotor/so3_cmd",
    "/quadrotor/so3_control/cmd_viz",
    "/quadrotor/so3_control/corrections",
    "/quadrotor/trackers_manager/epoch",
    "/quadrotor/trackers_manager/roi",
    "/quadrotor/trackers_manager/status",
    "/quadrotor/trpy_cmd",
    "/quadrotor/unity_rosflight/RC",
    "/quadrotor/waypoint_nav/feedback",
    "/quadrotor/waypoint_nav/update",
    "/quadrotor/waypoint_nav/update_full",
    "/quadrotor/waypoints",
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


def generate_launch_description():
    out_dir = LaunchConfiguration("dir")
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
                    *TOPICS,
                ],
                output="screen",
            ),
        ]
    )
