from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_dir = LaunchConfiguration('dir')

    topics = [
        '/Odometry',
        '/os_node/imu_packets',
        '/ovc/vectornav/imu',
        '/os_node/lidar_packets',
        '/os_node/metadata',
        '/ovc/vectornav/mag',
        '/ovc/left/compressed',
        '/ovc/right/compressed',
        '/tf',
        '/tf_static',
        '/quadrotor/local_plan_server/traj',
        '/quadrotor/global_plan_server/path',
        '/quadrotor/local_global_replan_server/local_global_server/global_path_wrt_map',
        '/rosout',
        '/rosout_agg',
        '/quadrotor/odom',
        '/ublox/fix',
        '/ublox/fix_velocity',
        '/quadrotor/so3_control/cmd_viz',
        '/quadrotor/position_cmd',
        '/quadrotor/local_plan_server/expanded_cloud',
        '/diagnostics',
        '/graph_slam/landmarks',
        '/graph_slam/odom',
        '/graph_slam/pose',
        '/graph_slam/submap',
        '/graph_slam/trajectory',
        '/sloam/debug/obs_tree_features',
        '/sloam/map_pose',
        '/quadrotor/mapper/local_voxel_map_throttled',
        '/quadrotor/mapper/global_voxel_map_throttled',
        '/quadrotor/local_plan_server/trajectory_planner/optimal_list',
        '/quadrotor/local_plan_server/trajectory_planner/kino_astar_list',
        '/quadrotor/local_plan_server/trajectory_planner/sikangpolyhedron',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('dir', default_value='/home/dcist/bags/falcon-california-field-trip'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_dir, *topics],
            output='screen',
        ),
    ])
