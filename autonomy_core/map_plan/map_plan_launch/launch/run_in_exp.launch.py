"""Best-effort conversion of run_in_exp.launch.

Notes: Original ROS1 launch used many param_env map modes; this skeleton
preserves only the single local_plan_server pipeline that was actually
active in the original. Commented-out blocks from the ROS1 launch were
dropped; adjust as needed for ROS2.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mav_name = LaunchConfiguration('mav_name')
    planner_config = LaunchConfiguration('planner_config')
    mapper_config = LaunchConfiguration('mapper_config')
    point_cloud = LaunchConfiguration('point_cloud')
    world_frame_id = LaunchConfiguration('world_frame_id')
    random_map = LaunchConfiguration('random_map')
    vicon_map = LaunchConfiguration('vicon_map')
    file_map = LaunchConfiguration('file_map')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    map_x_origin = LaunchConfiguration('map_x_origin')
    map_y_origin = LaunchConfiguration('map_y_origin')
    map_z_origin = LaunchConfiguration('map_z_origin')

    default_planner_config = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])
    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    vicon_launch = PathJoinSubstitution([
        FindPackageShare('vicon_env'), 'launch', 'vicon_map.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mav_name', default_value='f250_ariel'),
        DeclareLaunchArgument('mav_id', default_value='1'),
        DeclareLaunchArgument('mav_type', default_value='hummingbird'),
        DeclareLaunchArgument('world_frame_id', default_value='dragonfly67/odom'),
        DeclareLaunchArgument('sim', default_value='1'),
        DeclareLaunchArgument('vicon', default_value='1'),
        DeclareLaunchArgument('vicon_fps', default_value='100'),
        DeclareLaunchArgument('slow_baud_rate', default_value='true'),
        DeclareLaunchArgument('mass', default_value='1.45'),
        DeclareLaunchArgument('nodelet_manager_name', default_value='nodelet_manager_control'),
        DeclareLaunchArgument('odom_topic', default_value='/dragonfly67/quadrotor_ukf/control_odom'),
        DeclareLaunchArgument('random_map', default_value='false'),
        DeclareLaunchArgument('vicon_map', default_value='true'),
        DeclareLaunchArgument('file_map', default_value='false'),
        DeclareLaunchArgument('initial_position/x', default_value='0.0'),
        DeclareLaunchArgument('initial_position/y', default_value='1.0'),
        DeclareLaunchArgument('initial_position/z', default_value='0.0'),
        DeclareLaunchArgument('color/r', default_value='0.0'),
        DeclareLaunchArgument('color/g', default_value='0.0'),
        DeclareLaunchArgument('color/b', default_value='1.0'),
        DeclareLaunchArgument('color/a', default_value='0.7'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('map_size_x', default_value='20'),
        DeclareLaunchArgument('map_size_y', default_value='10'),
        DeclareLaunchArgument('map_size_z', default_value='5'),
        DeclareLaunchArgument('map_x_origin', default_value='-10.0'),
        DeclareLaunchArgument('map_y_origin', default_value='-5.0'),
        DeclareLaunchArgument('map_z_origin', default_value='0.0'),
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('planner_config', default_value=default_planner_config),
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('point_cloud', default_value='/global_cloud'),

        Node(
            package='action_planner',
            executable='local_plan_server',
            name='local_plan_server',
            output='screen',
            respawn=True,
            parameters=[
                planner_config,
                {
                    'poly_srv_name': ['/', mav_name, '/mav_services/poly_tracker'],
                },
            ],
            remappings=[
                ('~/local_voxel_map', 'mapper/local_voxel_map'),
                ('~/local_noinfla_voxel_map', 'mapper/local_voxel_no_inflation_map'),
                ('~/tracker_cmd', ['/', mav_name, '/trackers_manager/poly_tracker/PolyTracker/goal']),
                ('~/tracker_client', ['/', mav_name, '/trackers_manager/poly_tracker/PolyTracker']),
            ],
        ),

        # Random map
        GroupAction(
            actions=[
                Node(
                    package='param_env',
                    executable='structure_map',
                    name='structure_map',
                    output='screen',
                    parameters=[{
                        'map/x_size': map_size_x,
                        'map/y_size': map_size_y,
                        'map/z_size': map_size_z,
                        'map/x_origin': map_x_origin,
                        'map/y_origin': map_y_origin,
                        'map/z_origin': map_z_origin,
                        'map/resolution': 0.1,
                        'map/frame_id': world_frame_id,
                        'map/auto_change': False,
                        'params/cylinder_ratio': 0.02,
                        'params/circle_ratio': 0.02,
                        'params/gate_ratio': 0.02,
                        'params/ellip_ratio': 0.02,
                        'params/poly_ratio': 0.02,
                        'params/w1': 0.1,
                        'params/w2': 0.6,
                        'params/w3': 1.2,
                        'params/w4': 2.0,
                        'params/add_noise': False,
                        'params/seed': 35.8,
                        'dataset/save_map': True,
                        'dataset/samples_num': 100,
                        'dataset/start_index': 5080900,
                    }],
                    remappings=[('~/global_cloud', point_cloud)],
                ),
            ],
            condition=IfCondition(random_map),
        ),

        # File map
        GroupAction(
            actions=[
                Node(
                    package='param_env',
                    executable='read_grid_map',
                    name='read_grid_map',
                    output='screen',
                    parameters=[{
                        'map/x_size': map_size_x,
                        'map/y_size': map_size_y,
                        'map/z_size': map_size_z,
                        'map/x_origin': map_x_origin,
                        'map/y_origin': map_y_origin,
                        'map/z_origin': map_z_origin,
                        'map/resolution': 0.1,
                        'map/frame_id': world_frame_id,
                        'map/inflate_radius': 0.2,
                        'map/auto_change': False,
                        'map/publish_grid_centers': True,
                        'map/mode': 1,
                        'use_folder': True,
                        'img/occ_th': 0.6,
                    }],
                    remappings=[('~/global_gridmap', point_cloud)],
                ),
            ],
            condition=IfCondition(file_map),
        ),

        # Vicon map
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([vicon_launch]),
                    launch_arguments={'map_frame_id': world_frame_id}.items(),
                ),
            ],
            condition=IfCondition(vicon_map),
        ),

        # Rviz
        GroupAction(
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz',
                    arguments=['-d', PathJoinSubstitution([
                        FindPackageShare('map_plan_launch'), 'rviz', 'run_in_sim.rviz'
                    ]), '-f', 'map'],
                    respawn=False,
                    output='screen',
                ),
            ],
            condition=IfCondition(LaunchConfiguration('viz')),
        ),
    ])
