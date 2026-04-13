"""Best-effort conversion of run_in_sim.launch.

The original launch file instantiated 10 local_plan_server nodes with different
planner combinations for evaluation. That structure is preserved here with a
helper. Some `param_env` and nested simulator includes are best-effort.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _local_plan_server(name, search_type, opt_type=None, mav_name=None,
                       planner_config=None, point_cloud=None, map_name=None,
                       mass=None, grav_acc=None, min_thrust=None,
                       max_thrust=None, mav_radius=None):
    params = [
        planner_config,
        {
            'trajectory_planner/search_planner_type': search_type,
            'mav_name': mav_name,
            'map_name': map_name,
            'mav_radius': mav_radius,
            'trajectory_planner/VehicleMass': mass,
            'trajectory_planner/GravAcc': grav_acc,
            'trajectory_planner/MinThrust': min_thrust,
            'trajectory_planner/MaxThrust': max_thrust,
            'poly_srv_name': ['/', mav_name, '/mav_services/poly_tracker'],
        },
    ]
    if opt_type is not None:
        params[1]['trajectory_planner/opt_planner_type'] = opt_type

    remappings = [
        ('~/local_voxel_map', 'mapper/local_voxel_map'),
        ('~/local_noinfla_voxel_map', 'mapper/local_voxel_no_inflation_map'),
        ('~/tracker_cmd', ['/', mav_name, '/trackers_manager/poly_tracker/PolyTracker/goal']),
        ('~/tracker_client', ['/', mav_name, '/trackers_manager/poly_tracker/PolyTracker']),
    ]
    if point_cloud is not None and search_type != 0:
        remappings.append(('~/global_cloud', point_cloud))

    return Node(
        package='action_planner',
        executable='local_plan_server',
        name=name,
        output='screen',
        respawn=True,
        parameters=params,
        remappings=remappings,
    )


def generate_launch_description():
    mav_name = LaunchConfiguration('mav_name')
    planner_config = LaunchConfiguration('planner_config')
    mapper_config = LaunchConfiguration('mapper_config')
    world_frame_id = LaunchConfiguration('world_frame_id')
    point_cloud = LaunchConfiguration('point_cloud')
    map_name = LaunchConfiguration('map_name')
    mass = LaunchConfiguration('mass')
    grav_acc = LaunchConfiguration('grav_acc')
    min_thrust = LaunchConfiguration('min_thrust')
    max_thrust = LaunchConfiguration('max_thrust')
    mav_radius = LaunchConfiguration('mav_radius')
    viz = LaunchConfiguration('viz')
    vicon_map = LaunchConfiguration('vicon_map')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    map_x_origin = LaunchConfiguration('map_x_origin')
    map_y_origin = LaunchConfiguration('map_y_origin')
    map_z_origin = LaunchConfiguration('map_z_origin')
    map_resolution = LaunchConfiguration('map_resolution')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    default_planner_config = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'rviz', 'run_in_sim.rviz'
    ])
    example_control_launch = PathJoinSubstitution([
        FindPackageShare('kr_mav_launch'), 'launch', 'example_control.launch.py'
    ])
    sim_launch = PathJoinSubstitution([
        FindPackageShare('kr_mav_launch'), 'launch', 'sim.launch.py'
    ])
    vicon_launch = PathJoinSubstitution([
        FindPackageShare('vicon_env'), 'launch', 'vicon_map.launch.py'
    ])

    common_kwargs = dict(
        mav_name=mav_name, planner_config=planner_config,
        point_cloud=point_cloud, map_name=map_name, mass=mass,
        grav_acc=grav_acc, min_thrust=min_thrust, max_thrust=max_thrust,
        mav_radius=mav_radius,
    )

    local_plan_servers = [
        _local_plan_server('local_plan_server0', 0, **common_kwargs),
        _local_plan_server('local_plan_server1', 1, **common_kwargs),
        _local_plan_server('local_plan_server2', 2, **common_kwargs),
        _local_plan_server('local_plan_server3', 4, **common_kwargs),
        _local_plan_server('local_plan_server4', 5, **common_kwargs),
        _local_plan_server('local_plan_server5', 0, opt_type=1, **common_kwargs),
        _local_plan_server('local_plan_server6', 1, opt_type=1, **common_kwargs),
        _local_plan_server('local_plan_server7', 2, opt_type=1, **common_kwargs),
        _local_plan_server('local_plan_server8', 4, opt_type=1, **common_kwargs),
        _local_plan_server('local_plan_server9', 5, opt_type=1, **common_kwargs),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world_frame_id', default_value='map'),
        DeclareLaunchArgument('slow_baud_rate', default_value='true'),
        DeclareLaunchArgument('nodelet_manager_name', default_value='nodelet_manager_control'),
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('planner_config', default_value=default_planner_config),
        DeclareLaunchArgument('mav_name', default_value='quadrotor'),
        DeclareLaunchArgument('mav_id', default_value='1'),
        DeclareLaunchArgument('mav_type', default_value='hummingbird'),
        DeclareLaunchArgument('odom_topic', default_value=['/', LaunchConfiguration('mav_name'), '/odom']),
        DeclareLaunchArgument('initial_position/x', default_value='9.0'),
        DeclareLaunchArgument('initial_position/y', default_value='4.5'),
        DeclareLaunchArgument('initial_position/z', default_value='0.0'),
        DeclareLaunchArgument('color/r', default_value='0.0'),
        DeclareLaunchArgument('color/g', default_value='0.0'),
        DeclareLaunchArgument('color/b', default_value='1.0'),
        DeclareLaunchArgument('color/a', default_value='0.7'),
        DeclareLaunchArgument('mass', default_value='1.5'),
        DeclareLaunchArgument('grav_acc', default_value='9.81'),
        DeclareLaunchArgument('min_thrust', default_value='-1.0'),
        DeclareLaunchArgument('max_thrust', default_value='31.0'),
        DeclareLaunchArgument('mav_radius', default_value='0.20'),
        DeclareLaunchArgument('M_Ixx', default_value='6.7e-3'),
        DeclareLaunchArgument('M_Iyy', default_value='7.0e-3'),
        DeclareLaunchArgument('M_Izz', default_value='6.3e-3'),
        DeclareLaunchArgument('arm_length', default_value='0.125'),
        DeclareLaunchArgument('kf', default_value='1.0'),
        DeclareLaunchArgument('km', default_value='0.02'),
        DeclareLaunchArgument('vicon_map', default_value='false'),
        DeclareLaunchArgument('point_cloud', default_value='/global_cloud'),
        DeclareLaunchArgument('map_name', default_value='read_grid_map'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('map_size_x', default_value='20'),
        DeclareLaunchArgument('map_size_y', default_value='10'),
        DeclareLaunchArgument('map_size_z', default_value='5'),
        DeclareLaunchArgument('map_x_origin', default_value='-10.0'),
        DeclareLaunchArgument('map_y_origin', default_value='-5.0'),
        DeclareLaunchArgument('map_z_origin', default_value='0.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),

        # Control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([example_control_launch]),
            launch_arguments={
                'nodelet_manager_name': LaunchConfiguration('nodelet_manager_name'),
                'mass': mass,
                'mav_name': mav_name,
                'mav_type': LaunchConfiguration('mav_type'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }.items(),
        ),

        # Simulator
        GroupAction(actions=[
            PushRosNamespace(mav_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([sim_launch]),
                launch_arguments={
                    'mav_name': mav_name,
                    'mav_type': LaunchConfiguration('mav_type'),
                    'world_frame_id': world_frame_id,
                    'mass': mass,
                }.items(),
            ),
        ]),

        *local_plan_servers,

        # Map generator
        Node(
            package='param_env',
            executable=map_name,
            name=map_name,
            output='screen',
            parameters=[{
                'map/x_size': map_size_x,
                'map/y_size': map_size_y,
                'map/z_size': map_size_z,
                'map/x_origin': map_x_origin,
                'map/y_origin': map_y_origin,
                'map/z_origin': map_z_origin,
                'map/resolution': map_resolution,
                'map/frame_id': world_frame_id,
                'map/inflate_radius': 0.3,
                'map/auto_change': False,
                'map/mav_radius': mav_radius,
                'map/evaluate': True,
                'params/cylinder_ratio': 0.001,
                'params/circle_ratio': 0.001,
                'params/gate_ratio': 0.001,
                'params/ellip_ratio': 0.001,
                'params/poly_ratio': 0.001,
                'params/w1': 0.05,
                'params/w2': 0.3,
                'params/w3': 0.5,
                'params/w4': 0.6,
                'params/add_noise': False,
                'params/seed': 1,
                'dataset/save_map': False,
                'dataset/samples_num': 100,
                'dataset/start_index': 5080900,
                'map/publish_grid_centers': True,
                'map/mode': 4,
                'params/density_index': 0.2,
                'params/clutter_index': 0.5,
                'params/structure index': 0.1,
                'use_folder': True,
                'img/negate': 0,
                'img/occ_th': 0.6,
            }],
            remappings=[('~/global_cloud', point_cloud)],
        ),

        # Optional vicon map
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
                    arguments=['-d', rviz_config, '-f', 'map'],
                    respawn=False,
                    output='screen',
                ),
            ],
            condition=IfCondition(viz),
        ),
    ])
