from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    mapper_config = LaunchConfiguration('mapper_config')
    planner_config = LaunchConfiguration('planner_config')
    viz = LaunchConfiguration('viz')

    default_mapper_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'config', 'mapper.yaml'
    ])
    default_planner_config = PathJoinSubstitution([
        FindPackageShare('control_launch'), 'config', 'tracker_params_mp.yaml'
    ])
    mapper_launch = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'launch', 'mapper.launch.py'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('map_plan_launch'), 'rviz', 'planner_standalone.rviz'
    ])
    image_to_map_launch = PathJoinSubstitution([
        FindPackageShare('kr_planning_msgs_utils'), 'launch', 'image_to_map.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value='quadrotor'),
        ),
        DeclareLaunchArgument('mapper_config', default_value=default_mapper_config),
        DeclareLaunchArgument('planner_config', default_value=default_planner_config),
        DeclareLaunchArgument('viz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([mapper_launch]),
            launch_arguments={
                'odom_frame': 'map',
                'map_frame': 'map',
                'mapper_config': mapper_config,
                'cloud': '/global_cloud',
            }.items(),
        ),

        Node(
            package='action_planner',
            executable='local_plan_server',
            name='local_plan_server',
            output='screen',
            respawn=True,
            parameters=[planner_config],
            remappings=[
                ('~/local_voxel_map', 'mapper/local_voxel_map'),
                ('~/local_noinfla_voxel_map', 'mapper/local_voxel_no_inflation_map'),
            ],
        ),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([image_to_map_launch]),
        ),
    ])
