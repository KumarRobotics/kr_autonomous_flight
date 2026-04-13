from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    takeoff_height = LaunchConfiguration('takeoff_height')
    max_horizon = LaunchConfiguration('max_horizon')

    smach_launch = PathJoinSubstitution([
        FindPackageShare('state_machine_launch'), 'launch', 'smach.launch.py'
    ])
    replanner_launch = PathJoinSubstitution([
        FindPackageShare('state_machine_launch'), 'launch', 'replanner.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value='quadrotor'),
        ),
        DeclareLaunchArgument('takeoff_height', default_value='2'),
        DeclareLaunchArgument('max_horizon', default_value='5'),

        GroupAction(actions=[
            PushRosNamespace(robot),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([smach_launch]),
                launch_arguments={'takeoff_height': takeoff_height}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([replanner_launch]),
                launch_arguments={'max_horizon': max_horizon}.items(),
            ),
        ]),
    ])
