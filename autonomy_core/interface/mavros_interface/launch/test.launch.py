"""ROS 2 launch file for the SO3CmdToMavros composable node (test/DJI E310)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    robot_arg = DeclareLaunchArgument('robot', default_value='/')
    odom_arg = DeclareLaunchArgument('odom', default_value='odom')
    so3_cmd_arg = DeclareLaunchArgument('so3_cmd', default_value='so3_cmd')
    num_props_arg = DeclareLaunchArgument('num_props', default_value='4')
    use_kartik_arg = DeclareLaunchArgument(
        'use_kartik_thrust_model', default_value='false')

    # Kartik model DJI E310
    thrust_vs_rpm_coeff_a_arg = DeclareLaunchArgument(
        'thrust_vs_rpm_coeff_a', default_value='1.3414e-7')
    thrust_vs_rpm_coeff_b_arg = DeclareLaunchArgument(
        'thrust_vs_rpm_coeff_b', default_value='-1.1226e-4')
    thrust_vs_rpm_coeff_c_arg = DeclareLaunchArgument(
        'thrust_vs_rpm_coeff_c', default_value='0.0509')
    rpm_vs_throttle_coeff_a_arg = DeclareLaunchArgument(
        'rpm_vs_throttle_coeff_a', default_value='14113.7')
    rpm_vs_throttle_coeff_b_arg = DeclareLaunchArgument(
        'rpm_vs_throttle_coeff_b', default_value='-1855.7')

    # Mike model
    thrust_constant_arg = DeclareLaunchArgument(
        'thrust_constant', default_value='-5.340158886849379')
    thrust_vs_throttle_arg = DeclareLaunchArgument(
        'thrust_vs_throttle', default_value='13.537618230563210')
    thrust_vs_battery_arg = DeclareLaunchArgument(
        'thrust_vs_battery', default_value='0.385461527865615')

    container = ComposableNodeContainer(
        name='so3cmd_to_mavros_container',
        namespace=LaunchConfiguration('robot'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mavros_interface',
                plugin='mavros_interface::SO3CmdToMavros',
                name='so3cmd_to_mavros',
                namespace=LaunchConfiguration('robot'),
                parameters=[{
                    'num_props': LaunchConfiguration('num_props'),
                    'thrust_vs_rpm_coeff_a': LaunchConfiguration('thrust_vs_rpm_coeff_a'),
                    'thrust_vs_rpm_coeff_b': LaunchConfiguration('thrust_vs_rpm_coeff_b'),
                    'thrust_vs_rpm_coeff_c': LaunchConfiguration('thrust_vs_rpm_coeff_c'),
                    'rpm_vs_throttle_coeff_a': LaunchConfiguration('rpm_vs_throttle_coeff_a'),
                    'rpm_vs_throttle_coeff_b': LaunchConfiguration('rpm_vs_throttle_coeff_b'),
                    'thrust_constant': LaunchConfiguration('thrust_constant'),
                    'thrust_vs_throttle': LaunchConfiguration('thrust_vs_throttle'),
                    'thrust_vs_battery': LaunchConfiguration('thrust_vs_battery'),
                    'use_kartik_thrust_model': LaunchConfiguration('use_kartik_thrust_model'),
                    'check_psi': False,
                }],
                remappings=[
                    ('odom', LaunchConfiguration('odom')),
                    ('so3_cmd', LaunchConfiguration('so3_cmd')),
                    ('imu', 'mavros/imu/data'),
                    ('attitude_raw', 'mavros/setpoint_raw/attitude'),
                    ('odom_pose', 'mavros/vision_pose/pose'),
                    ('battery', 'mavros/battery'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        odom_arg,
        so3_cmd_arg,
        num_props_arg,
        use_kartik_arg,
        thrust_vs_rpm_coeff_a_arg,
        thrust_vs_rpm_coeff_b_arg,
        thrust_vs_rpm_coeff_c_arg,
        rpm_vs_throttle_coeff_a_arg,
        rpm_vs_throttle_coeff_b_arg,
        thrust_constant_arg,
        thrust_vs_throttle_arg,
        thrust_vs_battery_arg,
        container,
    ])
