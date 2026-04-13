from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    odom = LaunchConfiguration('odom')
    so3_cmd = LaunchConfiguration('so3_cmd')
    num_props = LaunchConfiguration('num_props')
    thrust_vs_rpm_coeff_a = LaunchConfiguration('thrust_vs_rpm_coeff_a')
    thrust_vs_rpm_coeff_b = LaunchConfiguration('thrust_vs_rpm_coeff_b')
    thrust_vs_rpm_coeff_c = LaunchConfiguration('thrust_vs_rpm_coeff_c')
    rpm_vs_throttle_coeff_a = LaunchConfiguration('rpm_vs_throttle_coeff_a')
    rpm_vs_throttle_coeff_b = LaunchConfiguration('rpm_vs_throttle_coeff_b')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=EnvironmentVariable('ROBOT_NAME', default_value='quadrotor'),
        ),
        DeclareLaunchArgument('odom', default_value='odom'),
        DeclareLaunchArgument('so3_cmd', default_value='so3_cmd'),
        DeclareLaunchArgument('num_props', default_value='4.0'),

        # Falcon 4 15" 1575 props FAKE (new manufacturer)
        # Thrust = a * RPM^2 + b * RPM + c (Newtons)
        DeclareLaunchArgument('thrust_vs_rpm_coeff_a', default_value='5.098e-7'),
        DeclareLaunchArgument('thrust_vs_rpm_coeff_b', default_value='-2.90571e-4'),
        DeclareLaunchArgument('thrust_vs_rpm_coeff_c', default_value='0.126701918'),

        # Falcon 4 coeffs: RPM = a * throttle + b where throttle in [0,1]
        DeclareLaunchArgument('rpm_vs_throttle_coeff_a', default_value='6000'),
        DeclareLaunchArgument('rpm_vs_throttle_coeff_b', default_value='0'),

        GroupAction(actions=[
            PushRosNamespace(robot),
            Node(
                package='mavros_interface',
                executable='so3cmd_to_mavros_node',
                name='so3cmd_to_mavros',
                output='screen',
                parameters=[{
                    'num_props': num_props,
                    'thrust_vs_rpm_coeff_a': thrust_vs_rpm_coeff_a,
                    'thrust_vs_rpm_coeff_b': thrust_vs_rpm_coeff_b,
                    'thrust_vs_rpm_coeff_c': thrust_vs_rpm_coeff_c,
                    'rpm_vs_throttle_coeff_a': rpm_vs_throttle_coeff_a,
                    'rpm_vs_throttle_coeff_b': rpm_vs_throttle_coeff_b,
                }],
                remappings=[
                    ('~/odom', odom),
                    ('~/so3_cmd', so3_cmd),
                    ('~/imu', 'mavros/imu/data'),
                    ('~/attitude_raw', 'mavros/setpoint_raw/attitude'),
                    ('~/odom_pose', 'mavros/vision_pose/pose'),
                ],
            ),
        ]),
    ])
