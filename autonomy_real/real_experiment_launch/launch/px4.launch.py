from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')

    mavros_launch = PathJoinSubstitution([
        FindPackageShare('mavros'), 'launch', 'node.launch.py'
    ])
    pluginlists_yaml = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'px4_pluginlists.yaml'
    ])
    config_yaml = PathJoinSubstitution([
        FindPackageShare('real_experiment_launch'), 'config', 'px4_config.yaml'
    ])

    return LaunchDescription([
        # example launch script for PX4 based FCUs
        # for rhea
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTGARU2U-if00-port0:921600',
        ),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([mavros_launch]),
            launch_arguments={
                'pluginlists_yaml': pluginlists_yaml,
                'config_yaml': config_yaml,
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'tgt_system': tgt_system,
                'tgt_component': tgt_component,
                'log_output': log_output,
                'fcu_protocol': fcu_protocol,
                'respawn_mavros': respawn_mavros,
            }.items(),
        ),
    ])
