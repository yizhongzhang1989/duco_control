import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_FALLBACKS = {
    'robot_ip': '192.168.1.10',
    'robot_port': 7003,
    'use_fake_hardware': True,
    'use_rviz': True,
    'db': False,
    'debug': False,
    'publish_frequency': 15.0,
}


def generate_launch_description():
    defaults, source = _defaults()

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value=str(defaults['robot_ip']),
            description='Robot IP address'),
        DeclareLaunchArgument(
            'robot_port',
            default_value=str(defaults['robot_port']),
            description='Robot controller TCP port'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=_launch_bool(defaults['use_fake_hardware']),
            description='Use mock_components instead of the real Duco hardware interface'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=_launch_bool(defaults['use_rviz']),
            description='Launch the MoveIt RViz UI'),
        DeclareLaunchArgument(
            'db',
            default_value=_launch_bool(defaults['db']),
            description='Launch the MoveIt warehouse database'),
        DeclareLaunchArgument(
            'debug',
            default_value=_launch_bool(defaults['debug']),
            description='Enable debug mode in the upstream MoveIt launch'),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value=str(defaults['publish_frequency']),
            description='robot_state_publisher TF rate in Hz'),
    ]

    upstream_launch = os.path.join(
        get_package_share_directory('duco_gcr5_910_moveit_config'),
        'launch',
        'demo_ros2_control.launch.py')

    forwarded_arguments = {
        name: LaunchConfiguration(name)
        for name in _FALLBACKS.keys()
    }

    return LaunchDescription([
        *declared_arguments,
        LogInfo(msg=(
            f'[duco_robot_bringup] config: {source}; '
            f'defaults: robot_ip={defaults["robot_ip"]} '
            f'robot_port={defaults["robot_port"]} '
            f'use_fake_hardware={defaults["use_fake_hardware"]} '
            f'use_rviz={defaults["use_rviz"]}')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            launch_arguments=forwarded_arguments.items()),
    ])


def _defaults():
    try:
        from common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f'FALLBACK (could not import common.config_manager: '
                f'{type(exc).__name__}: {exc})')

    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f'FALLBACK (could not load config: '
                f'{type(exc).__name__}: {exc})')

    section_name = 'duco_robot_bringup'
    if not cfg.has(section_name):
        return (dict(_FALLBACKS),
                f"FALLBACK (no '{section_name}:' section in {cfg.config_path})")

    section = cfg.section(section_name)
    return ({key: section.get(key, value) for key, value in _FALLBACKS.items()},
            f'loaded from {cfg.config_path}')


def _launch_bool(value):
    return str(value).lower()