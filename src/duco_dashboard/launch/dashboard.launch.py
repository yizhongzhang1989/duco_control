"""Launch the Duco robot monitoring dashboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FALLBACKS = {
    'host': '0.0.0.0',
    'port': 8090,
    'joint_states_topic': '/joint_states',
    'wrench_topic': '/duco_ft_sensor/wrench_raw',
    'controller_state_topic': '/arm_1_controller/state',
    'push_rate': 20.0,
    'stale_after': 1.0,
}


def generate_launch_description():
    defaults, source = _defaults()
    args = [
        DeclareLaunchArgument('host', default_value=str(defaults['host'])),
        DeclareLaunchArgument('port', default_value=str(defaults['port'])),
        DeclareLaunchArgument('joint_states_topic', default_value=str(defaults['joint_states_topic'])),
        DeclareLaunchArgument('wrench_topic', default_value=str(defaults['wrench_topic'])),
        DeclareLaunchArgument('controller_state_topic', default_value=str(defaults['controller_state_topic'])),
        DeclareLaunchArgument('push_rate', default_value=str(defaults['push_rate'])),
        DeclareLaunchArgument('stale_after', default_value=str(defaults['stale_after'])),
    ]
    log = LogInfo(msg=(
        f"[duco_dashboard] config: {source}; "
        f"defaults: port={defaults['port']} joint_states={defaults['joint_states_topic']} "
        f"controller={defaults['controller_state_topic']}"))
    node = Node(
        package='duco_dashboard',
        executable='dashboard',
        name='duco_dashboard',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'wrench_topic': LaunchConfiguration('wrench_topic'),
            'controller_state_topic': LaunchConfiguration('controller_state_topic'),
            'push_rate': LaunchConfiguration('push_rate'),
            'stale_after': LaunchConfiguration('stale_after'),
        }],
    )
    return LaunchDescription([log, *args, node])


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
    if not cfg.has('duco_dashboard'):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'duco_dashboard:' section in {cfg.config_path})")
    section = cfg.section('duco_dashboard')
    return ({key: section.get(key, value) for key, value in _FALLBACKS.items()},
            f'loaded from {cfg.config_path}')