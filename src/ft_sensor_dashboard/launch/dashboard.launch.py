"""Launch the web-based F/T sensor dashboard.

Defaults are taken from ``config/robot_config.yaml`` under
``dashboards.ft_sensor`` (via the ``common`` package). CLI overrides win.

Examples:
  ros2 launch ft_sensor_dashboard dashboard.launch.py
  ros2 launch ft_sensor_dashboard dashboard.launch.py port:=8081
  ros2 launch ft_sensor_dashboard dashboard.launch.py topic:=/my_other_sensor/wrench
  ros2 launch ft_sensor_dashboard dashboard.launch.py reliability:=reliable
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Hard-coded fallbacks if the central config is missing or doesn't
# contain a particular key. Match the historical defaults.
_FALLBACKS = {
    "topic": "/duco_ft_sensor/wrench_raw",
    "host": "0.0.0.0",
    "port": 8080,
    "window_seconds": 10.0,
    "push_rate": 30.0,
    "reliability": "best_effort",
    "title": "",
}


def _defaults():
    """Return (defaults_dict, source_str).

    ``source_str`` describes where the values came from -- printed by
    LogInfo so a launch operator can immediately see whether the
    central config was honored or whether the launcher fell back to
    the hard-coded defaults.
    """
    try:
        from common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not import common.config_manager: "
                f"{type(exc).__name__}: {exc})")
    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not load config: "
                f"{type(exc).__name__}: {exc})")
    sec = cfg.section("ft_sensor_dashboard") if cfg.has("ft_sensor_dashboard") else None
    if sec is None:
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'ft_sensor_dashboard:' section in {cfg.config_path})")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    args = [
        DeclareLaunchArgument("topic", default_value=str(d["topic"])),
        DeclareLaunchArgument(
            "host", default_value=str(d["host"]),
            description="HTTP bind address; 0.0.0.0 = LAN-visible"),
        DeclareLaunchArgument("port", default_value=str(d["port"])),
        DeclareLaunchArgument(
            "window_seconds", default_value=str(d["window_seconds"])),
        DeclareLaunchArgument(
            "push_rate", default_value=str(d["push_rate"]),
            description="SSE updates per second"),
        DeclareLaunchArgument(
            "reliability", default_value=str(d["reliability"]),
            description="best_effort or reliable"),
        DeclareLaunchArgument("title", default_value=str(d["title"])),
    ]

    log = LogInfo(msg=(
        f"[ft_sensor_dashboard] config: {source}; "
        f"defaults: port={d['port']} host={d['host']} topic={d['topic']}"))

    node = Node(
        package="ft_sensor_dashboard",
        executable="dashboard",
        name="ft_sensor_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "topic": LaunchConfiguration("topic"),
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "window_seconds": LaunchConfiguration("window_seconds"),
            "push_rate": LaunchConfiguration("push_rate"),
            "reliability": LaunchConfiguration("reliability"),
            "title": LaunchConfiguration("title"),
        }],
    )

    return LaunchDescription([log, *args, node])
