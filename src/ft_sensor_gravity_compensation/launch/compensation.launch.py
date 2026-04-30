"""Launch the gravity-compensation node + its web dashboard.

Defaults are taken from ``config/robot_config.yaml`` under
``ft_sensor_gravity_compensation`` (via the ``common`` package). CLI overrides
win.

Examples:
  ros2 launch ft_sensor_gravity_compensation compensation.launch.py
  ros2 launch ft_sensor_gravity_compensation compensation.launch.py \\
      enable_dashboard:=true
  ros2 launch ft_sensor_gravity_compensation compensation.launch.py port:=8101
  ros2 launch ft_sensor_gravity_compensation compensation.launch.py \\
      input_topic:=/duco_ft_sensor/wrench_raw \\
      output_topic:=/ft/wrench_compensated \\
      sensor_frame:=ft_sensor_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FALLBACKS = {
    "input_topic":  "/duco_ft_sensor/wrench_raw",
    "output_topic": "/duco_ft_sensor/wrench_compensated",
    "world_frame":  "base_link",
    "sensor_frame": "link_6",
    "reliability":  "best_effort",
    "publish_when_no_tf": False,
    "storage_path": "~/.ros/ft_sensor_gravity_compensation/end_effectors.yaml",
    "enable_dashboard": False,
    "host": "0.0.0.0",
    "port": 8100,
    "gravity": 9.80665,
    "tf_timeout": 0.05,
    "tf_max_age": 1.0,
}


def _defaults():
    """Return ``(defaults_dict, source_str)``.

    Mirrors the pattern used by the other packages in this workspace: try to
    load the central config, but fall back to ``_FALLBACKS`` so the launcher
    still works on a fresh checkout. The source string is logged at launch so
    operators can see whether the central config was honoured.
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
    sec = (cfg.section("ft_sensor_gravity_compensation")
           if cfg.has("ft_sensor_gravity_compensation") else None)
    if sec is None:
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'ft_sensor_gravity_compensation:' section "
                f"in {cfg.config_path})")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def _launch_bool(value):
    return str(value).lower()


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    args = [
        DeclareLaunchArgument("input_topic",  default_value=str(d["input_topic"])),
        DeclareLaunchArgument("output_topic", default_value=str(d["output_topic"])),
        DeclareLaunchArgument("world_frame",  default_value=str(d["world_frame"])),
        DeclareLaunchArgument("sensor_frame", default_value=str(d["sensor_frame"])),
        DeclareLaunchArgument(
            "reliability", default_value=str(d["reliability"]),
            description="best_effort or reliable (matches the publisher's QoS)"),
        DeclareLaunchArgument(
            "publish_when_no_tf",
            default_value=_launch_bool(d["publish_when_no_tf"]),
            description="if true, publish bias-only wrench when TF is stale"),
        DeclareLaunchArgument("storage_path", default_value=str(d["storage_path"])),
        DeclareLaunchArgument(
            "enable_dashboard",
            default_value=_launch_bool(d["enable_dashboard"]),
            description="if true, start the embedded web dashboard on host:port"),
        DeclareLaunchArgument(
            "host", default_value=str(d["host"]),
            description="HTTP bind address; 0.0.0.0 = LAN-visible"),
        DeclareLaunchArgument("port", default_value=str(d["port"])),
        DeclareLaunchArgument("gravity", default_value=str(d["gravity"])),
        DeclareLaunchArgument("tf_timeout", default_value=str(d["tf_timeout"])),
        DeclareLaunchArgument("tf_max_age", default_value=str(d["tf_max_age"])),
    ]

    log = LogInfo(msg=(
        f"[ft_sensor_gravity_compensation] config: {source}; "
        f"defaults: input_topic={d['input_topic']} "
        f"output_topic={d['output_topic']} "
        f"world_frame={d['world_frame']} sensor_frame={d['sensor_frame']} "
        f"enable_dashboard={d['enable_dashboard']} port={d['port']}"))

    node = Node(
        package="ft_sensor_gravity_compensation",
        executable="compensation_node",
        name="ft_sensor_gravity_compensation",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic":  LaunchConfiguration("input_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "world_frame":  LaunchConfiguration("world_frame"),
            "sensor_frame": LaunchConfiguration("sensor_frame"),
            "reliability":  LaunchConfiguration("reliability"),
            "publish_when_no_tf": LaunchConfiguration("publish_when_no_tf"),
            "storage_path": LaunchConfiguration("storage_path"),
            "enable_dashboard": LaunchConfiguration("enable_dashboard"),
            "host":         LaunchConfiguration("host"),
            "port":         LaunchConfiguration("port"),
            "gravity":      LaunchConfiguration("gravity"),
            "tf_timeout":   LaunchConfiguration("tf_timeout"),
            "tf_max_age":   LaunchConfiguration("tf_max_age"),
        }],
    )

    return LaunchDescription([log, *args, node])
