"""Launch the Duco F/T sensor node.

Defaults are taken from ``config/robot_config.yaml`` under
``duco.ft_sensor`` (via the ``common`` package). CLI overrides win.

Examples:
  ros2 launch duco_ft_sensor ft_sensor.launch.py
  ros2 launch duco_ft_sensor ft_sensor.launch.py port:=/dev/ttyUSB0 frame_id:=tool0
  ros2 launch duco_ft_sensor ft_sensor.launch.py tare_on_start:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Hard-coded fallbacks if the central config is missing or doesn't
# contain a particular key. These match the historical defaults so the
# launcher still works on a clean checkout.
_FALLBACKS = {
    "port": "/dev/ttyUSB0",
    "baud": 460800,
    "frame_id": "ft_sensor_link",
    "publish_rate": 0.0,
    "autostart": True,
    "tare_on_start": False,
}


def _defaults():
    """Return (defaults_dict, source_str).

    ``source_str`` describes where the values came from. We never
    silently swallow errors -- if the central config can't be loaded
    the source string says why, so the LogInfo line at launch time
    makes it obvious.
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
    sec = cfg.section("duco_ft_sensor") if cfg.has("duco_ft_sensor") else None
    if sec is None:
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'duco_ft_sensor:' section in {cfg.config_path})")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    args = [
        DeclareLaunchArgument("port", default_value=str(d["port"])),
        DeclareLaunchArgument("baud", default_value=str(d["baud"])),
        DeclareLaunchArgument("frame_id", default_value=str(d["frame_id"])),
        DeclareLaunchArgument(
            "publish_rate", default_value=str(d["publish_rate"]),
            description="0.0 = publish every frame (~960 Hz)"),
        DeclareLaunchArgument(
            "autostart", default_value=str(d["autostart"]).lower()),
        DeclareLaunchArgument(
            "tare_on_start", default_value=str(d["tare_on_start"]).lower()),
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    log = LogInfo(msg=(
        f"[duco_ft_sensor] config: {source}; "
        f"defaults: port={d['port']} baud={d['baud']} "
        f"frame_id={d['frame_id']}"))

    node = Node(
        package="duco_ft_sensor",
        executable="ft_sensor_node",
        name="duco_ft_sensor",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "autostart": LaunchConfiguration("autostart"),
            "tare_on_start": LaunchConfiguration("tare_on_start"),
        }],
    )

    return LaunchDescription([log, *args, node])
