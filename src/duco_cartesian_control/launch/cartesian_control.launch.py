"""Launch the Cartesian-control orchestrator + spawn FZI's controller (inactive).

Defaults are loaded from ``config/robot_config.yaml`` under
``duco_cartesian_control:`` (via the ``common`` package) when present,
with hard-coded fallbacks so the launch still works on a fresh checkout.

The launch also spawns FZI's ``cartesian_force_controller`` into the
live ``controller_manager`` in the **inactive** state.  Our orchestrator
node will activate / deactivate it on engage / disengage via
``switch_controller``.

Examples::

    ros2 launch duco_cartesian_control cartesian_control.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FALLBACKS = {
    # connectivity ---------------------------------------------------------
    "wrench_topic":          "/duco_ft_sensor/wrench_compensated",
    "joint_states_topic":    "/joint_states",
    "controller_manager_ns": "/controller_manager",
    "engaged_default":       False,
    # FZI controller wiring ------------------------------------------------
    "fzi_controller_name":     "cartesian_force_controller",
    "fzi_jtc_controller_name": "arm_1_controller",
    "fzi_ft_topic":            "/cartesian_force_controller/ft_sensor_wrench",
    "fzi_target_topic":        "/cartesian_force_controller/target_wrench",
    "fzi_target_frame":        "link_6",
    "fzi_target_rate_hz":      10.0,
    "fzi_service_timeout_sec": 2.0,
    # supervisor + state publish ------------------------------------------
    "loop_rate_hz":            50.0,
    "state_publish_rate_hz":   5.0,
    # safety ---------------------------------------------------------------
    "max_wrench_force":          80.0,
    "max_wrench_torque":         10.0,
    "engage_max_joint_velocity": 0.05,
    "ft_stale_after":            0.25,
    "joint_states_stale_after":  0.25,
}


def _defaults():
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
    if not cfg.has("duco_cartesian_control"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'duco_cartesian_control:' section in "
                f"{cfg.config_path})")
    sec = cfg.section("duco_cartesian_control")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def _bool(v):
    return str(v).lower()


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    args = []
    for key, default in _FALLBACKS.items():
        if isinstance(default, bool):
            args.append(DeclareLaunchArgument(key, default_value=_bool(d[key])))
        else:
            args.append(DeclareLaunchArgument(key, default_value=str(d[key])))

    log = LogInfo(msg=(
        f"[duco_cartesian_control] config: {source}"))

    parameters = {key: LaunchConfiguration(key) for key in _FALLBACKS.keys()}

    node = Node(
        package="duco_cartesian_control",
        executable="cartesian_control_node",
        name="duco_cartesian_control",
        output="screen",
        emulate_tty=True,
        parameters=[parameters],
    )

    # Pre-load FZI's controller into the live controller_manager
    # (inactive).  Our orchestrator will activate it on engage via the
    # SwitchController service.
    pkg_share = get_package_share_directory("duco_cartesian_control")
    fzi_yaml = os.path.join(pkg_share, "config", "fzi_zero_gravity.yaml")
    fzi_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_force_controller",
            "-c", LaunchConfiguration("controller_manager_ns"),
            "-p", fzi_yaml,
            "-t", "cartesian_force_controller/CartesianForceController",
            "--inactive",
        ],
        output="screen",
    )

    return LaunchDescription([log, *args, fzi_spawner, node])
