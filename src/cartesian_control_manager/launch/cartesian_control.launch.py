"""Launch the Cartesian-control orchestrator + spawn FZI's controllers (inactive).

Defaults are loaded from ``config/robot_config.yaml`` under
``cartesian_control_manager:`` (via the ``common`` package) when present,
with hard-coded fallbacks so the launch still works on a fresh checkout.

The launch spawns each FZI Cartesian controller plugin into the live
``controller_manager`` in the **inactive** state.  The orchestrator
node will then activate / deactivate the **selected** one on engage /
disengage via ``switch_controller``.  By default three controllers are
loaded:

* ``cartesian_force_controller``
* ``cartesian_motion_controller``
* ``cartesian_compliance_controller``

The default selection is ``cartesian_force_controller``; switch live
via ``ros2 param set /cartesian_control_manager active_controller_name
<name>`` (must be disengaged first), or use the
``cartesian_controller_dashboard`` UI.

The FZI controller YAML (joint names, base/EE frames, PD gains) is
**not** shipped by this package -- it is robot-specific.  The launch
takes a ``(fzi_controller_yaml_package, fzi_controller_yaml_relpath)``
pair pointing into a per-robot bringup package; the path is resolved
at launch time via ``ament_index``.

Examples::

    ros2 launch cartesian_control_manager cartesian_control.launch.py
    ros2 launch cartesian_control_manager cartesian_control.launch.py \\
        active_controller_name:=cartesian_compliance_controller
    ros2 launch cartesian_control_manager cartesian_control.launch.py \\
        fzi_controller_yaml_package:=ur_robot_bringup \\
        fzi_controller_yaml_relpath:=config/fzi_preset.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Catalogue of FZI Cartesian-controller plugins this launch file knows
# how to spawn.  Each entry is ``(name, kind, plugin_type)`` where
# ``kind`` is consumed by the orchestrator (drives which heartbeat
# topics it publishes) and ``plugin_type`` is the pluginlib class the
# spawner needs to register the controller with controller_manager.
_FZI_CONTROLLERS = [
    ("cartesian_force_controller",       "force",
     "cartesian_force_controller/CartesianForceController"),
    ("cartesian_motion_controller",      "motion",
     "cartesian_motion_controller/CartesianMotionController"),
    ("cartesian_compliance_controller",  "compliance",
     "cartesian_compliance_controller/CartesianComplianceController"),
]


_FALLBACKS = {
    # connectivity ---------------------------------------------------------
    "wrench_topic":          "/ft_sensor/wrench_compensated",
    "joint_states_topic":    "/joint_states",
    "controller_manager_ns": "/controller_manager",
    "engaged_default":       False,
    # FZI controller wiring ------------------------------------------------
    # Default JTC name 'joint_trajectory_controller' matches ros2_control's
    # convention; per-robot configs override (Duco uses 'arm_1_controller',
    # UR uses 'scaled_joint_trajectory_controller').
    "active_controller_name":  "cartesian_force_controller",
    "fzi_jtc_controller_name": "joint_trajectory_controller",
    "fzi_target_frame":        "tool0",
    "fzi_target_rate_hz":      10.0,
    "fzi_service_timeout_sec": 2.0,
    # FZI controller YAML location.  Resolved at launch time as
    # ``get_package_share_directory(<package>) / <relpath>``.
    # Each per-robot bringup package ships its own preset.
    "fzi_controller_yaml_package": "duco_robot_bringup",
    "fzi_controller_yaml_relpath": "config/fzi_preset.yaml",
    # target_wrench setpoint published by the heartbeat (fallback when
    # no external publisher is active).  Interpreted by FZI in the
    # end-effector frame (hand_frame_control:=true, default), or the
    # robot base frame (hand_frame_control:=false).  All-zero ==
    # pure free-drive (sensor-only -> compliance to operator pushes).
    "target_wrench_force_x":   0.0,
    "target_wrench_force_y":   0.0,
    "target_wrench_force_z":   0.0,
    "target_wrench_torque_x":  0.0,
    "target_wrench_torque_y":  0.0,
    "target_wrench_torque_z":  0.0,
    # external high-rate target_wrench input.  When non-empty, the
    # orchestrator subscribes BEST_EFFORT and forwards each incoming
    # WrenchStamped immediately (no rate limiting) to every consumer,
    # after clamping to max_wrench_*.  Intended for teleop devices
    # publishing 50-125 Hz (e.g. SpaceMouse).  If no message arrives
    # for external_target_wrench_timeout_sec, the parameter setpoint
    # above takes over via the heartbeat.  Empty = disabled.
    "external_target_wrench_topic":       "",
    "external_target_wrench_timeout_sec": 0.2,
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
    if not cfg.has("cartesian_control_manager"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'cartesian_control_manager:' section in "
                f"{cfg.config_path})")
    sec = cfg.section("cartesian_control_manager")
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
        f"[cartesian_control_manager] config: {source}; "
        f"controllers={[n for n,_,_ in _FZI_CONTROLLERS]}"))

    # Static parameter map -- the (yaml_package, yaml_relpath) pair is
    # only used by the spawners below (not by the orchestrator node), so
    # we strip those two entries to keep the node's parameter set lean.
    parameters = {
        key: LaunchConfiguration(key)
        for key in _FALLBACKS.keys()
        if key not in ("fzi_controller_yaml_package",
                       "fzi_controller_yaml_relpath")
    }
    # The orchestrator's catalogue (parallel string lists) is fixed at
    # launch time -- we always preload the three FZI controllers below,
    # so we hardcode the matching catalogue here too.
    parameters["available_controllers"] = [n for n, _, _ in _FZI_CONTROLLERS]
    parameters["controller_kinds"] = [k for _, k, _ in _FZI_CONTROLLERS]

    node = Node(
        package="cartesian_control_manager",
        executable="cartesian_control_node",
        name="cartesian_control_manager",
        output="screen",
        emulate_tty=True,
        parameters=[parameters],
    )

    # Pre-load each FZI controller into the live controller_manager
    # (inactive).  Our orchestrator activates the *selected* one on
    # engage via the SwitchController service.  All three share the
    # same YAML config (controller_manager picks the section keyed by
    # the controller's own name).  The YAML lives in a per-robot
    # bringup package; we resolve its absolute path at launch time.
    def _spawn_fzi(context, *_args, **_kwargs):
        yaml_pkg = LaunchConfiguration(
            "fzi_controller_yaml_package").perform(context)
        yaml_relpath = LaunchConfiguration(
            "fzi_controller_yaml_relpath").perform(context)
        try:
            pkg_share = get_package_share_directory(yaml_pkg)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"[cartesian_control_manager] could not resolve FZI YAML "
                f"package '{yaml_pkg}': {type(exc).__name__}: {exc}") from exc
        fzi_yaml = os.path.join(pkg_share, yaml_relpath)
        if not os.path.isfile(fzi_yaml):
            raise RuntimeError(
                f"[cartesian_control_manager] FZI YAML not found at "
                f"{fzi_yaml} (resolved from package={yaml_pkg!r} "
                f"relpath={yaml_relpath!r})")
        return [
            LogInfo(msg=f"[cartesian_control_manager] FZI YAML: {fzi_yaml}"),
            *[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        name,
                        "-c", LaunchConfiguration("controller_manager_ns"),
                        "-p", fzi_yaml,
                        "-t", plugin_type,
                        "--inactive",
                    ],
                    output="screen",
                )
                for name, _kind, plugin_type in _FZI_CONTROLLERS
            ],
        ]

    return LaunchDescription([
        log,
        *args,
        OpaqueFunction(function=_spawn_fzi),
        node,
    ])
