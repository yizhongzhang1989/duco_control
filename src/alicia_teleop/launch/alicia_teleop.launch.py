"""Launch the Alicia-D → Duco GCR5_910 teleop bridge.

Brings up, in one command:
  * the Alicia-D leader driver (``alicia_duo_leader_driver/serial_server.launch.py``)
  * the Alicia-D leader dashboard (``alicia_duo_leader_dashboard/dashboard.launch.py``)
  * the teleop bridge node (this package)

The driver and dashboard pieces can each be disabled with
``launch_driver:=false`` / ``launch_dashboard:=false`` if they are
already running in another terminal.

Teleop defaults are loaded from ``config/robot_config.yaml`` under
``alicia_teleop`` (via the ``cct_common`` package); CLI overrides win.

Example:
  ros2 launch alicia_teleop alicia_teleop.launch.py
  ros2 launch alicia_teleop alicia_teleop.launch.py rate:=100.0
  ros2 launch alicia_teleop alicia_teleop.launch.py launch_driver:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Hard-coded fallbacks if the central config is missing or doesn't
# contain a particular key. These match the historical defaults so the
# launcher still works on a clean checkout.
_FALLBACKS = {
    "rate": 100.0,
    "trajectory_time": 0.05,
    "velocity_feedforward": True,
    "max_velocity": 3.0,
    "max_acceleration": 10.0,
    "leader_topic": "/arm_joint_state",
    "trajectory_topic": "/arm_1_controller/joint_trajectory",
    "forward_position_topic": "/forward_position_controller/commands",
    # 'forward_position' (DEFAULT, FZI-style direct position streaming
    # via forward_command_controller) or 'trajectory' (publish
    # JointTrajectory to JTC). The teleop bridge auto-switches the
    # underlying ros2_control controller to match this mode at startup.
    "command_mode": "forward_position",
    # Auto-activate the controller required for `command_mode` and
    # deactivate its sibling on startup; restore on shutdown. Disable
    # if you are managing controller activation externally.
    "auto_switch_controller": True,
    "joint_names": [
        "arm_1_joint_1",
        "arm_1_joint_2",
        "arm_1_joint_3",
        "arm_1_joint_4",
        "arm_1_joint_5",
        "arm_1_joint_6",
    ],
    "joint_scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "joint_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    # leader hardware (passed through to alicia_duo_leader_driver)
    "leader_port": "",                 # empty = driver auto-detects
    "leader_baudrate": 1000000,
    "leader_query_rate": 200.0,
    "leader_debug_mode": False,
    # leader dashboard (passed through to alicia_duo_leader_dashboard).
    # 8090 collides with duco_dashboard's default -- use a free slot.
    "dashboard_port": 8130,
}


def _defaults():
    """Return (defaults_dict, source_str).

    ``source_str`` describes where the values came from; never silently
    swallowed so the LogInfo line at launch time makes it obvious.
    """
    try:
        from cct_common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not import cct_common.config_manager: "
                f"{type(exc).__name__}: {exc})")
    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not load config: "
                f"{type(exc).__name__}: {exc})")
    sec = cfg.section("alicia_teleop") if cfg.has("alicia_teleop") else None
    if sec is None:
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'alicia_teleop:' section in {cfg.config_path})")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()
    # Mirrors ur15_crisp's pattern: resolve the leader joint calibration
    # via the shared workspace helper so it always points at the project's
    # config/joint_config.yaml regardless of where the launch is invoked
    # from. Empty string is a safe fallback -- the upstream alicia driver
    # falls back to its packaged template in that case.
    try:
        from cct_common.workspace_utils import get_config_path  # type: ignore
        joint_config_default = get_config_path("joint_config.yaml")
    except Exception:
        joint_config_default = ""

    args = [
        # teleop bridge
        DeclareLaunchArgument("rate", default_value=str(d["rate"])),
        DeclareLaunchArgument("trajectory_time", default_value=str(d["trajectory_time"])),
        DeclareLaunchArgument("velocity_feedforward",
                              default_value=str(d["velocity_feedforward"]).lower(),
                              description="Send leader-velocity estimates with each trajectory point"),
        DeclareLaunchArgument("max_velocity", default_value=str(d["max_velocity"]),
                              description="Per-joint velocity cap for the "
                                          "rate-limited command interpolator "
                                          "(also used as velocity feedforward "
                                          "value), rad/s"),
        DeclareLaunchArgument("max_acceleration",
                              default_value=str(d["max_acceleration"]),
                              description="Per-joint acceleration cap for the "
                                          "rate-limited command interpolator, "
                                          "rad/s^2. Together with max_velocity "
                                          "this controls how quickly the "
                                          "follower closes a leader/follower "
                                          "gap on SYNC engage."),
        DeclareLaunchArgument("leader_topic", default_value=str(d["leader_topic"])),
        DeclareLaunchArgument("trajectory_topic", default_value=str(d["trajectory_topic"])),
        DeclareLaunchArgument(
            "forward_position_topic",
            default_value=str(d["forward_position_topic"]),
            description="Topic for std_msgs/Float64MultiArray when "
                        "command_mode == 'forward_position'"),
        DeclareLaunchArgument(
            "command_mode",
            default_value=str(d["command_mode"]),
            choices=["trajectory", "forward_position"],
            description="forward_position (default) = publish "
                        "Float64MultiArray to forward_command_controller "
                        "(FZI-style direct position streaming, no JTC "
                        "spline); trajectory = publish JointTrajectory to "
                        "JTC"),
        DeclareLaunchArgument(
            "auto_switch_controller",
            default_value=str(d["auto_switch_controller"]).lower(),
            choices=["true", "false", "True", "False"],
            description="Auto-activate the controller required for "
                        "command_mode on startup and restore on shutdown"),
        DeclareLaunchArgument("namespace", default_value=""),
        # composition switches
        DeclareLaunchArgument(
            "launch_driver", default_value="true",
            description="Also start alicia_duo_leader_driver/serial_server.launch.py"),
        DeclareLaunchArgument(
            "launch_dashboard", default_value="true",
            description="Also start alicia_duo_leader_dashboard/dashboard.launch.py"),
        # leader driver pass-throughs
        DeclareLaunchArgument(
            "leader_port", default_value=str(d["leader_port"]),
            description="Serial port for the Alicia leader (empty = auto-detect)"),
        DeclareLaunchArgument(
            "leader_baudrate", default_value=str(d["leader_baudrate"])),
        DeclareLaunchArgument(
            "leader_query_rate", default_value=str(d["leader_query_rate"])),
        DeclareLaunchArgument(
            "leader_debug_mode",
            default_value=str(d["leader_debug_mode"]).lower()),
        DeclareLaunchArgument(
            "joint_config", default_value=joint_config_default,
            description="Path to Alicia leader joint_config.yaml "
                        "(empty = upstream packaged template)"),
        # leader dashboard pass-throughs
        DeclareLaunchArgument(
            "dashboard_port", default_value=str(d["dashboard_port"]),
            description="HTTP port for the Alicia leader dashboard"),
    ]

    log = LogInfo(msg=(
        f"[alicia_teleop] config: {source}; "
        f"rate={d['rate']} traj_time={d['trajectory_time']} "
        f"leader_topic={d['leader_topic']} "
        f"trajectory_topic={d['trajectory_topic']} "
        f"joint_names={d['joint_names']} "
        f"joint_config={joint_config_default or '(upstream default)'} "
        f"dashboard_port={d['dashboard_port']}"))

    # --- Alicia leader driver (optional) -----------------------------------
    driver_launch_file = os.path.join(
        get_package_share_directory("alicia_duo_leader_driver"),
        "launch", "serial_server.launch.py")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch_file),
        launch_arguments={
            "debug_mode": LaunchConfiguration("leader_debug_mode"),
            "port": LaunchConfiguration("leader_port"),
            "baudrate": LaunchConfiguration("leader_baudrate"),
            "query_rate": LaunchConfiguration("leader_query_rate"),
            "joint_config": LaunchConfiguration("joint_config"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    # --- Alicia leader dashboard (optional) --------------------------------
    dashboard_launch_file = os.path.join(
        get_package_share_directory("alicia_duo_leader_dashboard"),
        "launch", "dashboard.launch.py")
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dashboard_launch_file),
        launch_arguments={
            "web_port": LaunchConfiguration("dashboard_port"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_dashboard")),
    )

    # --- teleop bridge ------------------------------------------------------
    teleop_node = Node(
        package="alicia_teleop",
        executable="teleop_node",
        name="alicia_teleop",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "rate": LaunchConfiguration("rate"),
            "trajectory_time": LaunchConfiguration("trajectory_time"),
            "velocity_feedforward": LaunchConfiguration("velocity_feedforward"),
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "leader_topic": LaunchConfiguration("leader_topic"),
            "trajectory_topic": LaunchConfiguration("trajectory_topic"),
            "forward_position_topic": LaunchConfiguration("forward_position_topic"),
            "command_mode": LaunchConfiguration("command_mode"),
            "auto_switch_controller": LaunchConfiguration("auto_switch_controller"),
            "joint_names": d["joint_names"],
            "joint_scale": [float(x) for x in d["joint_scale"]],
            "joint_offset": [float(x) for x in d["joint_offset"]],
        }],
    )

    return LaunchDescription([log, *args, driver, dashboard, teleop_node])
