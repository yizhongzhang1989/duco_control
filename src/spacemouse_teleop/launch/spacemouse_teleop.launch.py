"""Launch the SpaceMouse pose -> ikt_pose_commander target translator.

Reads defaults from ``config/robot_config.yaml`` (``spacemouse_teleop:``
section) via cct_common; CLI args override. The SpaceMouse driver + integrator
(``ros2 launch spacemouse spacemouse.launch.py``) and ``ikt_pose_commander`` are
launched SEPARATELY -- this node only bridges their topics.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_FALLBACKS = {
    "input_pose_topic": "/spacemouse/curr_pose",
    "output_command_topic": "/ikt_pose_commander/pose_command",
    "set_pose_topic": "/spacemouse/set_pose",
    "commander_status_topic": "/ikt_pose_commander/status",
    "base_frame": "base_link",
    "tip_frame": "",
    "follow_commander_enable": "true",
}


def _defaults():
    try:
        from cct_common.config_manager import get_config  # type: ignore
        cfg = get_config()
        if cfg.has("spacemouse_teleop"):
            sec = cfg.section("spacemouse_teleop")
            return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
                    f"loaded from {cfg.config_path}")
        return (dict(_FALLBACKS),
                "FALLBACK (no 'spacemouse_teleop:' section)")
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS), f"FALLBACK ({type(exc).__name__}: {exc})")


def generate_launch_description():
    d, source = _defaults()
    return LaunchDescription([
        DeclareLaunchArgument("input_pose_topic",
                              default_value=str(d["input_pose_topic"])),
        DeclareLaunchArgument("output_command_topic",
                              default_value=str(d["output_command_topic"])),
        DeclareLaunchArgument("set_pose_topic",
                              default_value=str(d["set_pose_topic"])),
        DeclareLaunchArgument("commander_status_topic",
                              default_value=str(d["commander_status_topic"])),
        DeclareLaunchArgument("base_frame",
                              default_value=str(d["base_frame"])),
        DeclareLaunchArgument("tip_frame",
                              default_value=str(d["tip_frame"])),
        DeclareLaunchArgument("follow_commander_enable",
                              default_value=str(d["follow_commander_enable"])),
        LogInfo(msg=f"[spacemouse_teleop] config: {source}"),
        Node(
            package="spacemouse_teleop",
            executable="bridge_node",
            name="spacemouse_teleop",
            output="screen",
            parameters=[{
                "input_pose_topic":
                    LaunchConfiguration("input_pose_topic"),
                "output_command_topic":
                    LaunchConfiguration("output_command_topic"),
                "set_pose_topic":
                    LaunchConfiguration("set_pose_topic"),
                "commander_status_topic":
                    LaunchConfiguration("commander_status_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "tip_frame": LaunchConfiguration("tip_frame"),
                "follow_commander_enable": ParameterValue(
                    LaunchConfiguration("follow_commander_enable"),
                    value_type=bool),
            }],
        ),
    ])
