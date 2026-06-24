"""One-command SpaceMouse Cartesian teleop: driver + IK commander + bridge.

Brings up the full primary path from the plan:

    spacenav driver  ->  spacemouse_servo (twist->PoseStamped)  ->
    ikt_pose_commander (IK + safety gate)  ->  ros2_control  ->  robot

The robot bringup (publishing ``/robot_description`` + ``/joint_states`` and
loading the controllers) must already be running. ``base_frame`` and
``tip_frame`` are REQUIRED; the commander is pinned to ``tip_frame`` so joints
and controllers auto-derive, and the servo is wired to that commander
instance's target topic + enable/disable services.

Examples::

    # Duco mock, streaming (fpc):
    ros2 launch spacemouse_teleop spacemouse_teleop.launch.py \
        base_frame:=base_link tip_frame:=compliance_link command_mode:=fpc

    # Use the FZI cartesian_motion_controller instead of the IK commander:
    ros2 launch spacemouse_teleop spacemouse_teleop.launch.py \
        base_frame:=base_link tip_frame:=tool0 output:=fzi \
        fzi_target_topic:=/cartesian_motion_controller/target_frame

Safety: the commander starts DISABLED; the servo calls its enable service only
while the dead-man button is held and disable on release.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from spacemouse_teleop.teleop_defaults import load_defaults


def _setup(context, *_, **__):
    base_frame = LaunchConfiguration("base_frame").perform(context)
    tip_frame = LaunchConfiguration("tip_frame").perform(context)
    instance = LaunchConfiguration("instance_name").perform(context)
    command_mode = LaunchConfiguration("command_mode").perform(context)
    jog_frame = LaunchConfiguration("jog_frame").perform(context)
    output = LaunchConfiguration("output").perform(context).strip().lower()
    fzi_target = LaunchConfiguration("fzi_target_topic").perform(context)
    launch_driver = LaunchConfiguration("launch_driver").perform(context)
    dashboard_port = LaunchConfiguration("dashboard_port").perform(context)

    if not base_frame or not tip_frame:
        raise RuntimeError(
            "spacemouse_teleop.launch.py requires base_frame:= and tip_frame:=")
    if output not in ("ikt", "fzi"):
        raise RuntimeError("output must be 'ikt' or 'fzi', got %r" % output)

    actions = [LogInfo(msg=(
        "[spacemouse_teleop] base=%s tip=%s output=%s jog_frame=%s"
        % (base_frame, tip_frame, output, jog_frame)))]

    # --- SpaceMouse driver (global hardware; not namespaced) --------------
    sm_pkg = get_package_share_directory("spacemouse")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sm_pkg, "launch", "spacemouse.launch.py")),
        condition=IfCondition(launch_driver),
    ))

    # --- Output sink: IK commander (default) or FZI controller ----------
    if output == "ikt":
        node_ns = "ikt_pose_commander_%s" % instance
        target_topic = "/%s/target_pose" % node_ns
        enable_srv = "/%s/enable" % node_ns
        disable_srv = "/%s/disable" % node_ns
        cmd_pkg = get_package_share_directory("ikt_pose_commander")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cmd_pkg, "launch", "commander.launch.py")),
            launch_arguments={
                "instance_name": instance,
                "controlled_frame": tip_frame,   # pin -> auto-derive joints/ctrls
                "command_mode": command_mode,
                "base_frame": base_frame,
                "start_enabled": "false",         # SAFETY: servo enables on engage
            }.items(),
        ))
        enable_commander = "true"
    else:  # fzi
        target_topic = fzi_target
        enable_srv = ""
        disable_srv = ""
        enable_commander = "false"   # FZI controller has no enable/disable srv
        actions.append(LogInfo(msg=(
            "[spacemouse_teleop] FZI mode: publishing targets to %s; ensure the "
            "cartesian_motion_controller is active (cartesian_control_manager)."
            % fzi_target)))

    # --- The bridge node -------------------------------------------------
    # Servo tunables come from config/robot_config.yaml (spacemouse_teleop);
    # the topic / enable wiring below is computed per output sink + instance.
    d, source = load_defaults()
    actions.append(LogInfo(msg="[spacemouse_teleop] config: " + source))
    actions.append(Node(
        package="spacemouse_teleop",
        executable="servo_node",
        name="spacemouse_servo",
        output="screen",
        parameters=[{
            "base_frame": base_frame,
            "tip_frame": tip_frame,
            "jog_frame": jog_frame,
            "target_pose_topic": target_topic,
            "enable_commander": enable_commander,
            "commander_enable_srv": enable_srv,
            "commander_disable_srv": disable_srv,
            "input_topic": d["input_topic"],
            "joy_topic": d["joy_topic"],
            "rate_hz": d["rate_hz"],
            "linear_scale": d["linear_scale"],
            "angular_scale": d["angular_scale"],
            "deadband_lin": d["deadband_lin"],
            "deadband_ang": d["deadband_ang"],
            "max_linear_speed": d["max_linear_speed"],
            "max_angular_speed": d["max_angular_speed"],
            "input_timeout": d["input_timeout"],
            "deadman_button": d["deadman_button"],
            "deadman_mode": d["deadman_mode"],
            "button1_index": d["button1_index"],
            "button1_action": d["button1_action"],
            "speed_scales": d["speed_scales"],
        }],
    ))
    if dashboard_port:
        actions.append(Node(
            package="spacemouse_teleop",
            executable="dashboard_node",
            name="spacemouse_servo_dashboard",
            output="screen",
            parameters=[{"port": int(dashboard_port),
                         "servo_ns": "/spacemouse_servo"}],
        ))
    return actions


def generate_launch_description():
    d, _ = load_defaults()
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value=str(d["base_frame"]),
                              description="TF base frame for targets (REQUIRED)."),
        DeclareLaunchArgument("tip_frame", default_value=str(d["tip_frame"]),
                              description="End-effector link to jog (REQUIRED)."),
        DeclareLaunchArgument("instance_name", default_value="arm",
                              description="ikt_pose_commander instance suffix."),
        DeclareLaunchArgument("command_mode", default_value="fpc",
                              description="ikt commander mode: jtc | fpc."),
        DeclareLaunchArgument("jog_frame", default_value=str(d["jog_frame"]),
                              description="'base' (base-frame jog) or 'tool'."),
        DeclareLaunchArgument("output", default_value="ikt",
                              description="Target sink: 'ikt' or 'fzi'."),
        DeclareLaunchArgument("fzi_target_topic",
                              default_value="/cartesian_motion_controller/target_frame",
                              description="PoseStamped sink when output:=fzi."),
        DeclareLaunchArgument("launch_driver", default_value="true",
                              description="Also start the spacenav driver."),
        DeclareLaunchArgument("dashboard_port", default_value="",
                              description="If set (e.g. 8200), also launch the on/off dashboard."),
        OpaqueFunction(function=_setup),
    ])
