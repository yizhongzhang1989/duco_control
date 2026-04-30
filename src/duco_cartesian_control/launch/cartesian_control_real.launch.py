"""Conservative-limits launch profile for real-hardware bring-up.

This file wraps :file:`cartesian_control.launch.py` and overlays a set
of **conservative** safety limits so that the very first runs on the
real arm cannot accelerate, push, or jerk hard.

Usage::

    ros2 launch duco_cartesian_control cartesian_control_real.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Conservative defaults.  These intentionally throttle the safety knobs
# below the lab-tested fake-hardware values so the first real-hardware
# pushes can't run away.  Override at the CLI when you have confidence.
_CONSERVATIVE_OVERRIDES = {
    "engaged_default":          "false",
    # Cap operator-applied wrench (sensor will trip well below the soft
    # limits of the arm).  100 N gives generous head-room for hand-
    # guidance; reduce if the operator wants the supervisor to fire
    # earlier.
    "max_wrench_force":         "100.0",
    "max_wrench_torque":        "10.0",
    # Refuse engage if the arm is moving (anything above 0.02 rad/s).
    "engage_max_joint_velocity": "0.02",
    # Staleness windows.  Empirically the USB-serial FT driver shows
    # 60-70 ms gaps in the worst case (kernel scheduling), so 0.10 s
    # is too tight and trips spuriously; 0.25 s leaves margin while
    # still catching a genuine stall (~250 ms of silence -> disengage).
    "ft_stale_after":            "0.25",
    "joint_states_stale_after":  "0.25",
}


def generate_launch_description() -> LaunchDescription:
    base = os.path.join(
        get_package_share_directory("duco_cartesian_control"),
        "launch", "cartesian_control.launch.py")

    # Expose the conservative overrides as launch arguments so the
    # operator can still override individual knobs from the CLI without
    # editing this file.
    args = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in _CONSERVATIVE_OVERRIDES.items()
    ]

    overrides = {
        name: LaunchConfiguration(name)
        for name in _CONSERVATIVE_OVERRIDES.keys()
    }

    log = LogInfo(msg=(
        "[duco_cartesian_control] REAL-HARDWARE profile: "
        "conservative limits applied. FZI's cartesian_force_controller "
        "is the hot path; this node is the safety supervisor + engage UX."))

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base),
        launch_arguments=overrides.items(),
    )

    return LaunchDescription([log, *args, include])
