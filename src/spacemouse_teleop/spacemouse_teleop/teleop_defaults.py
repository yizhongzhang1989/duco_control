"""Shared launch defaults for spacemouse_teleop.

Mirrors the ``alicia_teleop`` / ``ikt_pose_commander`` convention: the
authoritative tunables live in the workspace ``config/robot_config.yaml`` under
``spacemouse_teleop:``; the hard-coded values below are the fallback used on a
clean checkout (or when a key is missing). Both launch files import this so the
defaults have a single source of truth.
"""

# Robot-AGNOSTIC fallbacks. base_frame / tip_frame are intentionally empty --
# they are robot-specific and must come from robot_config.yaml or a CLI arg;
# the servo node refuses to start without them.
FALLBACKS = {
    "base_frame": "",
    "tip_frame": "",
    "target_pose_topic": "ikt_pose_commander/target_pose",
    "jog_frame": "base",                # 'base' (base-frame jog) or 'tool'
    "rate_hz": 50.0,
    "linear_scale": [0.15, 0.15, 0.15],
    "angular_scale": [0.6, 0.6, 0.6],
    "deadband_lin": 0.0,
    "deadband_ang": 0.0,
    "max_linear_speed": 0.30,
    "max_angular_speed": 1.0,
    "input_timeout": 0.2,
    "deadman_button": 0,
    "deadman_mode": "none",             # none | hold | toggle
    "button1_index": 1,
    "button1_action": "speed",          # speed | position_only | none
    "speed_scales": [0.5, 1.0, 2.0],
    "enable_commander": True,
    "commander_enable_srv": "ikt_pose_commander/enable",
    "commander_disable_srv": "ikt_pose_commander/disable",
    "input_topic": "spacenav/twist",
    "joy_topic": "spacenav/joy",
    "dashboard_port": "",               # "" = no dashboard; e.g. 8200 to enable
}


def load_defaults():
    """Return ``(defaults, source)``.

    ``defaults`` is :data:`FALLBACKS` with the workspace
    ``config/robot_config.yaml`` ``spacemouse_teleop:`` section merged on top
    (when available). ``source`` is a human-readable string for a launch
    LogInfo so it is obvious where the values came from.
    """
    try:
        from cct_common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return dict(FALLBACKS), "FALLBACK (cct_common import failed: %s)" % exc
    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return dict(FALLBACKS), "FALLBACK (config load failed: %s)" % exc
    if not cfg.has("spacemouse_teleop"):
        return (dict(FALLBACKS),
                "FALLBACK (no 'spacemouse_teleop:' in %s)" % cfg.config_path)
    sec = cfg.section("spacemouse_teleop")
    return ({k: sec.get(k, v) for k, v in FALLBACKS.items()},
            "loaded from %s" % cfg.config_path)
