# spacemouse_teleop

Translate the 3Dconnexion **SpaceMouse** pose into a unified command for
[`ikt_pose_commander`](../../external/inverse_kinematics_toolkit/ikt_pose_commander).
It is the single adapter between two packages that know nothing about each other:

- the `spacemouse` package's `pose_node` is **robot-agnostic** — it publishes an
  absolute, accumulated puck pose on `/spacemouse/curr_pose` and accepts a reset
  on `/spacemouse/set_pose` (it owns the pose accumulation / anchoring);
- `ikt_pose_commander` is **device-agnostic** — it wants an
  `ikt_interfaces/PoseCommand` (`frame_link` + `control_link` + `pose`) on its
  `pose_command` topic, solves IK in-process, and streams joints to the robot.

This package keeps both sides decoupled: the SpaceMouse never learns a robot
link name, and the commander never learns a device exists.

```text
spacemouse pose_node          spacemouse_teleop                 ikt_pose_commander
/spacemouse/curr_pose ─────▶ translate 1:1 ──────────────▶ pose_command ─▶ IK ─▶ robot
      ▲ /spacemouse/set_pose ◀── EE on enable / link change      ◀── status (enabled, link)
```

## How it works

The bridge is a **thin translator** — it carries no anchoring math of its own:

- **Forward** — each `/spacemouse/curr_pose` is republished verbatim as a
  `PoseCommand` with `has_pose=true` and `pose = curr_pose`. `frame_link` and
  `control_link` are left **empty**, so the commander reuses whatever the config
  or dashboard already set (base root and the controlled tip). The bridge thus
  never fights the dashboard for the control link.
- **Anchor (no jump)** — on the commander's **enable** rising edge or a
  **control-link change**, the bridge reads the robot's current EE (TF
  `base_frame → tip_frame`) and publishes it to `/spacemouse/set_pose`. The
  `pose_node` resets its `curr_pose` to that EE, so the next forwarded pose
  equals where the arm already is. Anchoring lives in the `pose_node`; the bridge
  only triggers it.

Because every message is a **full absolute pose**, the commander always tracks
the latest one and intermediate poses can be dropped freely — transport delay or
a lost message never corrupts the goal.

## Topics & service

| Direction | Name (default) | Type | Purpose |
|-----------|----------------|------|---------|
| sub | `/spacemouse/curr_pose` | `geometry_msgs/PoseStamped` | absolute puck pose to translate |
| sub | `/ikt_pose_commander/status` | `std_msgs/String` (JSON) | `enabled` + `controlled_frame` |
| pub | `/ikt_pose_commander/pose_command` | `ikt_interfaces/PoseCommand` | target pose for the commander |
| pub | `/spacemouse/set_pose` | `geometry_msgs/PoseStamped` | re-anchor `pose_node` to the EE |
| srv | `~/reanchor` | `std_srvs/Trigger` | push current EE to `set_pose` (recentre) |

TF: looks up `base_frame → tip_frame` to capture the EE for anchoring.

## Parameters

Defaults live in `config/robot_config.yaml` under `spacemouse_teleop:`
(loaded via `cct_common`); CLI args override.

| Param | Default | Description |
|-------|---------|-------------|
| `input_pose_topic` | `/spacemouse/curr_pose` | absolute puck pose from `pose_node` |
| `output_command_topic` | `/ikt_pose_commander/pose_command` | unified `PoseCommand` to the commander |
| `set_pose_topic` | `/spacemouse/set_pose` | reset `pose_node` to the current EE (re-anchor) |
| `commander_status_topic` | `/ikt_pose_commander/status` | source of `enabled` + `controlled_frame` |
| `base_frame` | `base_link` | robot root: TF base for set_pose / EE lookup |
| `tip_frame` | `""` | controlled EE link; empty = take `controlled_frame` from commander status |
| `follow_commander_enable` | `true` | set_pose on enable/link change, feed only while enabled; `false` = continuous |

### Modes

- `follow_commander_enable: true` (default) — anchor on the enable edge, feed
  targets only while the commander is enabled. Disabling the commander stops
  motion; re-enabling re-anchors to the EE.
- `follow_commander_enable: false` — anchor once on the first pose and feed
  continuously, ignoring commander status (standalone / no status topic).
- `tip_frame: ""` — follow the commander's controlled link live, so the anchor
  always matches what the commander drives; set a fixed link to pin it.

## Run

```bash
# 1) robot bringup + 2) ikt_pose_commander (its own terminals)
# 3) SpaceMouse driver + integrator (robot-agnostic):
ros2 launch spacemouse spacemouse.launch.py \
    integration_frame:=world max_trans_speed:=0.15 max_rot_speed:=0.6 \
    dashboard_port:=8080
# 4) this bridge:
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py
# then enable the commander; the bridge set_poses to the current EE and jogs.

# recentre after jogging far (curr_pose drifted from the EE):
ros2 service call /spacemouse_teleop/reanchor std_srvs/srv/Trigger
```

## Troubleshooting

- **No motion** — the commander starts DISABLED; call `~/enable` on it (or use
  the dashboard). With `follow_commander_enable:true` the bridge feeds only while
  enabled.
- **Jump on enable** — TF `base_frame → tip_frame` must resolve; if the EE is
  unavailable the anchor is skipped. Check `tip_frame` (or status) and TF.
- **`integration_frame`** — keep the `pose_node` on `world` so puck motion is a
  base-frame increment, matching how the commander applies the target.

## Build & test

```bash
colcon build --packages-select spacemouse_teleop && \
  colcon test --packages-select spacemouse_teleop
```
