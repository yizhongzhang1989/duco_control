# spacemouse_teleop

Translate the 3Dconnexion **SpaceMouse** pose into a unified command for
[`ikt_pose_commander`](../../external/inverse_kinematics_toolkit/ikt_pose_commander).
It is the single adapter between two packages that know nothing about each other:

- the `spacemouse` package's `pose_node` is **robot-agnostic** — it publishes an
  absolute, accumulated puck pose on `/spacemouse/curr_pose` and accepts a reset
  on `/spacemouse/set_pose` (it owns the pose accumulation / anchoring);
- `ikt_pose_commander` is **device-agnostic** — it wants an
  `ikt_interfaces/PoseCommand` (frame_link + control_link + pose) on
  `~/pose_command`.

```text
spacemouse pose_node          spacemouse_teleop                 ikt_pose_commander
/spacemouse/curr_pose ─────▶ translate 1:1 ──────────────▶ ~/pose_command ─▶ IK ─▶ robot
      ▲ /spacemouse/set_pose ◀── EE on enable / link change  (control_link/frame_link)
```

## How it works

The bridge is a **thin translator** — no anchoring math:

- **Forward**: each `/spacemouse/curr_pose` is republished as a `PoseCommand`
  (`pose` = curr_pose, `frame_link` = base, `control_link` = tip) while enabled.
- **Set**: on the commander's **enable** edge or a **control-link change**, the
  bridge publishes the current EE (TF `base_frame` → `tip_frame`) to
  `/spacemouse/set_pose`, so `pose_node` resets `curr_pose` to the EE → the next
  forwarded pose equals the EE (**no jump**). The pose_node does the anchoring.

## Parameters

Defaults live in `config/robot_config.yaml` under `spacemouse_teleop:`
(loaded via `cct_common`); CLI args override.

| Param | Default | Description |
|-------|---------|-------------|
| `input_pose_topic` | `/spacemouse/curr_pose` | absolute puck pose from `pose_node` |
| `output_command_topic` | `/ikt_pose_commander/pose_command` | unified `PoseCommand` to the commander |
| `set_pose_topic` | `/spacemouse/set_pose` | reset `pose_node` to the current EE (re-anchor) |
| `commander_status_topic` | `/ikt_pose_commander/status` | source of `enabled` + `controlled_frame` |
| `base_frame` | `base_link` | robot root: `frame_link` + TF base for set_pose |
| `tip_frame` | `""` | controlled EE link; empty = take `controlled_frame` from the commander status |
| `follow_commander_enable` | `true` | set_pose on enable/link change, feed only while enabled; `false` = continuous |

## Service

- `~/reanchor` (`std_srvs/Trigger`) — push the current EE to `set_pose` so the
  next pose starts at the EE (recentre after jogging far).

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
```
