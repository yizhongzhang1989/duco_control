# spacemouse_teleop

Translate the 3Dconnexion **SpaceMouse** pose into an absolute target pose for
[`ikt_pose_commander`](../../external/inverse_kinematics_toolkit/ikt_pose_commander).
It is the single adapter between two packages that know nothing about each other:

- the `spacemouse` package's `pose_node` is **robot-agnostic** — it publishes an
  absolute, accumulated puck pose on `/spacemouse/curr_pose`
  (`geometry_msgs/PoseStamped`) in its own abstract origin frame;
- `ikt_pose_commander` is **device-agnostic** — it wants an absolute target pose
  on `~/target_pose` expressed in a robot frame.

```text
spacemouse pose_node          spacemouse_teleop                 ikt_pose_commander
/spacemouse/curr_pose ──▶ anchor + compose (TF base<-tip) ──▶ ~/target_pose ──▶ IK ──▶ robot
                              ▲ status: enabled + controlled_frame
```

## How it works

- On **engage** (by default the commander's `enable` rising edge) it captures an
  *anchor*: the robot's current end-effector pose (TF `base_frame` → `tip_frame`)
  and the SpaceMouse's current `curr_pose`.
- For every later `curr_pose` it composes the puck's motion **since the anchor**
  onto the robot anchor and publishes an absolute target:

  ```
  target = robot_anchor  (+)  (curr_pose - sm_anchor)
  ```

  so the first target equals the current EE (**no jump**) and the arm jogs from
  there. Puck motion is treated as a **world/base-frame** increment — match the
  `pose_node` `integration_frame: world` (base-frame jog) setting.

Every output is a **full absolute pose**, so the commander always tracks the
**latest** one and intermediate poses may be dropped (no delta accumulation —
transport delay or dropped messages never corrupt the goal).

## Parameters

Defaults live in `config/robot_config.yaml` under `spacemouse_teleop:`
(loaded via `cct_common`); CLI args override.

| Param | Default | Description |
|-------|---------|-------------|
| `input_pose_topic` | `/spacemouse/curr_pose` | absolute puck pose from `pose_node` |
| `output_pose_topic` | `/ikt_pose_commander/target_pose` | the commander's target |
| `commander_status_topic` | `/ikt_pose_commander/status` | source of `enabled` + `controlled_frame` |
| `base_frame` | `base_link` | robot root: TF anchor base **and** output `frame_id` |
| `tip_frame` | `""` | controlled EE link; empty = take `controlled_frame` from the commander status |
| `follow_commander_enable` | `true` | anchor on the commander's enable edge and feed only while enabled; `false` = anchor once and feed continuously |

## Service

- `~/reanchor` (`std_srvs/Trigger`) — re-capture the anchor onto the current EE
  without disabling (recentre after jogging far).

## Run

```bash
# 1) robot bringup + 2) ikt_pose_commander (its own terminals)
# 3) SpaceMouse driver + integrator (robot-agnostic):
ros2 launch spacemouse spacemouse.launch.py \
    integration_frame:=world max_trans_speed:=0.15 max_rot_speed:=0.6 \
    dashboard_port:=8080
# 4) this bridge:
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py
# then enable the commander; the bridge anchors at the current EE and jogs.
```
