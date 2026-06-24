# spacemouse_teleop

Cartesian **jog bridge** for the 3Dconnexion SpaceMouse. It turns the
`spacenav` velocity twist into a streaming Cartesian **pose target** so the puck
can drive a robot end-effector in real time.

The SpaceMouse driver (`spacenav`) publishes a 6-DOF *velocity* (`Twist`); the
robot-side pose consumers — [`ikt_pose_commander`](https://github.com/yizhongzhang1989/inverse_kinematics_toolkit)
and the FZI `cartesian_motion_controller` — want a *moving pose target*
(`PoseStamped`). This package is the missing bridge between them.

```
spacenav/twist (Twist)  ┐
spacenav/joy   (Joy)    ┤─ dead-man gate ─ deadband+scale+clamp ─┐
TF base<-tip (capture)  ┘                                        │
                                                                 ▼
                           integrate (tool|base frame)  ──▶  PoseStamped target
                                                                 │
                                  ikt_pose_commander  /  FZI cartesian_motion_controller
```

## Why a separate node

The bridge is **robot-agnostic** (it only emits a generic `PoseStamped` and
reads TF), so it lives next to the SpaceMouse driver and works for any pose
consumer. It never commands the robot directly — actuation and all hard safety
gates stay in `ikt_pose_commander` (reachability / jump / speed / staleness) or
in the FZI controller. The bridge adds only *teleop* safety: dead-man gating,
idle re-capture, input-staleness zeroing, and conservative speed clamps.

## Safety model

* **Dead-man gated.** No target is published unless the dead-man button is held
  (`deadman_mode: hold`) or toggled on (`deadman_mode: toggle`). On release the
  optional downstream commander is disabled.
* **Idle re-capture.** While disengaged, the target is continuously reset to the
  current end-effector pose (via TF), so engaging never produces a jump.
* **Input staleness.** If no twist arrives within `input_timeout`, the puck is
  treated as centred — a disconnected/!frozen SpaceMouse cannot drift the target.
* **Conservative limits.** Per-axis scales plus hard `max_linear_speed` /
  `max_angular_speed` clamps; defaults are intentionally slow.

## Build

```bash
colcon build --symlink-install --packages-select spacemouse_teleop
source install/setup.bash
```

Depends on `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`,
`tf2_ros`, `numpy` (and the `spacenav` driver at runtime).

## Run

### One-command full chain (driver + IK commander + bridge)

A robot bringup must already publish `/robot_description` + `/joint_states` and
load the controllers. Then:

```bash
# Duco mock (streaming fpc), tool-frame jog:
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py \
    base_frame:=base_link tip_frame:=compliance_link command_mode:=fpc
```

This starts the `spacenav` driver, an `ikt_pose_commander` instance pinned to
`tip_frame` (joints + controllers auto-derive), and the bridge wired to that
commander's target topic + enable/disable services. The commander starts
**DISABLED**; hold the dead-man button to jog.

Use the FZI motion controller instead of the IK commander:

```bash
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py \
    base_frame:=base_link tip_frame:=tool0 output:=fzi \
    fzi_target_topic:=/cartesian_motion_controller/target_frame
# (bring up the FZI controller via cartesian_control_manager separately)
```

### Bridge node only

```bash
ros2 launch spacemouse_teleop spacemouse_servo.launch.py \
    base_frame:=base_link tip_frame:=tool0 \
    target_pose_topic:=/ikt_pose_commander_arm/target_pose launch_driver:=true
```

## ROS interface

**Subscribes**
| Topic | Type | Purpose |
|---|---|---|
| `spacenav/twist` | `geometry_msgs/Twist` | puck 6-DOF velocity |
| `spacenav/joy` | `sensor_msgs/Joy` | buttons (dead-man, mode/speed) |
| TF `base_frame`←`tip_frame` | — | capture the live EE pose |

**Publishes**
| Topic | Type | Purpose |
|---|---|---|
| `~/target_pose` (remap to the sink) | `geometry_msgs/PoseStamped` | integrated jog target |
| `~/status` | `std_msgs/String` (JSON) | engaged?, jog_frame, speed_scale, position_only, has_target, twist_fresh |

**Service clients (optional)** — `commander_enable_srv` / `commander_disable_srv`
(`std_srvs/Trigger`), called on engage / release when `enable_commander:=true`.

## Button mapping (default 2-button puck)

| Input | Action |
|---|---|
| Button `deadman_button` (0) | **Dead-man**: hold-to-move (`deadman_mode: hold`) or toggle |
| Button `button1_index` (1) | `button1_action`: cycle **speed** scale, or toggle **position_only**, or none |
| no twist for `input_timeout` | puck treated as centred (zero velocity) |

## Parameters

Defaults live in the `spacemouse_teleop:` section of
[`config/robot_config.yaml`](../../config/robot_config.yaml) (like the other
packages); package fallbacks are in `spacemouse_teleop/teleop_defaults.py`.
CLI args still override. Most-used:

| Param | Default | Notes |
|---|---|---|
| `base_frame` / `tip_frame` | `base_link` / `compliance_link` | **REQUIRED** TF frames (node refuses to start without them) |
| `rate_hz` | 50 | integration / publish rate |
| `linear_scale` / `angular_scale` | `[0.15…]` / `[0.6…]` | per-axis m/s, rad/s per unit twist |
| `max_linear_speed` / `max_angular_speed` | 0.30 / 1.0 | hard clamps (m/s, rad/s) |
| `jog_frame` | `base` | `base` (base-frame jog) or `tool` |
| `deadman_button` / `deadman_mode` | 0 / `none` | dead-man config (`none` = always on) |
| `speed_scales` | `[0.5,1,2]` | button-1 speed cycle multipliers (starts at 1.0×) |
| `input_timeout` | 0.2 | s — twist staleness → zero |
| `enable_commander` | true | drive commander enable/disable from engage |
| `dashboard_port` | "" | set e.g. 8200 for the on/off web dashboard |

> **Tip (from the driver README):** align the SpaceMouse axes with the
> end-effector and use `jog_frame: tool` for the most intuitive control —
> pushing the puck forward moves the tool forward.

## Tests

The twist→pose integrator is ROS-free and unit-tested offline:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=. python3 -m pytest test/test_twist_integrator.py -q
```

## License

MIT
