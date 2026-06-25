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

* **Engagement gate.** With `deadman_mode: none` (default) the puck is always
  engaged &mdash; touch to move, centre to hold. Set `hold` (publish only while
  the dead-man button is held) or `toggle` (press to engage / disengage) for a
  button-gated workflow. On disengage the optional downstream commander is
  disabled. The dashboard's **On/Off** (`~/set_enabled`) gates the whole sender.
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

Defaults (`base_frame`, `tip_frame`, `jog_frame`, speeds, `dashboard_port`, …)
come from the `spacemouse_teleop:` section of
[`config/robot_config.yaml`](../../config/robot_config.yaml); every value below is
a CLI override. A robot bringup must already publish `/robot_description` +
`/joint_states` and load the controllers.

### Recommended: standalone pose commander + dashboards

The full SpaceMouse teleop stack on the real Duco (each line in its own terminal):

```bash
# 1. Robot bringup (real hardware).
ros2 launch robot_bringup duco_bringup.launch.py use_fake_hardware:=false

# 2. Pose commander, pinned to the tip link, web dashboard on :8180.
ros2 launch ikt_pose_commander commander.launch.py \
    dashboard_port:=8180 controlled_frame:=compliance_link command_mode:=fpc

# 3. SpaceMouse driver (shared hardware; launched on its own).
ros2 launch spacemouse spacemouse.launch.py

# 4. Jog bridge + On/Off dashboard on :8200
#    (base_frame/tip_frame/jog_frame default from robot_config.yaml).
ros2 launch spacemouse_teleop spacemouse_servo.launch.py dashboard_port:=8200
```

* **Pose-commander dashboard** &rarr; <http://localhost:8180/> (3D gizmo,
  Read/Send mode, live params).
* **SpaceMouse On/Off dashboard** &rarr; <http://localhost:8200/> &mdash; switch
  the sender **On** (jog with the puck) / **Off** (release the commander so you
  can drive from the :8180 dashboard instead). The two share one target topic, so
  use one source at a time. It also shows the **live device status** &mdash; axis
  bars, a 3D preview cube, and button indicators (from `spacenav/twist` +
  `spacenav/joy`).

With `deadman_mode: none` (the default) the bridge auto-enables the commander and
jogs as soon as you touch the puck &mdash; centre it to hold.

### One-command commander + bridge (its own commander instance)

When you don't need a separate commander/dashboard, this starts a private
`ikt_pose_commander` instance pinned to `tip_frame` (joints + controllers
auto-derive) and the bridge wired to it. Launch the `spacenav` driver
separately first (`ros2 launch spacemouse spacemouse.launch.py`):

```bash
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py \
    command_mode:=fpc dashboard_port:=8200
```

Jog by **deltas** instead of absolute targets (the commander owns the goal and is
seeded from the current pose on engage; no TF capture on the servo side):

```bash
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py \
    command_mode:=fpc output_mode:=delta dashboard_port:=8200
# sets the servo's output_mode AND the commander's target_mode=delta +
# delta_frame=jog_frame in one switch (requires output:=ikt).
```

Use the FZI motion controller instead of the IK commander:

```bash
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py \
    tip_frame:=tool0 output:=fzi \
    fzi_target_topic:=/cartesian_motion_controller/target_frame
# (bring up the FZI controller via cartesian_control_manager separately)
```

### Bridge node only

The servo never starts the driver &mdash; launch the `spacenav` driver
separately (`ros2 launch spacemouse spacemouse.launch.py`). The On/Off dashboard
comes up only when `dashboard_port` is non-empty (default `8200` from
`robot_config.yaml`; set it to `""` there to run headless):

```bash
ros2 launch spacemouse_teleop spacemouse_servo.launch.py
```

## ROS interface

**Subscribes**
| Topic | Type | Purpose |
|---|---|---|
| `spacenav/twist` | `geometry_msgs/Twist` | puck 6-DOF velocity |
| `spacenav/joy` | `sensor_msgs/Joy` | buttons (dead-man, mode/speed) |
| TF `base_frame`←`tip_frame` | — | capture the live EE pose (`output_mode: absolute` only) |

**Publishes**
| Topic | Type | Purpose |
|---|---|---|
| `~/target_pose` (remap to the sink) | `geometry_msgs/PoseStamped` | integrated **absolute** jog target (`output_mode: absolute`) |
| `~/target_delta` (→ commander `~/target_delta`) | `geometry_msgs/PoseStamped` | per-tick **incremental** pose (`output_mode: delta`) |
| `~/status` | `std_msgs/String` (JSON) | sender_enabled, engaged?, output_mode, jog_frame, speed_scale, position_only, has_target, twist_fresh |

**Service servers** — `~/set_enabled` (`std_srvs/SetBool`): master On/Off for the
sender (used by the On/Off dashboard, `dashboard_port:=8200`). Off stops
publishing + disables the commander; On re-captures the live pose + re-enables.

**Service clients (optional)** — `commander_enable_srv` / `commander_disable_srv`
(`std_srvs/Trigger`), called on engage / release when `enable_commander:=true`;
`commander_snap_srv` (`std_srvs/Trigger`), called on engage in `output_mode:
delta` to seed the commander's goal onto the current pose.

### Absolute vs delta output

* **`output_mode: absolute`** (default) — the servo captures the live EE pose over
  TF and **integrates** the puck twist into an **absolute** `PoseStamped` on
  `target_pose_topic`. The commander runs in `target_mode: absolute`.
* **`output_mode: delta`** — the servo streams **per-tick incremental** poses on
  `target_delta_topic`; the commander (`target_mode: delta`) composes them onto
  its own goal, which it seeds from the current pose via `~/snap_target` on
  engage. No TF capture is needed on this side. Centre the puck to hold (identity
  deltas), release to let another source (e.g. the dashboard) take over.
  `jog_frame` must match the commander's `delta_frame` (both default `base`).
  `spacemouse_teleop.launch.py output_mode:=delta` wires this end-to-end (it also
  sets the commander's `target_mode` + `delta_frame`).

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
| `output_mode` | `absolute` | `absolute` (integrate → `target_pose_topic`) or `delta` (stream increments → `target_delta_topic`) |
| `target_delta_topic` | `ikt_pose_commander/target_delta` | where deltas go in `output_mode: delta` |
| `rate_hz` | 50 | integration / publish rate |
| `linear_scale` / `angular_scale` | `[0.15…]` / `[0.6…]` | per-axis m/s, rad/s per unit twist |
| `max_linear_speed` / `max_angular_speed` | 0.30 / 1.0 | hard clamps (m/s, rad/s) |
| `jog_frame` | `base` | `base` (base-frame jog) or `tool` (match the commander's `delta_frame`) |
| `deadman_button` / `deadman_mode` | 0 / `none` | dead-man config (`none` = always on) |
| `speed_scales` | `[0.5,1,2]` | button-1 speed cycle multipliers (starts at 1.0×) |
| `input_timeout` | 0.2 | s — twist staleness → zero |
| `enable_commander` | true | drive commander enable/disable from engage |
| `commander_snap_srv` | `ikt_pose_commander/snap_target` | seeds the goal on engage (`output_mode: delta`) |
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
