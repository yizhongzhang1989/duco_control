# duco_control

ROS 2 (Humble) workspace for hand-guided / Cartesian-compliance control of
the Duco GCR5_910 6-DoF arm.  The hot-path Cartesian controllers are the
C++ `cartesian_force_controller` / `cartesian_motion_controller` /
`cartesian_compliance_controller` plugins from a fork of FZI's
[cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)
(tracked as a submodule under
[`external/cartesian_controllers`](external/cartesian_controllers)).
Everything in `src/` is the project-owned glue around them: the FT sensor
driver, gravity compensation, the orchestrator that spawns / engages /
supervises the active Cartesian controller, and a set of optional web
dashboards.

---

## Packages

| package | purpose | runtime port |
|---|---|---|
| [`robot_bringup`](src/robot_bringup) | top-level "start all" meta-bringup: one launch per robot (`duco_bringup.launch.py`, `ur15_bringup.launch.py`) that starts the **complete** stack -- robot bringup, F/T + gravity compensation, the cartesian orchestrator, and the web dashboards -- with a single command. Pins `ROBOT_CONFIG_PATH` to the robot's config automatically. See [One-command bringup](#one-command-bringup). | -- |
| [`duco_robot_bringup`](src/duco_robot_bringup) | project-owned launch wrapper around the upstream `duco_*` driver (URDF + `controller_manager` + JTC); also ships the per-robot `config/fzi_preset.yaml` consumed by `cartesian_control_manager` | -- |
| [`ur15_robot_bringup`](src/ur15_robot_bringup) | sibling bringup wrapper around the apt-installed `ur_robot_driver` for the UR15 (Universal Robots) arm; also ships its own `config/fzi_preset.yaml`. Selected via `ROBOT_CONFIG_PATH=$PWD/config/robot_config.ur15.yaml`. See [Multi-robot support](#multi-robot-support). | -- |
| [`duco_ft_sensor`](src/duco_ft_sensor) | serial driver + ROS publisher for the Duco F/T sensor | -- |
| [`ft_sensor_dashboard`](external/cartesian_controllers_toolkit/ft_sensor_dashboard) | optional web UI for any `WrenchStamped` topic *(from the toolkit submodule)* | `8080` |
| [`ft_sensor_gravity_compensation`](external/cartesian_controllers_toolkit/ft_sensor_gravity_compensation) | subscribes to the raw wrench + `/tf`, publishes a gravity-compensated wrench, has its own calibration UI *(from the toolkit submodule)* | `8100` |
| [`aux_frame_manager`](external/cartesian_controllers_toolkit/aux_frame_manager) | single writer of the canonical augmented `robot_description`: appends the configured aux frames (`ft_sensor_link`, `compliance_link`) and serves them on the latched topic `/cartesian/robot_description` that the FZI controllers read; mirrors to TF via `robot_state_publisher`; optional 3D web view + live frame editor *(from the toolkit submodule)* | `8160` |
| [`cartesian_control_manager`](external/cartesian_controllers_toolkit/cartesian_control_manager) | spawns FZI's `cartesian_force_controller` / `cartesian_motion_controller` / `cartesian_compliance_controller` (all inactive), relays the wrench, optionally publishes a zero target_wrench heartbeat, runs the engage / disengage Trigger services and a safety supervisor *(from the toolkit submodule)* | -- |
| [`cartesian_controller_dashboard`](external/cartesian_controllers_toolkit/cartesian_controller_dashboard) | optional web UI for engage / disengage, controller selection (force / motion / compliance), and live-tuning of the active controller's gains *(from the toolkit submodule)* | `8120` |
| [`duco_dashboard`](src/duco_dashboard) | optional web UI for joint / controller / TCP state | `8090` |
| [`alicia_teleop`](src/alicia_teleop) | leader-follower teleop bridge: maps Alicia-D joint angles to the Duco follower with a per-joint velocity- and acceleration-limited interpolator; auto-switches `arm_1_controller` ↔ `forward_position_controller` at launch / shutdown | -- |
| [`alicia_duo_leader_driver`](src/alicia_leader/alicia_duo_leader_driver) | serial driver for the Alicia-D 6-DoF leader arm (publishes `/arm_joint_state`) | -- |
| [`alicia_duo_leader_dashboard`](src/alicia_leader/alicia_duo_leader_dashboard) | optional web UI for the leader arm (button / joint state) | `8130` |
| [`spacemouse_teleop`](src/spacemouse_teleop) | SpaceMouse Cartesian jog bridge: integrates the `spacenav` velocity twist into a streaming `PoseStamped` target (captured from TF) that drives `ikt_pose_commander`; ships an On/Off web dashboard. See [SpaceMouse teleoperation](#teleoperation-spacemouse-cartesian-jog). | `8200` |
| [`cct_common`](external/cartesian_controllers_toolkit/cct_common) | centralised config loader (reads `config/robot_config.yaml`) + shared URDF/XML helpers *(from the toolkit submodule)* |

The official Duco ROS 2 driver is tracked as a git submodule at
`external/duco_ros2_driver`.  Its packages provide
`duco_gcr5_910_moveit_config`, `duco_support`, `duco_hardware`,
`duco_ros_driver`, etc.  Keep upstream driver changes in the submodule.

---

## Configuration

All runtime defaults live in [`config/robot_config.yaml`](config/robot_config.yaml),
keyed by package name.  On a fresh checkout copy the template:

```bash
cp config/robot_config.example.yaml config/robot_config.yaml
```

Key knobs:

* `duco_robot_bringup.use_fake_hardware` -- `true` by default; flip to
  `false` only when the controller IP is reachable.
* `duco_robot_bringup.robot_ip`, `robot_port` -- the Duco controller endpoint.
* `duco_ft_sensor.port`, `baud` -- serial device for the F/T sensor.
* `cartesian_control_manager.max_wrench_force`, `max_wrench_torque`,
  `engage_max_joint_velocity` -- safety supervisor trip thresholds.
* `aux_frame_manager.aux_frames` -- list of fixed-joint TF frames
  appended to the URDF (default chain
  `link_6 -> ft_sensor_link -> compliance_link`). `ft_sensor_link` is
  the gravity-compensation sensor frame; `compliance_link` is the FZI
  `end_effector_link`. `aux_frame_manager` reads this list from its
  **own** config section and is the sole owner of the frames (the
  bringup publishes the bare URDF with `apply_aux_frames:=false`); edit
  xyz/rpy by hand or via the cartesian dashboard's "Tool frames" panel
  and the change is applied **live** (the manager republishes the
  canonical URDF and the engaged controller swaps its chain -- no
  relaunch needed).

Anything declared in a launch file's `_FALLBACKS` block can be overridden
on the CLI as well, e.g. `port:=9120`.

---

## Build

```bash
git submodule update --init --recursive
colcon build --symlink-install
source install/setup.bash
```

Repeat `colcon build --symlink-install` after pulling.  Because all the
project-owned packages use `--symlink-install`, edits to Python sources
take effect on the next launch without rebuilding.

> Two packages from the upstream FZI submodule
> (`cartesian_controller_simulation`, `cartesian_controller_tests`) pull in
> Gazebo-classic dependencies we don't need; they are excluded via
> [`colcon_defaults.yaml`](colcon_defaults.yaml) at the workspace root.
> The `colcon-defaults` plugin auto-discovers that file when `colcon` is
> invoked from this directory, so no extra flags or env vars are required.
> The submodule's working tree stays clean (`git status` shows nothing).

---

## One-command bringup

The [`robot_bringup`](src/robot_bringup) package starts the **entire** stack
for a robot with a single launch -- no need to open six terminals.  Each
per-robot launch pins `ROBOT_CONFIG_PATH` to that robot's config
automatically, brings up the robot + F/T + gravity compensation immediately,
then stages the Cartesian orchestrator and the web dashboards a few seconds
later (so `controller_manager` is up first).

```bash
cd /home/robot/Documents/duco_control
source install/setup.bash

# Duco GCR5-910 -- everything (uses config's use_fake_hardware)
ros2 launch robot_bringup duco_bringup.launch.py

# UR15 -- everything
ros2 launch robot_bringup ur15_bringup.launch.py
```

What each launch starts:

| stage | Duco (`duco_bringup.launch.py`) | UR15 (`ur15_bringup.launch.py`) |
|---|---|---|
| 0 (now) | `duco_robot_bringup` (bare URDF) + `aux_frame_manager` (+ guard, dashboard `8160`) + `duco_ft_sensor` + `ft_sensor_gravity_compensation` | `ur15_robot_bringup` (incl. FT broadcaster) + `ft_sensor_gravity_compensation` |
| 1 (after `cartesian_delay`, default 8 s) | `cartesian_control_manager` (real-HW limits) + `cartesian_controller_dashboard` (`8120`) + `duco_dashboard` (`8090`) | `cartesian_control_manager` (real-HW limits) + `cartesian_controller_dashboard` (`8120`) |

Common overrides (forwarded to the sub-launches):

```bash
# Fake-hardware smoke test (no robot connected)
ros2 launch robot_bringup duco_bringup.launch.py use_fake_hardware:=true
ros2 launch robot_bringup ur15_bringup.launch.py use_fake_hardware:=true robot_ip:=127.0.0.1

# Headless (no web dashboards), and move the gravity-comp UI off 8100
ros2 launch robot_bringup duco_bringup.launch.py \
    ft_dashboard_port:=0 launch_cartesian_dashboard:=false \
    launch_robot_state_dashboard:=false aux_frame_dashboard_port:=''

# Disable the aux-frame 3D dashboard (on by default on :8160)
ros2 launch robot_bringup duco_bringup.launch.py aux_frame_dashboard_port:=''

# Select a robot config explicitly (otherwise the per-robot default is used)
ROBOT_CONFIG_PATH=$PWD/config/robot_config.ur15.yaml \
    ros2 launch robot_bringup ur15_bringup.launch.py
```

After it's up, engage exactly as in the manual flow:

```bash
ros2 service call /cartesian_control_manager/engage std_srvs/srv/Trigger
```

> The sections below document the **manual, terminal-per-step** flow that
> `robot_bringup` automates -- useful for understanding each stage,
> debugging, or running a subset.

## Bringup sequence (real hardware)

The full free-drive / hand-guidance stack is started by running each of
these in its own terminal, in order.  Every terminal needs
`source install/setup.bash` first; `cd /home/robot/Documents/duco_control`
is assumed.

> Before starting: make sure `duco_robot_bringup.use_fake_hardware: false`
> in `config/robot_config.yaml`, the Duco controller is reachable at
> `robot_ip:robot_port`, and the FT sensor is connected at `duco_ft_sensor.port`.

### 1. Robot bringup -- `controller_manager` + JTC

```bash
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py \
    use_rviz:=false apply_aux_frames:=false
```

`apply_aux_frames:=false` makes the bringup publish the **bare**
manufacturer URDF on `/robot_description`; `aux_frame_manager` (step 2)
becomes the sole owner of the `ft_sensor_link` / `compliance_link` aux
frames and serves the augmented URDF on its own topic.  (Omit the arg --
default `true` -- only if running the bringup standalone *without*
`aux_frame_manager`, which bakes the frames straight into the URDF.)

This brings up `controller_manager`, loads the URDF, and activates
`joint_state_broadcaster` and `arm_1_controller` (the
`JointTrajectoryController`).  Verify:

```bash
ros2 control list_controllers
# joint_state_broadcaster   active
# arm_1_controller          active
ros2 topic echo /joint_states --once
```

### 2. Canonical URDF source -- `aux_frame_manager`

```bash
ros2 launch aux_frame_manager cartesian_urdf_source.launch.py \
    end_effector_link:=compliance_link \
    dashboard_port:=8160
```

`aux_frame_manager` is the **single writer** of the augmented URDF.  It
reads the bare `/robot_description`, appends the
`aux_frame_manager.aux_frames` entries (`ft_sensor_link`,
`compliance_link`) from `config/robot_config.yaml` (its **own** config
section -- no selector argument needed), and:

* publishes the canonical URDF on the latched topic
  `/cartesian/robot_description` -- the FZI controllers read their chain
  from here (`urdf_from_topic: true` in
  [`config/fzi_preset.yaml`](src/duco_robot_bringup/config/fzi_preset.yaml)),
  so tool-frame edits swap their chain **live**;
* mirrors it to `robot_state_publisher` so `/tf` (and RViz) get the aux
  frames too;
* runs a **guard** that latches `/cartesian/robot_description_ready`
  once `compliance_link` is present and in the `base_link -> ee` chain,
  and prints an actionable error otherwise.

`dashboard_port:=8160` also serves a 3D view + live frame editor on
<http://localhost:8160/> (omit it to run headless).  Verify:

```bash
ros2 topic echo /cartesian/robot_description --once | head -c 120  # canonical URDF present
ros2 run tf2_ros tf2_echo base_link compliance_link               # aux frame in TF
```

### 3. F/T sensor driver

```bash
ros2 launch duco_ft_sensor ft_sensor.launch.py

```

Publishes raw wrench on `/duco_ft_sensor/wrench_raw` (BEST_EFFORT) at the
sensor's native rate (a few hundred Hz over USB at 460800 baud).

Optional raw-wrench web plot on <http://localhost:8080/>:

```bash
ros2 launch ft_sensor_dashboard dashboard.launch.py topic:=/duco_ft_sensor/wrench_raw port:=8080
```

### 4. Gravity compensation

```bash
ros2 launch ft_sensor_gravity_compensation compensation.launch.py dashboard_port:=8100
```

Subscribes to the raw wrench + `/tf`, subtracts the tool's gravity-
induced force/torque using the stored end-effector profile, and publishes
the **gravity-compensated** wrench on `/duco_ft_sensor/wrench_compensated`
(BEST_EFFORT).  Passing `dashboard_port:=8100` also serves the
calibration UI on <http://localhost:8100/> (omit it to run headless).

### 5. Cartesian-controller orchestrator

```bash
# Conservative real-HW limits + dashboard-friendly defaults.
ros2 launch cartesian_control_manager cartesian_control_real.launch.py
```

This:

* loads three FZI Cartesian controllers into `controller_manager` in
  the **inactive** state (operator picks one and engages):
  * `cartesian_force_controller` -- pure free-drive (target_wrench = 0),
  * `cartesian_motion_controller` -- pose-hold (snapshots current pose
    on activate),
  * `cartesian_compliance_controller` -- pose-hold + yields to wrench;
* relays `/duco_ft_sensor/wrench_compensated` (BEST_EFFORT) onto each
  controller's RELIABLE input `/<controller>/ft_sensor_wrench`,
* can publish a constant zero `WrenchStamped` at 10 Hz on
  `/<controller>/target_wrench` for the force + compliance controllers
  (FZI minimises `target - measured`, so identically zero target =
  pure free-drive / compliance against zero).  This heartbeat is **off
  by default** (`publish_target_wrench` = `false`) so an external
  publisher can own the topic; FZI still defaults its internal target to
  zero, so free-drive works either way,
* exposes `~/engage` and `~/disengage` `Trigger` services that
  atomically swap `arm_1_controller` <-> the **active** Cartesian
  controller (selected via the `active_controller_name` parameter or
  the dashboard's dropdown -- locked while engaged),
* runs a 50 Hz safety supervisor that auto-disengages on stale topics
  or wrench / torque limit violation,
* publishes a JSON status snapshot on `/cartesian_control_manager/state`
  (`std_msgs/String`, RELIABLE + TRANSIENT_LOCAL) including the
  catalogue, the current `active_controller`, and the
  `engaged_controller`.

Use `cartesian_control.launch.py` instead of `_real` when running
against fake hardware (no conservative-limit overlay).

> Requires step 2 (`aux_frame_manager`) to be up: the FZI controllers
> read their chain from `/cartesian/robot_description`
> (`urdf_from_topic: true`), so without that topic they load but stay
> deferred and never reach `inactive`.

Verify:

```bash
ros2 control list_controllers
# joint_state_broadcaster           active
# arm_1_controller                  active
# cartesian_force_controller        inactive   <-- selectable, standby
# cartesian_motion_controller       inactive
# cartesian_compliance_controller   inactive
ros2 topic hz /cartesian_force_controller/ft_sensor_wrench   # ~FT rate (relay always on)
ros2 service list | grep cartesian_control_manager
# The zero-target heartbeat is OFF by default; turn it on to monitor it:
ros2 param set /cartesian_control_manager publish_target_wrench true
ros2 topic hz /cartesian_force_controller/target_wrench   # ~10 Hz
# Switch which controller engage will activate (only while idle):
ros2 param set /cartesian_control_manager active_controller_name cartesian_compliance_controller
```

### 6. Cartesian dashboard (optional but recommended)

```bash
ros2 launch cartesian_controller_dashboard dashboard.launch.py port:=8120
```

Open <http://localhost:8120/>.  The dashboard:

* shows engaged / tripped / idle status, live wrench, freshness pills,
* has big **Engage** / **Disengage** buttons (call the orchestrator's
  Trigger services),
* renders a live 3D skeleton of `/robot_description` with the URDF's
  TF tree (drag to orbit, wheel to zoom),
* lets you live-tune the FZI controller's gains
  (`pd_gains.trans_*.p`, `pd_gains.rot_*.p`, `solver.error_scale`,
  `solver.iterations`) without restarting anything,
* has a **Tool frames** panel for editing the xyz / rpy of each
  `aux_frame_manager.aux_frames` entry (e.g. `ft_sensor_link`,
  `compliance_link`).  Saving writes back to `config/robot_config.yaml`
  preserving comments **and** routes the change through
  `aux_frame_manager`, which republishes the canonical URDF so the
  engaged FZI controller swaps its KDL chain **live** (no relaunch
  needed).

The dashboard is purely a UI; closing it does **not** stop the
orchestrator, the controller, or the safety supervisor.

### 7. Robot-state dashboard (optional)

```bash
ros2 launch duco_dashboard dashboard.launch.py
```

Open <http://localhost:8090/>.  Shows joint angles / velocities /
efforts, controller list, TCP pose, latest wrench.

### Engage / disengage

Either click the dashboard buttons, or use the CLI:

```bash
ros2 service call /cartesian_control_manager/engage    std_srvs/srv/Trigger
ros2 service call /cartesian_control_manager/disengage std_srvs/srv/Trigger
```

Engage preconditions: at least one wrench + one joint_states have
arrived, and the arm is effectively stationary
(`engage_max_joint_velocity`, default 0.02 rad/s on real HW).

Auto-disengage trips: stale wrench, stale joint_states,
`|F| > max_wrench_force`, `|T| > max_wrench_torque`.  The supervisor
reverses the controller switch so the JTC takes over and the arm holds
its current pose.

### Setting the target wrench

While engaged on the `cartesian_force_controller` (or
`cartesian_compliance_controller`), the FZI plugin minimises
`target_wrench − measured_wrench`.  An all-zero target gives **pure
free-drive** (the robot yields to operator pushes); a non-zero target
makes the robot **actively apply that wrench** to its environment.

By default the wrench is interpreted in the **end-effector frame**
(`hand_frame_control:=true`).  Set
`ros2 param set /cartesian_force_controller hand_frame_control false`
to interpret it in the robot base frame instead.

The orchestrator owns the publisher to
`/<active_controller>/target_wrench` and gives you two ways to drive
it -- both **safety-clamped** to `max_wrench_force` /
`max_wrench_torque` before being forwarded to FZI.

#### 1. Static setpoint via parameters (one-shot, low rate)

For fixed-direction tasks (constant push, weight assist, ...).  The
orchestrator publishes these six values at `fzi_target_rate_hz`
(default 10 Hz) whenever no external publisher is fresh:

```bash
# Apply +10 N along the EE +Z axis (e.g. press into a surface).
ros2 param set /cartesian_control_manager target_wrench_force_z 10.0

# Zero everything (pure free-drive again).
for ax in x y z; do
  ros2 param set /cartesian_control_manager target_wrench_force_$ax  0.0
  ros2 param set /cartesian_control_manager target_wrench_torque_$ax 0.0
done
```

These six params can also be set as launch overrides:

```bash
ros2 launch cartesian_control_manager cartesian_control_real.launch.py \
    target_wrench_force_z:=10.0
```

#### 2. External topic (high rate, e.g. SpaceMouse / scripts)

Set `external_target_wrench_topic` to any `geometry_msgs/WrenchStamped`
topic.  The orchestrator subscribes **BEST_EFFORT**, clamps each
message, and forwards it to FZI immediately (no rate limiting):

```bash
# Launch with the SpaceMouse topic wired in:
ros2 launch cartesian_control_manager cartesian_control_real.launch.py \
    external_target_wrench_topic:=/spacemouse/target_wrench \
    external_target_wrench_timeout_sec:=0.2

# Drive it from anywhere -- here a constant 10 N along EE +Z at 100 Hz:
ros2 topic pub -r 100 /spacemouse/target_wrench geometry_msgs/WrenchStamped \
    '{header: {frame_id: ""}, wrench: {force: {z: 10.0}}}'
```

If no message arrives for `external_target_wrench_timeout_sec`
(default 0.2 s) the heartbeat falls back to the parameter setpoint, so
losing the external publisher disengages the active wrench gracefully
rather than freezing the last value.

#### 3. Direct to FZI (bypasses the safety clamp)

You can also publish straight to `/<controller>/target_wrench`.  By
default the orchestrator's own heartbeat is **off**
(`publish_target_wrench` = `false`), so nothing competes with you.  If
you enabled it, the heartbeat (and the external-topic forwarder) will
keep overwriting you at 10 Hz unless you publish faster **and** keep
`external_target_wrench_topic` empty.  Use only for debugging.

#### Verifying

```bash
# What the active controller actually sees:
ros2 topic echo /cartesian_force_controller/target_wrench

# Orchestrator status (engaged, active controller, last external age):
ros2 topic echo /cartesian_control_manager/state --once
```

Sign conventions on the GCR5_910 with the default URDF: EE +Z points
along the tool away from the wrist, so `target_wrench_force_z = +10`
means "press the tool tip into the surface in front of it".

> Tuning note: holding a steady non-zero wrench against a **rigid**
> surface is governed by the force-control loop gain
> (`pd_gains.*.p × solver.error_scale`).  The packaged defaults in
> [`config/fzi_preset.yaml`](src/duco_robot_bringup/config/fzi_preset.yaml)
> are tuned for stable rigid contact.  See
> [Live tuning of the FZI controller](#live-tuning-of-the-fzi-controller)
> if you need to trade some contact stability for snappier free-drive
> response, or vice versa.

---

## Teleoperation (Alicia-D leader arm)

A second, independent workflow that drives the Duco follower from the
Alicia-D 6-DoF leader arm.  Maps the leader's joint angles to the
follower's joints in real time through a **per-joint velocity- and
acceleration-limited interpolator** that is seeded from the robot's
actual pose on every SYNC engage -- so a large leader / follower
mismatch closes smoothly instead of tripping the driver's
"position deviation too large" safety stop.

```bash
# 1. Same robot bringup as the Cartesian flow above.
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py use_rviz:=false

# 2. Plug in the Alicia-D leader arm, then in a second terminal:
ros2 launch alicia_teleop alicia_teleop.launch.py
```

The teleop launch brings up three things together:

* `alicia_duo_leader_driver` -- serial driver for the leader
  (publishes `/arm_joint_state`),
* `alicia_duo_leader_dashboard` -- optional leader web UI on
  <http://localhost:8130/>,
* the `alicia_teleop` bridge node itself.

Pass `launch_driver:=false` or `launch_dashboard:=false` if either piece
is already running elsewhere.  Press **SYNC** on the leader to engage;
release to disengage.

By default the bridge uses `command_mode:=forward_position` (FZI-style
direct position streaming via `forward_command_controller`) and
auto-switches `arm_1_controller` ↔ `forward_position_controller` at
launch and on shutdown.  The interpolator caps live under
`alicia_teleop:` in [`config/robot_config.yaml`](config/robot_config.yaml):

* `max_velocity` -- rad/s per joint (default `3.0`)
* `max_acceleration` -- rad/s² per joint (default `10.0`)

Common CLI overrides:

```bash
# Slower / safer caps for first-time tuning:
ros2 launch alicia_teleop alicia_teleop.launch.py \
    max_velocity:=1.5 max_acceleration:=5.0

# Publish JointTrajectory to the JTC instead (auto-switch follows):
ros2 launch alicia_teleop alicia_teleop.launch.py command_mode:=trajectory

# Manage the underlying ros2_control controller yourself:
ros2 launch alicia_teleop alicia_teleop.launch.py auto_switch_controller:=false
```

On engage, the bridge logs `Rate limiter seeded from /joint_states; max
initial gap = X.XXX rad ...` so you can confirm the limiter is starting
from the robot's actual pose.

---

## Teleoperation (SpaceMouse Cartesian jog)

A third teleop workflow: jog the end-effector in real time with a 3Dconnexion
SpaceMouse. [`spacemouse_teleop`](src/spacemouse_teleop) integrates the
`spacenav` velocity twist into a streaming `PoseStamped` target (captured from
TF) that drives `ikt_pose_commander` &mdash; which owns the IK and all hard
safety gates (reachability / jump / speed / staleness).

```bash
# 1. Robot bringup (same as the Cartesian flow above).
ros2 launch robot_bringup duco_bringup.launch.py use_fake_hardware:=false

# 2. Pose commander pinned to the tip link, web dashboard on :8180.
ros2 launch ikt_pose_commander commander.launch.py \
    dashboard_port:=8180 controlled_frame:=compliance_link command_mode:=fpc

# 3. SpaceMouse driver + jog bridge + On/Off dashboard on :8200.
ros2 launch spacemouse_teleop spacemouse_servo.launch.py \
    launch_driver:=true dashboard_port:=8200
```

* **Pose-commander dashboard** <http://localhost:8180/> &mdash; 3D gizmo,
  Read / Send mode, live IK / motion params.
* **SpaceMouse On/Off dashboard** <http://localhost:8200/> &mdash; gate the
  sender: **On** jogs with the puck; **Off** releases the commander so you can
  drive from the :8180 dashboard instead (they share one target topic &mdash; use
  one source at a time).

Teleop defaults (`base_frame`, `tip_frame`, `jog_frame`, speeds, `dashboard_port`)
live under `spacemouse_teleop:` in
[`config/robot_config.yaml`](config/robot_config.yaml); CLI args override. With
`deadman_mode: none` (default) the puck is touch-to-move (centre to hold). See
[`src/spacemouse_teleop/README.md`](src/spacemouse_teleop/README.md) for the full
interface and the all-in-one `spacemouse_teleop.launch.py`.

---

## Quick start (fake hardware -- no real arm needed)

For a contained sanity check without the real Duco controller:

```bash
# In config/robot_config.yaml: duco_robot_bringup.use_fake_hardware: true
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py \
    use_rviz:=false apply_aux_frames:=false
ros2 launch aux_frame_manager cartesian_urdf_source.launch.py \
    end_effector_link:=compliance_link
ros2 launch cartesian_control_manager cartesian_control.launch.py
ros2 launch cartesian_controller_dashboard dashboard.launch.py     # optional
```

The FT pipeline isn't needed for fake-hardware sanity (FZI just sees a
silent topic and won't move).  `aux_frame_manager` **is** still required:
the FZI controllers read their chain from `/cartesian/robot_description`
(`urdf_from_topic: true`), so without it they stay deferred and never
activate.  The orchestrator, dashboard, and controller-switching paths
are exercised end-to-end.

---

## Live tuning of the FZI controller

The plugin's gains are exposed as `ros2 params` on the
`/cartesian_force_controller` node and can be edited at runtime, either
through the dashboard or directly:

```bash
ros2 param set /cartesian_force_controller pd_gains.trans_x.p 0.2
ros2 param set /cartesian_force_controller pd_gains.trans_y.p 0.2
ros2 param set /cartesian_force_controller pd_gains.trans_z.p 0.2
ros2 param set /cartesian_force_controller pd_gains.rot_x.p   0.5
ros2 param set /cartesian_force_controller pd_gains.rot_y.p   0.5
ros2 param set /cartesian_force_controller pd_gains.rot_z.p   0.5
ros2 param set /cartesian_force_controller solver.error_scale 0.05
ros2 param set /cartesian_force_controller solver.iterations  5
```

The packaged defaults
([`config/fzi_preset.yaml`](src/duco_robot_bringup/config/fzi_preset.yaml))
are deliberately conservative; on the GCR5_910 the working values felt
around `pd_gains.trans p = 0.2`, `error_scale = 0.05`.

> **Gotcha:** chaining many `ros2 param set/get` calls with `&&` in a
> single shell line can hang.  Issue them one at a time, optionally
> with `timeout 5 ros2 param ...`.

---

## Architecture

```
+---------------+      +-------------------+      +-------------------------+
| duco_ft_sensor| ---> | ft_sensor_gravity | ---> | /duco_ft_sensor/        |
|  (~960 Hz)    |      | _compensation     |      |  wrench_compensated     |
+---------------+      +-------------------+      +-----------+-------------+
                                                              |
                                              relayed by      |
                                              orchestrator    v
+--------------------------------------+    +-----------------+-------------+
| cartesian_control_manager            |    | cartesian_force_controller    |
|  - spawns + engages FZI controller   |--->|  (C++ ros2_control plugin)    |
|  - wrench relay (BEST -> RELIABLE)   |    |  - forward-dynamics solver    |
|  - target_wrench=0 @ 10 Hz (opt-in)  |    |  - writes joint *position*    |
|  - safety supervisor @ 50 Hz         |    |    commands to hardware       |
|  - /state JSON topic                 |    +-----------------+-------------+
|  - ~/engage, ~/disengage Trigger     |                      |
+--+--------------------+--------------+                      |
   |                    |  switch_controller (atomic JTC<->FZI)|
   v                    v                                      |
+--+----------------+ +-+--------------+                       v
| cartesian_-       | | controller_    |             +--------+-----------+
| controller_-      | | manager        |             | duco_hardware       |
| dashboard (UI)    | |  - JTC         |             |  (DucoHardware-     |
|  http://...:8120  | |  - FZI         |             |   Interface, 250Hz) |
+-------------------+ +----------------+             +---------------------+
```

`cartesian_control_manager` is the only project-owned node that talks to
`controller_manager`'s `switch_controller` service.  All UIs (dashboards)
talk through standard topics + Trigger services, so they're optional and
swappable.

### Canonical URDF flow

The URDF has a single owner.  `duco_robot_bringup` publishes the **bare**
manufacturer URDF on `/robot_description` (launched with
`apply_aux_frames:=false`); `aux_frame_manager` appends the configured
aux frames and publishes the **canonical** URDF on the latched topic
`/cartesian/robot_description`.  The FZI controllers read their kinematic
chain from that topic (`urdf_from_topic: true` in `fzi_preset.yaml`), and
the manager mirrors the same URDF to `robot_state_publisher` so `/tf` and
RViz stay consistent.  This makes tool-frame offsets editable at runtime:
the manager republishes and each engaged controller atomically swaps its
KDL chain in the RT loop -- no relaunch.

```
duco_robot_bringup --/robot_description (bare)--> aux_frame_manager
                                                    |  append aux frames
                       /cartesian/robot_description |  (latched, single writer)
        +-------------------------+-----------------+--------------------+
        v                         v                                      v
  FZI controllers          robot_state_publisher                    aux_frame
  (urdf_from_topic)         --> /tf, /tf_static                     dashboard :8160
```

---

## Multi-robot support

This workspace also drives a Universal Robots **UR15** arm without any
edits inside the `cartesian_controllers_toolkit/` submodule.  The
toolkit's `common.config_manager.get_config()` already honours an
explicit `ROBOT_CONFIG_PATH` env override, so we ship a parallel
per-robot config file and a UR-specific bringup package and let the
toolkit pick whichever one is currently active.

### What's in the workspace for UR15

| file / package | purpose |
|---|---|
| [`src/ur15_robot_bringup`](src/ur15_robot_bringup) | thin wrapper around the apt-installed `ur_robot_driver/ur_control.launch.py`. Reads its defaults from `config/robot_config.yaml::ur15_robot_bringup` (or whichever file `ROBOT_CONFIG_PATH` resolves to). Ships its own `config/fzi_preset.yaml` with `tool0` / `base_link` / `command_interfaces: [position]` for the FZI Cartesian solver. |
| [`config/robot_config.ur15.example.yaml`](config/robot_config.ur15.example.yaml) | UR15 template. Points `cartesian_control_manager.fzi_jtc_controller_name` at `scaled_joint_trajectory_controller`, `fzi_target_frame` at `tool0`, `fzi_controller_yaml_package` at `ur15_robot_bringup`, etc. |
| [`config/robot_config.ur15.yaml`](config/robot_config.ur15.yaml) | local UR15 config, copied from the example on first use (gitignored). |

The apt-installed `ros-humble-ur-robot-driver` provides the driver,
URDF, hardware interface, and the standard UR controller stack
(`scaled_joint_trajectory_controller`, `force_torque_sensor_broadcaster`,
`io_and_status_controller`, etc.).  No additional submodule is needed.

### Switching active robot

The toolkit's config loader resolves `ROBOT_CONFIG_PATH` first, so the
recommended pattern is to set it for the lifetime of your shell:

```bash
# Duco GCR5-910 (default — no env var needed)
cd /home/robot/Documents/duco_control
source install/setup.bash
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py ...

# UR15
cd /home/robot/Documents/duco_control
source install/setup.bash
export ROBOT_CONFIG_PATH=$PWD/config/robot_config.ur15.yaml
ros2 launch ur15_robot_bringup ur15_ros2_control.launch.py ...
```

The toolkit reads `ROBOT_CONFIG_PATH` once per process, so **every**
terminal you use in the UR15 stack (bringup, gravity compensation,
cartesian_control_manager, dashboard) must export it before launching.

### UR15 bringup sequence

The flow mirrors the Duco one; only steps 1 and 2 differ.

1.  **UR15 ros2_control bringup**
    ```bash
    export ROBOT_CONFIG_PATH=$PWD/config/robot_config.ur15.yaml
    ros2 launch ur15_robot_bringup ur15_ros2_control.launch.py
    ```
    This wraps the upstream `ur_control.launch.py` and brings up the
    standard UR controller stack including
    `scaled_joint_trajectory_controller` (active) and
    `force_torque_sensor_broadcaster` (active, publishing
    `/force_torque_sensor_broadcaster/wrench` in `tool0`).
    Smoke-test with `use_fake_hardware:=true` first.
    > Be careful: the upstream `ur_control.launch.py` arg is
    > `use_fake_hardware`, NOT `use_mock_hardware` (the latter is
    > silently ignored and the driver will connect to the real arm
    > at `robot_ip`).

2.  **Gravity compensation** — same launch as Duco, but the topic the
    toolkit subscribes to is now
    `/force_torque_sensor_broadcaster/wrench` (set in
    `robot_config.ur15.yaml::ft_sensor_gravity_compensation.input_topic`).
    No `duco_ft_sensor` launch is needed — UR's broadcaster replaces it.
    ```bash
    ros2 launch ft_sensor_gravity_compensation compensation.launch.py
    ```

3.  **Cartesian control manager** — identical command, identical
    safety supervisor; the manager auto-routes between
    `scaled_joint_trajectory_controller` and the FZI controllers
    because `fzi_jtc_controller_name` is set in
    `robot_config.ur15.yaml`.
    ```bash
    ros2 launch cartesian_control_manager cartesian_control_real.launch.py
    ```

4.  **Dashboards (optional)** — identical commands; the dashboards
    read from whichever config `ROBOT_CONFIG_PATH` points at.

### Smoke test (no robot)

```bash
export ROBOT_CONFIG_PATH=$PWD/config/robot_config.ur15.yaml

# Terminal A
ros2 launch ur15_robot_bringup ur15_ros2_control.launch.py use_fake_hardware:=true

# Terminal B (after A is up)
ros2 launch cartesian_control_manager cartesian_control.launch.py

# Verify
ros2 control list_controllers
#   cartesian_{force,motion,compliance}_controller all "inactive"
#   plus scaled_joint_trajectory_controller "active" (and the rest of UR's stack)

ros2 service call /cartesian_control_manager/engage std_srvs/srv/Trigger
#   expect refusal: "no wrench received yet" — mock_components zero-fills
#   the FT broadcaster's state interfaces, so the manager has not seen
#   a real wrench message yet.
```

### Firewall

When connecting to a real UR15, the URScript pushed by the driver
opens reverse connections back to the host on TCP ports
`50001`–`50004`.  With UFW + `default deny incoming`, the controllers
will load `active` but the arm will not move.  Open the ports
inbound from the robot's IP:

```bash
ROBOT_IP=192.168.1.15
for p in 50001 50002 50003 50004; do
  sudo ufw allow from "$ROBOT_IP" to any port $p proto tcp comment "UR URScript reverse $p"
done
sudo ufw reload
```

### UR-vs-Duco notes

* FZI uses the `position` command interface on UR (same one
  `scaled_joint_trajectory_controller` writes to), so the manager's
  atomic JTC<->FZI swap works identically to Duco.
* UR firmware compensates for gravity internally and FZI does not use
  joint torques, so there is no double-gravity-compensation concern
  (unlike the CRISP setup, which uses `effort` and must explicitly
  disable gravity compensation).
* The Duco-tuned PD gains in `fzi_preset.yaml` transfer to UR15
  because they tune the IK solver's *virtual* end-effector mass, not
  the arm's real dynamics.

---

## Repository layout

```
duco_control/
├── config/
│   ├── robot_config.yaml             # active Duco config (gitignored)
│   ├── robot_config.example.yaml     # Duco template
│   ├── robot_config.ur15.yaml        # active UR15 config (gitignored)
│   └── robot_config.ur15.example.yaml  # UR15 template
├── external/
│   ├── duco_ros2_driver/             # submodule (upstream Duco driver)
│   ├── cartesian_controllers/        # submodule (FZI fork w/ Duco mods)
│   └── cartesian_controllers_toolkit/  # submodule: robot-agnostic Cartesian stack
│       ├── cct_common/               #   shared config loader + URDF helpers
│       ├── aux_frame_manager/        #   single-writer of the canonical URDF (+ 3D dashboard)
│       ├── cartesian_control_manager/  # FZI orchestrator + safety supervisor
│       ├── cartesian_controller_dashboard/  # optional web UI
│       ├── ft_sensor_gravity_compensation/  # gravity-compensated wrench
│       └── ft_sensor_dashboard/      #   sensor-agnostic wrench UI
├── src/
│   ├── duco_robot_bringup/           # per-robot bringup + fzi_preset.yaml (Duco)
│   ├── ur15_robot_bringup/           # per-robot bringup + fzi_preset.yaml (UR15)
│   ├── duco_ft_sensor/
│   ├── duco_dashboard/
│   ├── alicia_teleop/                # leader -> follower teleop bridge
│   └── alicia_leader/                # Alicia-D leader driver + dashboard
└── tools/
```

To clone with all submodules:

```bash
git clone --recurse-submodules https://github.com/yizhongzhang1989/duco_control.git
# or, after a flat clone:
git submodule update --init --recursive
```

---

## Testing

Each Python package has its own `test/` directory:

```bash
colcon test --packages-select cartesian_control_manager cartesian_controller_dashboard
colcon test-result --verbose
```

The `cartesian_control_manager` orchestration suite stubs out the rclpy
service-client API and verifies the controller-switch logic
(request shape, timeout / failure handling, controller-name routing);
the full engage flow is covered by end-to-end runs on the real arm.
