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
| [`duco_robot_bringup`](src/duco_robot_bringup) | project-owned launch wrapper around the upstream `duco_*` driver (URDF + `controller_manager` + JTC) | -- |
| [`duco_ft_sensor`](src/duco_ft_sensor) | serial driver + ROS publisher for the Duco F/T sensor | -- |
| [`ft_sensor_dashboard`](src/ft_sensor_dashboard) | optional web UI for any `WrenchStamped` topic | `8080` |
| [`ft_sensor_gravity_compensation`](src/ft_sensor_gravity_compensation) | subscribes to the raw wrench + `/tf`, publishes a gravity-compensated wrench, has its own calibration UI | `8100` |
| [`duco_cartesian_control`](src/duco_cartesian_control) | spawns FZI's `cartesian_force_controller` / `cartesian_motion_controller` / `cartesian_compliance_controller` (all inactive), relays the wrench, publishes a zero target_wrench heartbeat, runs the engage / disengage Trigger services and a safety supervisor | -- |
| [`cartesian_controller_dashboard`](src/cartesian_controller_dashboard) | optional web UI for engage / disengage, controller selection (force / motion / compliance), and live-tuning of the active controller's gains | `8120` |
| [`duco_dashboard`](src/duco_dashboard) | optional web UI for joint / controller / TCP state | `8090` |
| [`common`](src/common) | centralised config loader (reads `config/robot_config.yaml`) |

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
* `duco_robot_bringup.duco_ip`, `duco_port` -- the Duco controller endpoint.
* `duco_ft_sensor.port`, `baud` -- serial device for the F/T sensor.
* `duco_cartesian_control.max_wrench_force`, `max_wrench_torque`,
  `engage_max_joint_velocity` -- safety supervisor trip thresholds.

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

## Bringup sequence (real hardware)

The full free-drive / hand-guidance stack is started by running each of
these in its own terminal, in order.  Every terminal needs
`source install/setup.bash` first; `cd /home/robot/Documents/duco_control`
is assumed.

> Before starting: make sure `duco_robot_bringup.use_fake_hardware: false`
> in `config/robot_config.yaml`, the Duco controller is reachable at
> `duco_ip:duco_port`, and the FT sensor is connected at `duco_ft_sensor.port`.

### 1. Robot bringup -- `controller_manager` + JTC

```bash
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py use_rviz:=false
```

This brings up `controller_manager`, loads the URDF, and activates
`joint_state_broadcaster` and `arm_1_controller` (the
`JointTrajectoryController`).  Verify:

```bash
ros2 control list_controllers
# joint_state_broadcaster   active
# arm_1_controller          active
ros2 topic echo /joint_states --once
```

### 2. F/T sensor driver

```bash
ros2 launch duco_ft_sensor ft_sensor.launch.py
```

Publishes raw wrench on `/duco_ft_sensor/wrench` (BEST_EFFORT) at the
sensor's native rate (~960 Hz).

### 3. Gravity compensation

```bash
ros2 launch ft_sensor_gravity_compensation compensation.launch.py
```

Subscribes to the raw wrench + `/tf`, subtracts the tool's gravity-
induced force/torque using the stored end-effector profile, and publishes
the **gravity-compensated** wrench on `/duco_ft_sensor/wrench_compensated`
(BEST_EFFORT).  Pass `enable_dashboard:=true` to also serve the
calibration UI on port `8100`.

### 4. Cartesian-controller orchestrator

```bash
# Conservative real-HW limits + dashboard-friendly defaults.
ros2 launch duco_cartesian_control cartesian_control_real.launch.py
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
* publishes a constant zero `WrenchStamped` at 10 Hz on
  `/<controller>/target_wrench` for the force + compliance controllers
  (FZI minimises `target - measured`, so identically zero target =
  pure free-drive / compliance against zero),
* exposes `~/engage` and `~/disengage` `Trigger` services that
  atomically swap `arm_1_controller` <-> the **active** Cartesian
  controller (selected via the `active_controller_name` parameter or
  the dashboard's dropdown -- locked while engaged),
* runs a 50 Hz safety supervisor that auto-disengages on stale topics
  or wrench / torque limit violation,
* publishes a JSON status snapshot on `/duco_cartesian_control/state`
  (`std_msgs/String`, RELIABLE + TRANSIENT_LOCAL) including the
  catalogue, the current `active_controller`, and the
  `engaged_controller`.

Use `cartesian_control.launch.py` instead of `_real` when running
against fake hardware (no conservative-limit overlay).

Verify:

```bash
ros2 control list_controllers
# joint_state_broadcaster           active
# arm_1_controller                  active
# cartesian_force_controller        inactive   <-- selectable, standby
# cartesian_motion_controller       inactive
# cartesian_compliance_controller   inactive
ros2 topic hz /cartesian_force_controller/ft_sensor_wrench
ros2 topic hz /cartesian_force_controller/target_wrench   # ~10 Hz
ros2 service list | grep duco_cartesian_control
# Switch which controller engage will activate (only while idle):
ros2 param set /duco_cartesian_control active_controller_name \\
    cartesian_compliance_controller
```

### 5. Cartesian dashboard (optional but recommended)

```bash
ros2 launch cartesian_controller_dashboard dashboard.launch.py
```

Open <http://localhost:8120/>.  The dashboard:

* shows engaged / tripped / idle status, live wrench, freshness pills,
* has big **Engage** / **Disengage** buttons (call the orchestrator's
  Trigger services),
* lets you live-tune the FZI controller's gains
  (`pd_gains.trans_*.p`, `pd_gains.rot_*.p`, `solver.error_scale`,
  `solver.iterations`) without restarting anything.

The dashboard is purely a UI; closing it does **not** stop the
orchestrator, the controller, or the safety supervisor.

### 6. Robot-state dashboard (optional)

```bash
ros2 launch duco_dashboard dashboard.launch.py
```

Open <http://localhost:8090/>.  Shows joint angles / velocities /
efforts, controller list, TCP pose, latest wrench.

### Engage / disengage

Either click the dashboard buttons, or use the CLI:

```bash
ros2 service call /duco_cartesian_control/engage    std_srvs/srv/Trigger
ros2 service call /duco_cartesian_control/disengage std_srvs/srv/Trigger
```

Engage preconditions: at least one wrench + one joint_states have
arrived, and the arm is effectively stationary
(`engage_max_joint_velocity`, default 0.02 rad/s on real HW).

Auto-disengage trips: stale wrench, stale joint_states,
`|F| > max_wrench_force`, `|T| > max_wrench_torque`.  The supervisor
reverses the controller switch so the JTC takes over and the arm holds
its current pose.

---

## Quick start (fake hardware -- no real arm needed)

For a contained sanity check without the real Duco controller:

```bash
# In config/robot_config.yaml: duco_robot_bringup.use_fake_hardware: true
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py use_rviz:=false
ros2 launch duco_cartesian_control cartesian_control.launch.py
ros2 launch cartesian_controller_dashboard dashboard.launch.py     # optional
```

The FT pipeline isn't needed for fake-hardware sanity (FZI just sees a
silent topic and won't move).  The orchestrator, dashboard, and
controller-switching paths are exercised end-to-end.

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
([`config/fzi_zero_gravity.yaml`](src/duco_cartesian_control/config/fzi_zero_gravity.yaml))
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
| duco_cartesian_control               |    | cartesian_force_controller    |
|  - spawns + engages FZI controller   |--->|  (C++ ros2_control plugin)    |
|  - wrench relay (BEST -> RELIABLE)   |    |  - forward-dynamics solver    |
|  - target_wrench=0 @ 10 Hz           |    |  - writes joint *position*    |
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

`duco_cartesian_control` is the only project-owned node that talks to
`controller_manager`'s `switch_controller` service.  All UIs (dashboards)
talk through standard topics + Trigger services, so they're optional and
swappable.

---

## Repository layout

```
duco_control/
├── config/
│   ├── robot_config.yaml             # active config (gitignored)
│   └── robot_config.example.yaml     # template
├── external/
│   ├── duco_ros2_driver/             # submodule (upstream Duco driver)
│   └── cartesian_controllers/        # submodule (FZI fork w/ Duco mods)
├── src/
│   ├── common/
│   ├── duco_robot_bringup/
│   ├── duco_ft_sensor/
│   ├── ft_sensor_dashboard/
│   ├── ft_sensor_gravity_compensation/
│   ├── duco_cartesian_control/
│   ├── cartesian_controller_dashboard/
│   └── duco_dashboard/
└── tools/
```

---

## Testing

Each Python package has its own `test/` directory:

```bash
colcon test --packages-select duco_cartesian_control cartesian_controller_dashboard
colcon test-result --verbose
```

The `duco_cartesian_control` orchestration suite stubs out the rclpy
service-client API and verifies the controller-switch logic
(request shape, timeout / failure handling, controller-name routing);
the full engage flow is covered by end-to-end runs on the real arm.
