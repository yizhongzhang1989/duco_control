# alicia_teleop

Teleoperate the **Duco GCR5_910** follower from the **Alicia-D** 6-DoF
leader arm.

This package is a single ROS 2 Python node (`teleop_node`) plus a launch
file that also brings up the leader driver and its dashboard. It owns the
real-time mapping from leader joint angles to follower joint commands,
the safety-critical rate limiting that prevents the robot from tripping
its "position deviation too large" hard stop, and the automatic
controller switching so a fresh bringup followed by

```bash
ros2 launch alicia_teleop alicia_teleop.launch.py
```

is enough to start teleoperating.

This README is **the reference for how position control is currently
achieved**, intended for anyone extending the bridge later (e.g. adding
gripper / wrist actions, swapping in a different leader, adding shared
control, switching to a different controller plugin, ...).

---

## 1. Architectural overview

```
+-------------------+     /arm_joint_state    +----------------------+
| Alicia-D leader   |  ---------------------> |  alicia_teleop node  |
| (serial server)   |  ArmJointState 200 Hz   |  (Python, 100 Hz)    |
+-------------------+                         |                      |
                                              |  - SYNC gating        |
+--------------------+    /joint_states       |  - rate limiter       |
| ros2_control       |  --------------------> |  - controller switch  |
| joint_state_       |  JointState 250 Hz     |                      |
| broadcaster        |                        +----------+-----------+
+--------------------+                                   |
                                                         |
       /controller_manager/switch_controller             |
       /controller_manager/list_controllers              |
                  ^                                      |
                  |  on startup / shutdown               |
                  |                                      v
+--------------------------------------------+   /forward_position_controller/commands
| controller_manager                          |  (Float64MultiArray, 100 Hz)
|                                             |   -- OR --
|  arm_1_controller (JTC)         <----+      |   /arm_1_controller/joint_trajectory
|  forward_position_controller    <----+--    |  (JointTrajectory, 100 Hz)
|  joint_state_broadcaster                    |
+--------------------------------------------+
            ^
            |  position command interface (each tick)
            |
+--------------------------------------------+
| duco_hardware (DucoHardwareInterface)       |
|   250 Hz read/write to the GCR5_910 over    |
|   the Duco Thrift SDK                       |
+--------------------------------------------+
```

The node does **not** talk to the robot directly. All hardware writes
go through `ros2_control` so that:

* the existing safety / state-broadcaster / collision-avoidance pieces
  keep working,
* swapping between `JointTrajectory`-based control and direct position
  streaming is a one-parameter change,
* the same bridge will work on any other arm that exposes those two
  standard controllers.

---

## 2. The command pipeline (single 100 Hz timer)

Each timer tick (`_timer_cb`) walks through the same 5 stages:

```
leader joints (rad)                <-- from ArmJointState.joint{1..6}
        |
        |  per-joint scale + offset
        v
target  =  leader * joint_scale  +  joint_offset
        |
        |  rate limiter (vmax, amax)
        |  seeded from /joint_states on SYNC engage
        v
q_cmd, v_cmd          <-- the only thing we publish
        |
        |  command_mode
        v
   +---------+---------+
   |                   |
forward_position    trajectory
   |                   |
   v                   v
Float64MultiArray    JointTrajectory(points=[{positions=q_cmd,
to /forward_position  velocities=v_cmd (if vff), time_from_start=traj_time}])
_controller/commands  to /arm_1_controller/joint_trajectory
```

Every stage is deliberately stateless except the rate limiter, which is
where all the position-control magic lives.

---

## 3. SYNC gating — how engagement works

The Alicia driver publishes two latched physical-button states on every
`ArmJointState` message:

| field  | semantic | values         |
|--------|----------|----------------|
| `but1` | LOCK     | 0 = released, 1 = locked |
| `but2` | SYNC     | 0 = unsync,   1 = sync   |

The current policy in `_leader_cb` is dead simple:

| `but2` (SYNC) | action                            |
|---------------|-----------------------------------|
| `1`           | ENGAGED — drive the follower      |
| `0`           | DISENGAGED — stop publishing      |

LOCK is logged on transition but otherwise ignored. The historical
"transparent / hold-last" state has been removed because the buttons are
now reliable enough that it added no value.

The critical detail for position control: **every transition from
DISENGAGED → ENGAGED clears `_q_cmd` and `_v_cmd` to `None`**. The next
timer tick after engagement will re-seed them from `/joint_states` (see
section 4). That is what makes engagement safe regardless of where the
leader currently is — there is no "warmup" mode, no rejection, just a
guaranteed-bounded ramp.

---

## 4. The rate limiter — the heart of position control

`teleop_node` does **not** publish the leader joint angles directly.
That would cause the follower to receive an instantaneous step from
"where the robot is now" to "where the leader is now" the first time
SYNC is pressed, and the Duco driver would (correctly) refuse to track
it, raising `position deviation too large` and stopping the arm.

Instead, on engage we initialise a per-joint commanded state
`(q_cmd, v_cmd)` from the robot's **actual** pose, then on every tick we
update it with a trapezoidal velocity profile capped by `max_velocity`
(rad/s) and `max_acceleration` (rad/s²) per joint. **`q_cmd` is what gets
published**, not the leader target.

### 4.1 Initial conditions (seeding)

```python
if self._q_cmd is None:
    if self._follower_joints is None:
        # /joint_states hasn't arrived yet -- DON'T publish.
        # Holding off here is safer than falling back to the
        # leader target (which would defeat the whole point).
        warn("Engaged but /joint_states not received yet; holding off")
        return
    self._q_cmd = self._follower_joints.copy()
    self._v_cmd = np.zeros(6)
```

`_follower_joints` is filled by `_joint_state_cb`, which subscribes to
`/joint_states` and matches **by joint name** rather than position
index. This matters because `joint_state_broadcaster` does NOT guarantee
the canonical `[j1, j2, j3, j4, j5, j6]` ordering — in practice on the
GCR5_910 we observe `[j1, j3, j2, j4, j5, j6]`. A positional lookup would
silently swap j2 and j3. See [common/README.md](../common) for that
quirk too.

The seeding log line

```
Rate limiter seeded from /joint_states; max initial gap = 0.421 rad
(vmax=3.00 rad/s, amax=10.00 rad/s^2 will close it smoothly)
```

is the operator's confirmation that the safety behavior is active.

### 4.2 The per-tick update

The integration step in `_timer_cb` is:

```python
err   = target - q_cmd                                # signed position error
v_stop = sqrt(2 * amax * |err|)                       # stopping-velocity cap
v_des = sign(err) * min(|err|/dt, v_stop, vmax)       # desired velocity
dv    = clip(v_des - v_cmd, -amax*dt, +amax*dt)       # accel-limited delta
v_cmd = clip(v_cmd + dv,    -vmax,    +vmax)          # vel-limited result
q_cmd = q_cmd + v_cmd * dt
```

All operations are element-wise NumPy on length-6 arrays. The per-tick
cost is negligible (sub-microsecond).

Why three caps on `v_des`?

* `|err| / dt` — reach the target in **one tick** if it's already close
  enough; without this term the controller would still take a fraction
  of a second to "creep in" once the gap shrinks below `vmax * dt`.
* `sqrt(2 * amax * |err|)` — the **stopping-velocity cap**, derived
  from the kinematic identity $v^2 = 2 a s$. It is the maximum speed
  from which we can still decelerate to zero in distance `|err|` given
  acceleration `amax`. **This is what makes the limiter overshoot-free.**
  Without it, the limiter sprints at `vmax` until it hits the target,
  then physically cannot stop in time and overshoots by
  $v_{max}^2 / (2 a_{max})$.
* `vmax` — the steady-state speed cap.

Why `clip(dv, ±amax*dt)`?

Because $v$ is updated discretely; without that clip an instantaneous
desired-velocity change would propagate to `v_cmd` in one tick,
effectively producing infinite acceleration as far as the downstream
controller / hardware is concerned. The clip enforces $|\dot v| \le a_{max}$.

### 4.3 Behaviour

For an initial gap of $\Delta q$ with $\Delta q \gg v_{max}^2/(2 a_{max})$:

1. **Accelerate phase** — `v_des = vmax`, `v_cmd` ramps up at `amax`.
2. **Cruise phase** — `v_cmd = vmax`, `q_cmd` linear in time.
3. **Decelerate phase** — `v_stop < vmax` so `v_des = v_stop`,
   `v_cmd` ramps down following $v^2 = 2a(\Delta q_{remaining})$.
4. **Final tick** — `|err|/dt < v_stop` so `v_des = |err|/dt`, the last
   step takes us exactly to the target with `v_cmd ≈ 0` (no overshoot).

For very small `Δq` (< one step at `vmax`), only step 4 runs and we
arrive in a single tick.

For a leader that is **moving** (the steady-state case once you've
matched poses), `target` changes ~5 mm worth per tick, `err` stays small
on every joint, `v_stop` is huge, and the limiter is effectively
transparent: $v_{cmd} \approx err/dt$, $q_{cmd}$ tracks `target` with
one-tick lag.

### 4.4 Why this lives in the bridge node and not in the controller

The `forward_command_controller` is intentionally dumb: it writes its
input to the hardware interface every tick, no clamping. The
`JointTrajectoryController` does do its own spline interpolation, but
its velocity / acceleration limits are *trajectory* limits, not
real-time *command-stream* limits — JTC will happily accept a 0.5 rad
step as a single waypoint and try to execute it in `time_from_start`.

Putting the limiter at the source (this node) keeps the math in one
place, makes the same limits apply regardless of `command_mode`, and
makes it trivial to retune from a parameter file without re-deploying
anything controller-side.

### 4.5 Tuning the caps

Defaults live in [config/robot_config.yaml](../../config/robot_config.yaml)
under `alicia_teleop:`:

```yaml
alicia_teleop:
  max_velocity: 3.0          # rad/s   per joint
  max_acceleration: 10.0     # rad/s^2 per joint
```

Override on the CLI:

```bash
ros2 launch alicia_teleop alicia_teleop.launch.py \
    max_velocity:=1.5 max_acceleration:=5.0
```

Rules of thumb:

* `max_velocity` should be **at least** the leader's expected joint
  speed in normal operation; otherwise the follower will lag visibly.
* `max_acceleration` controls how fast a gap is closed and how snappy
  the follower feels. Higher = snappier, but the duco's "position
  deviation too large" tolerance is finite — if a single tick produces
  a position step larger than that tolerance the safety stop trips.
  $\Delta q_{max\,per\,tick} = v_{max} \cdot dt = 3.0 \cdot 0.01 = 30~mrad$
  on the defaults, well within the driver's tolerance.
* If you raise either, raise `rate` proportionally so the per-tick step
  stays small.

---

## 5. Output: two `command_mode`s

The bridge can publish in either of two ros2_control-friendly formats.
**Both are wire-compatible with the same `q_cmd` / `v_cmd` produced by
the rate limiter** — the choice only changes which controller plugin
consumes them.

### 5.1 `forward_position` (DEFAULT)

```
msg = std_msgs/Float64MultiArray
msg.data = q_cmd.tolist()
publish to /forward_position_controller/commands
```

* Topic: `/forward_position_controller/commands` (configurable
  via `forward_position_topic` parameter).
* Consumed by `forward_command_controller/ForwardCommandController` v2.x,
  configured for the `position` interface (see
  `external/duco_ros2_driver/duco_gcr5_910_moveit_config/config/ros2_controllers_hardware.yaml`).
* **No spline, no `time_from_start`, no JTC tolerances.** Whatever
  position we write becomes the joint command on the next controller
  tick. Same write pattern as the FZI Cartesian controllers.
* `trajectory_time` and `velocity_feedforward` are **ignored** in this
  mode (the node logs that fact at startup).

This is the default because:

1. It's the closest analogue to a "velocity-following position drive",
   which is the natural pairing for a leader-follower teleop link.
2. The rate limiter is already doing all the trajectory smoothing we
   need; sending the same point through JTC's spline is redundant work
   that adds ~1 tick of lag.
3. It's the same write pattern used by the rest of this workspace
   (`cartesian_control_manager`), so the controller-switching machinery is
   shared.

### 5.2 `trajectory`

```
traj = trajectory_msgs/JointTrajectory
traj.joint_names = joint_names
point.positions  = q_cmd.tolist()
point.velocities = v_cmd.tolist()                   # if velocity_feedforward
point.time_from_start = Duration(traj_time)         # default 0.05 s
publish to /arm_1_controller/joint_trajectory
```

* Topic: `/arm_1_controller/joint_trajectory`.
* Consumed by JTC; goes through JTC's spline interpolator.
* `velocity_feedforward` adds `v_cmd` (the rate-limited velocity,
  matching the position derivative we publish) to each point so JTC's
  cubic spline matches our profile instead of constructing a
  stop-and-go ramp between successive waypoints.
* `trajectory_time` is the `time_from_start` we hand JTC; it should be
  ≥ 1 controller tick. Default 0.05 s.

Use this mode when you need JTC's tolerance / abort behaviour, or when
some other consumer (e.g. MoveIt) is also publishing to the same JTC.

### 5.3 Switching modes

`command_mode` is **only read once at startup**. Change the parameter
in the config or pass `command_mode:=trajectory` on the launch CLI and
restart the bridge.

---

## 6. Controller auto-switching

`auto_switch_controller: true` (the default) makes the node, at startup:

1. Call `/controller_manager/list_controllers` to snapshot which
   controllers are currently active. The snapshot is kept in
   `original_states`.
2. Compute `(wanted_active, wanted_inactive)` from `command_mode`:
   - `forward_position` → activate `forward_position_controller`,
     deactivate `arm_1_controller`,
   - `trajectory` → activate `arm_1_controller`, deactivate
     `forward_position_controller`.
3. Call `/controller_manager/switch_controller` with `STRICT`
   strictness and `activate_asap=True`. STRICT means the call fails if
   any controller cannot reach the requested state, rather than
   silently activating the ones it can.

On shutdown (any path — Ctrl+C, SIGTERM, exception):

4. Read the live controller states.
5. Compute the diff between current and `original_states`.
6. Call `switch_controller` again to undo whatever step 3 did, so the
   world is left exactly as we found it.

### 6.1 Shutdown signal handling

This is subtle and worth documenting because it bit us during
development:

```python
rclpy.init(args=args,
           signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

stop_flag = {"value": False}
def _sigint_handler(_signum, _frame):
    stop_flag["value"] = True

signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)
```

* `rclpy` installs its own SIGINT handler by default that immediately
  invalidates the global context. If that fired we'd have no live rclpy
  context left for the controller-restore service call in `finally`.
* Disabling rclpy's handler with `SignalHandlerOptions.NO` and adding
  our own one that just sets a flag lets the main loop notice the
  signal, break out cleanly, and run the restore against a still-live
  context.
* We also handle SIGTERM because `ros2 launch` in Humble has a 5 s
  `sigterm_timeout` after which it escalates SIGINT to SIGTERM, and the
  child process needs to react to either.

### 6.2 When to disable auto-switch

Set `auto_switch_controller:=false` if:

* You're driving the controller activations yourself (e.g. from another
  orchestrator).
* You want to run the teleop in `forward_position` mode while a
  separate motion-planning workflow leaves the JTC active (e.g. mixed
  manual + autonomous handoffs).

When disabled, the node will still publish; whether anything moves
depends on whether the matching controller happens to be active.

---

## 7. Joint mapping

The bridge expects 6 leader joints and writes to 6 follower joints. The
mapping is positional + per-joint linear:

```
follower[i] = leader[i] * joint_scale[i] + joint_offset[i]
```

Defaults are identity (`scale=1.0`, `offset=0.0`) because the
**leader-side calibration is already done inside the Alicia driver**
via its `joint_config.yaml` — by the time `ArmJointState` reaches this
node the angles are already in the follower robot's joint frame
(continuous-rotation unwrapped, direction-flipped if needed, zero-offset
applied).

The bridge-side `joint_scale` / `joint_offset` are an escape hatch for
small ad-hoc tweaks (e.g. to compensate for a follower mounting
orientation) without touching the leader calibration file. If you find
yourself filling them in heavily, the right answer is usually to fix
the leader's `joint_config.yaml` instead.

---

## 8. Public parameter reference

All parameters are declared on the `alicia_teleop` node. Values in
parentheses are the in-source defaults; the launch file overrides those
from `config/robot_config.yaml` (`alicia_teleop:` section).

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rate` | `100.0` Hz | Timer rate for `_timer_cb`. `dt = 1/rate`. |
| `joint_names` | `arm_1_joint_{1..6}` | Names used for the JointTrajectory and for matching `/joint_states`. |
| `command_mode` | `forward_position` | `forward_position` or `trajectory`. See section 5. |
| `auto_switch_controller` | `true` | Auto-activate the controller for the mode on startup; restore on shutdown. See section 6. |
| `trajectory_topic` | `/arm_1_controller/joint_trajectory` | Output topic in `trajectory` mode. |
| `forward_position_topic` | `/forward_position_controller/commands` | Output topic in `forward_position` mode. |
| `leader_topic` | `/arm_joint_state` | Input topic from the Alicia driver. |
| `joint_scale` | `[1,1,1,1,1,1]` | Per-joint scale (after the leader's own calibration). |
| `joint_offset` | `[0,0,0,0,0,0]` | Per-joint constant offset (rad). |
| `trajectory_time` | `0.05` s | `time_from_start` on each JointTrajectory point (trajectory mode only). |
| `velocity_feedforward` | `true` | Include `v_cmd` as `point.velocities` (trajectory mode only). |
| `max_velocity` | `3.0` rad/s | Per-joint velocity cap on the rate limiter. |
| `max_acceleration` | `10.0` rad/s² | Per-joint acceleration cap on the rate limiter. |

Validation in `__init__` rejects `max_velocity <= 0`, `max_acceleration <= 0`,
unknown `command_mode`, or any of `joint_names` / `joint_scale` /
`joint_offset` not being length 6.

---

## 9. Topic / service interface

### Inputs

| Topic | Type | Notes |
|-------|------|-------|
| `/arm_joint_state` | `alicia_duo_leader_driver/msg/ArmJointState` | leader pose + button state, ~200 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | follower pose, used to seed the rate limiter |

### Outputs (mutually exclusive per `command_mode`)

| Topic | Type | When |
|-------|------|------|
| `/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | `command_mode == forward_position` |
| `/arm_1_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | `command_mode == trajectory` |

### Services consumed (auto-switch)

| Service | Type | When |
|---------|------|------|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | startup + shutdown if `auto_switch_controller` |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | startup + shutdown if `auto_switch_controller` |

---

## 10. Operational notes

* **Start order is flexible.** If `/joint_states` isn't available yet at
  the moment of engagement, the bridge logs a throttled warning and
  silently holds off publishing until it arrives. So bringup ordering
  (`duco_robot_bringup` first, `alicia_teleop` second) is recommended
  but not strictly required — pressing SYNC before the robot is up
  just defers the first command, it does NOT corrupt the rate limiter.
* **Engaging when leader and follower are far apart** is the original
  failure mode this whole pipeline is built to handle gracefully. The
  bridge will spend $\sim \sqrt{2 \Delta q / a_{max}}$ seconds closing
  the gap at the configured caps, then track normally.
* **Hot-reload of caps** is not supported — the parameters are only
  read in `__init__`. Restart the node to change them. (This is
  intentional: live-tuning `vmax` during a trajectory could otherwise
  create transient `dv > amax*dt` situations.)
* **The 5 s shutdown delay** when running under `ros2 launch` is a
  Humble launch-system quirk (SIGINT → 5 s wait → SIGTERM) and is
  cosmetic; the restore runs correctly either way.

---

## 11. Files in this package

```
alicia_teleop/
├── alicia_teleop/
│   ├── __init__.py
│   └── teleop_node.py          # the entire bridge node
├── launch/
│   └── alicia_teleop.launch.py # composes driver + dashboard + bridge
├── resource/
│   └── alicia_teleop
├── package.xml                 # depends on alicia_duo_leader_driver,
│                               # alicia_duo_leader_dashboard,
│                               # controller_manager_msgs, common, ...
├── setup.cfg
├── setup.py                    # entry_point: teleop_node = ...
└── README.md                   # this file
```

The launch file is thin glue:

* loads defaults from `config/robot_config.yaml` via `common.config_manager`
  (falling back to the hard-coded `_FALLBACKS` dict if the workspace
  config is missing),
* declares CLI overrides,
* optionally `IncludeLaunchDescription`s the Alicia leader driver and
  its dashboard (skip with `launch_driver:=false` / `launch_dashboard:=false`),
* spawns the teleop node with all parameters bound.

---

## 12. Extension points

Common follow-on work and where to make the edit:

* **Add a gripper / wrist channel.** The leader publishes additional
  fields on `ArmJointState`; you'd extend `_leader_cb` to extract them,
  add another publisher (e.g. to a gripper action server), and either
  share or duplicate the rate limiter for the new DoF. Keep the limiter
  per-channel — gripper dynamics differ from arm dynamics.
* **Use a different controller plugin.** Add another `command_mode`
  value, a matching publisher type in `__init__`, a publish branch in
  `_timer_cb`, and the controller-name mapping in
  `_controllers_for_mode`. The rate limiter is reusable as-is.
* **Add shared control / haptic forces from the follower.** The
  follower-side wrench is already published on
  `/duco_ft_sensor/wrench_compensated`. Subscribe to it, fold it into
  `target` (e.g. virtual-fixture deflections) before the rate limiter,
  not after — keeping the rate limiter as the last stage before the
  hardware preserves the safety guarantee.
* **Replace the leader.** The only Alicia-specific code is the
  `ArmJointState` import, the field extraction in `_leader_cb`, and the
  driver launch in `alicia_teleop.launch.py`. Everything downstream of
  `_leader_joints` is leader-agnostic.
* **Live-tunable caps.** Re-declare `max_velocity` / `max_acceleration`
  with `add_on_set_parameters_callback`, but only allow changes when
  `_active is False` (i.e. while disengaged), otherwise restart the
  rate limiter cleanly to avoid the `dv > amax*dt` transient.

---

## 13. Quick verification recipe

```bash
# 1. Start the follower (real hardware or fake).
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py use_rviz:=false

# 2. Plug in the Alicia leader and start the teleop in a second terminal.
ros2 launch alicia_teleop alicia_teleop.launch.py

# 3. Verify controllers were switched.
ros2 control list_controllers
# expected:
#   joint_state_broadcaster      active
#   forward_position_controller  active     <-- switched on by us
#   arm_1_controller             inactive   <-- switched off by us

# 4. Press SYNC on the leader and watch the log.
# Look for:  "Teleop ENGAGED (leader SYNC)"
# Look for:  "Rate limiter seeded from /joint_states; max initial gap = ..."

# 5. Confirm command flow.
ros2 topic hz /forward_position_controller/commands
# ~100 Hz

# 6. Release SYNC, then Ctrl+C the launch. After ~5 s look for:
#   "Restoring controllers: activate=['arm_1_controller'] deactivate=['forward_position_controller']"
ros2 control list_controllers
# original state restored.
```

If any of steps 3, 4, or 6 deviate, walk back through the corresponding
section above — the implementation is intentionally straight-line so
each log line maps 1:1 to one block in `teleop_node.py`.
