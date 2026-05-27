# duco_cartesian_control

Cartesian-controller setup + operator orchestration for the Duco
GCR5_910 arm.  Brings up FZI's `cartesian_force_controller` in the live
`controller_manager` and provides the operator UX (engage / disengage),
wrench plumbing, zero-target heartbeat, and safety supervisor.

The actual Cartesian compliance hot path is a C++ `ros2_control` plugin
from
[yizhongzhang1989/cartesian_controllers](https://github.com/yizhongzhang1989/cartesian_controllers)
(a fork of [fzi-forschungszentrum-informatik/cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)),
which runs inside `controller_manager` and writes joint position
commands directly to the hardware interface.  This package is the glue
that makes it usable end-to-end on the Duco arm.

The system runs without any dashboard; the
[`cartesian_controller_dashboard`](../cartesian_controller_dashboard)
package is a separate, optional UI that subscribes to the state topic
and calls the engage/disengage services exposed here.

## What this package does

1. **Spawns** `cartesian_force_controller` into the live
   `controller_manager` in the **inactive** state at launch
   (using [`config/fzi_zero_gravity.yaml`](config/fzi_zero_gravity.yaml)).
2. **Subscribes** to the gravity-compensated wrench
   (`/duco_ft_sensor/wrench_compensated`, BEST_EFFORT) and republishes
   it onto FZI's RELIABLE input
   (`/cartesian_force_controller/ft_sensor_wrench`).
3. **Publishes** a constant zero `WrenchStamped` at 10 Hz on
   `/cartesian_force_controller/target_wrench`.  FZI's controller
   minimises `(target - measured)`, so an identically zero target
   means it tries to drive the operator-applied wrench to zero --
   pure free-drive / hand-guidance.
4. **Exposes** `~/engage` and `~/disengage` `Trigger` services.
   Engage atomically deactivates `arm_1_controller` (the JTC) and
   activates `cartesian_force_controller` via
   `controller_manager/switch_controller`; disengage does the reverse.
5. **Supervises safety** at 50 Hz: stale FT or joint_states topic,
   force-magnitude limit, torque-magnitude limit.  Any trip
   auto-disengages and reverses the controller switch so the JTC
   takes over and the arm holds its current pose.
6. **Publishes** orchestration state (engaged / tripped, limits,
   wired topics) as JSON on `~/state` (`std_msgs/String`,
   `TRANSIENT_LOCAL`) at 5 Hz, plus an extra publish on every state
   transition.  Late subscribers (such as a dashboard) see the
   current state immediately.

## Quick start (real hardware)

```bash
# Terminal 1 -- bringup
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py \
    use_fake_hardware:=false use_rviz:=false

# Terminal 2 -- F/T sensor + gravity compensation
ros2 launch duco_ft_sensor ft_sensor.launch.py
ros2 launch ft_sensor_gravity_compensation compensation.launch.py

# Terminal 3 -- this orchestrator (with conservative real-HW limits)
ros2 launch duco_cartesian_control cartesian_control_real.launch.py

# Terminal 4 -- engage / disengage
ros2 service call /duco_cartesian_control/engage    std_srvs/srv/Trigger
ros2 service call /duco_cartesian_control/disengage std_srvs/srv/Trigger

# (optional) Terminal 5 -- web dashboard
ros2 launch cartesian_controller_dashboard dashboard.launch.py
# -> opens at http://<host>:8120/
```

For lab / fake-hardware bring-up use `cartesian_control.launch.py`
instead (no conservative-limit overlay).

## Engage / disengage / safety

| service | type | effect |
|---|---|---|
| `~/engage`    | `std_srvs/srv/Trigger` | switch JTC -> FZI atomically |
| `~/disengage` | `std_srvs/srv/Trigger` | switch FZI -> JTC atomically |

Engage preconditions: at least one wrench + one joint_states have
arrived, and (when `engage_max_joint_velocity > 0`) the arm is
effectively stationary.

Auto-disengage trips:

* wrench topic stale (> `ft_stale_after` seconds, default 0.25)
* joint_states stale (> `joint_states_stale_after` seconds, default 0.25)
* force magnitude > `max_wrench_force` (default 80 N; 100 N in `_real`)
* torque magnitude > `max_wrench_torque` (default 10 Nm)

## State topic

`/duco_cartesian_control/state` (`std_msgs/String`, JSON):

```json
{
  "engaged": false,
  "trip_reason": "",
  "wrench_topic": "/duco_ft_sensor/wrench_compensated",
  "joint_states_topic": "/joint_states",
  "fzi_controller": "cartesian_force_controller",
  "fzi_jtc": "arm_1_controller",
  "fzi_ft_topic": "/cartesian_force_controller/ft_sensor_wrench",
  "fzi_target_topic": "/cartesian_force_controller/target_wrench",
  "controller_manager_ns": "/controller_manager",
  "loop_rate_hz": 50.0,
  "ft_age": 0.012,
  "joint_states_age": 0.004,
  "joint_velocity_max": 0.0,
  "ft_ok": true,
  "joint_states_ok": true,
  "limits": {
    "max_wrench_force": 100.0,
    "max_wrench_torque": 10.0,
    "engage_max_joint_velocity": 0.02,
    "ft_stale_after": 0.25,
    "joint_states_stale_after": 0.25
  }
}
```

QoS is `RELIABLE` + `TRANSIENT_LOCAL` so a dashboard launched after
the orchestrator immediately receives the latest state.

## Tuning the FZI controller (live, no restart)

The FZI plugin's gains are exposed as `ros2 params` on the controller
node `/cartesian_force_controller`.  Tune them while the arm is
running:

```bash
# Translational stiffness (per-axis P gain on Cartesian acceleration).
ros2 param set /cartesian_force_controller pd_gains.trans_x.p 0.2
ros2 param set /cartesian_force_controller pd_gains.trans_y.p 0.2
ros2 param set /cartesian_force_controller pd_gains.trans_z.p 0.2

# Rotational stiffness (radians/sec per Nm).
ros2 param set /cartesian_force_controller pd_gains.rot_x.p 0.5
ros2 param set /cartesian_force_controller pd_gains.rot_y.p 0.5
ros2 param set /cartesian_force_controller pd_gains.rot_z.p 0.5

# Solver step (per-cycle integration step before forward dynamics).
ros2 param set /cartesian_force_controller solver.error_scale   0.05
ros2 param set /cartesian_force_controller solver.iterations    5
```

The same parameters can be edited from the
`cartesian_controller_dashboard` UI.

> **Gotcha:** chaining many `ros2 param set/get` calls with `&&` in a
> single shell line can hang.  Issue them one at a time, optionally
> with `timeout 5 ros2 param ...`.

## Configuration

Defaults are loaded from `config/robot_config.yaml` under the
`duco_cartesian_control:` key (see
`config/robot_config.example.yaml` for the schema).  Anything declared
in the launch file's `_FALLBACKS` can be overridden from the CLI as
well.

| key | default | description |
|---|---|---|
| `wrench_topic`               | `/duco_ft_sensor/wrench_compensated` | input wrench (BEST_EFFORT) |
| `joint_states_topic`         | `/joint_states` | joint state for staleness + engage gate |
| `controller_manager_ns`      | `/controller_manager` | where switch_controller lives |
| `engaged_default`            | `false` | auto-engage on first complete state |
| `fzi_controller_name`        | `cartesian_force_controller` | name registered in controller_manager |
| `fzi_jtc_controller_name`    | `arm_1_controller` | the JTC to swap with |
| `fzi_ft_topic`               | `/cartesian_force_controller/ft_sensor_wrench` | FZI's RELIABLE wrench input |
| `fzi_target_topic`           | `/cartesian_force_controller/target_wrench` | FZI's setpoint topic |
| `fzi_target_frame`           | `link_6` | `header.frame_id` for the zero-target heartbeat |
| `fzi_target_rate_hz`         | `10.0` | heartbeat rate |
| `fzi_service_timeout_sec`    | `2.0` | switch_controller call timeout |
| `loop_rate_hz`               | `50.0` | safety-supervisor tick rate (NOT the FZI hot path) |
| `state_publish_rate_hz`      | `5.0`  | rate at which the state topic is heartbeated |
| `max_wrench_force`           | `80.0`  N | trip threshold |
| `max_wrench_torque`          | `10.0`  Nm | trip threshold |
| `engage_max_joint_velocity`  | `0.05`  rad/s | refuse engage above this |
| `ft_stale_after`             | `0.25`  s | trip if wrench is older |
| `joint_states_stale_after`   | `0.25`  s | trip if joint_states older |

## Architecture

```
+---------------------+       +-----------------------+       +----------------------+
|  duco_ft_sensor     |  --   ft_sensor_gravity_-     |  --   wrench_compensated     |
|  (raw ~960 Hz)      |       compensation            |       (BEST_EFFORT)          |
+---------------------+       +-----------------------+       +----------+-----------+
                                                                          |
                                                  this node relays it     |
                                                          (RELIABLE)      v
+----------------------------------+   <- BEST_EFFORT  ----------------------------+
|  duco_cartesian_control          |   ROS subscriber                              |
|  (orchestrator + supervisor)     |   ROS publisher  -> /cartesian_force_-        |
|  - engage / disengage Trigger    |                     controller/ft_sensor_-    |
|  - state topic (JSON)            |                     wrench (RELIABLE)         |
|  - safety trips @ 50 Hz          |                                               |
|  - target_wrench=0 @ 10 Hz       |   +--------------------------------------+    |
+------+---------------------------+   |  cartesian_force_controller (C++)    |    |
       | switch_controller (sync)      |  - forward-dynamics solver           |    |
       v                               |  - writes joint *position*           |    |
+----------------------------+         |    commands directly to hardware     |    |
|  controller_manager        |  <----  +-------------------+------------------+    |
|  - arm_1_controller (JTC)  |  active <-> inactive (atomic switch on engage)      |
|  - cartesian_force_-       |                                                     |
|    controller (FZI)        |                                                     |
+----------------------------+                                                     |
       |                                                                           |
       v   joint_position_command (250 Hz)                                         |
+----------------------------+                                                     |
|  duco_hardware             |                                                     |
|  (DucoHardwareInterface)   |                                                     |
+----------------------------+                                                     |
```

Implementation notes:

* The engage handler issues a synchronous `switch_controller` service
  call (we wait for the response before returning). This requires a
  `MultiThreadedExecutor` + `ReentrantCallbackGroup` so the response
  can be dispatched on a different executor thread while the handler
  is still on the stack.  See `main()` in `control_node.py`.
* On a clean shutdown the node best-effort switches the JTC back on so
  the arm is in a sane state for the next session.

## Testing

```bash
cd src/duco_cartesian_control
python3 -m pytest test/ -v
```

The tests stub out rclpy's service-client API and verify the
orchestration logic (request shape, timeout / failure handling,
controller-name routing).  The full engage flow (which depends on
rclpy timers + executors + a live controller_manager) is covered by
end-to-end runs on the real arm.

## Controller math & parameter reference

This section documents what each FZI controller actually computes and
how its parameters map onto observable behaviour, so operators can
tune them with intent instead of trial-and-error.  It starts with the
force controller because that is what `cartesian_control_real.launch.py`
brings up by default; motion and compliance controllers will be added
to the same section as their numerical models are documented.

### Force controller

#### What it computes

The hot path runs inside `controller_manager` at the bringup rate
(250 Hz on this arm) and goes through four blocks per cycle:

```
                                                         joint cmd
   FT sensor ----> ftSensorWrenchCallback                (to HW)
                            |                              ^
                            v   m_ft_sensor_wrench         |
target_wrench ----> computeForceError() --> spatial PD --> IK ---> writeJointControlCmds()
                            ^                |             ^
                            |          solver.error_scale  |
                            +-------- ft sensor frame      |
                                       transform           |
                                  (precomputed at          |
                                   on_configure)           |
                                                           |
   joint_states ----> synchronizeJointPositions -----------+
```

The four blocks correspond to the four parameter groups (`ft_sensor_ref_link`,
`pd_gains.*`, `solver.*`, `command_interfaces`); the rest of this
subsection walks through them in order.

#### 1. Wrench error in base frame

[`computeForceError`](../../external/cartesian_controllers/cartesian_force_controller/src/cartesian_force_controller.cpp)
produces a 6-vector "net Cartesian wrench to apply":

$$
\mathbf{e}(t) \;=\; \underbrace{R_{\text{base}\leftarrow\text{sensor}}\, T_{\text{ft}\rightarrow\text{ee}}\, \mathbf{w}_{\text{measured}}(t)}_{\text{FT residual in base frame}}
\;+\;
\underbrace{R_{\text{base}\leftarrow\text{ee}}\, \mathbf{w}_{\text{target}}(t)}_{\text{operator setpoint in base frame}}
$$

* $\mathbf{w}_{\text{measured}}$ is read from `~/ft_sensor_wrench`
  (the gravity-compensated wrench this orchestrator forwards onto
  FZI's RELIABLE input).
* $T_{\text{ft}\rightarrow\text{ee}}$ is a *static* `KDL::Frame`
  precomputed once during `on_configure` from
  $T_{\text{ft}\rightarrow\text{ee}} = T_{\text{base}\rightarrow\text{ee}}^{-1}\, T_{\text{base}\rightarrow\text{ft}}$,
  evaluated at the joint configuration that happened to be current
  when the controller configured.  When the kinematic chain is
  rebuilt live (via `robot_description` updates) this transform is
  recomputed in `onChainRebuilt`.
* $R_{\text{base}\leftarrow\cdot}$ is the rotation matrix from the
  current FK; it is re-evaluated every cycle from the latest joint
  state, so the error vector tracks the EE's current orientation.
* $\mathbf{w}_{\text{target}}$ is the latest message received on
  `~/target_wrench`.  It is interpreted **in the end-effector frame**
  by default (`hand_frame_control: true`); set
  `hand_frame_control: false` to interpret the target in `robot_base_link`
  instead.

**Sign convention.** The two terms are *added*, not subtracted: the
controller assumes the FT topic publishes the wrench the *load*
applies to the sensor.  An operator push in +X then appears as
$+F_x$ on the topic; with $\mathbf{w}_{\text{target}}=0$ the error
points in +X and the IK below moves the EE in +X to relieve the push
&mdash; free-drive.  If you ever see the arm *push toward* the operator
instead of away, the FT publisher is using the opposite sign and the
fix is to flip it at the source (typically a single sign in
`ft_sensor_gravity_compensation`), not to fight it from inside the
controller.

#### 2. Spatial PD on the wrench error

The 6-vector $\mathbf{e}$ is fed through six independent PD
controllers (one per axis: `trans_x`, `trans_y`, `trans_z`, `rot_x`,
`rot_y`, `rot_z`).  Each axis computes

$$
u_i(t) \;=\; P_i\, e_i(t) \;+\; D_i\, \frac{e_i(t) - e_i(t-\Delta t)}{\Delta t}
$$

with $\Delta t = 0.02$ s (the hard-coded `internal_period` in
`CartesianForceController::update`), $P_i$ from `pd_gains.<axis>.p`
and $D_i$ from `pd_gains.<axis>.d`.
The output is scaled by `solver.error_scale` to form the "virtual
applied wrench" that drives the forward-dynamics IK:

$$
\mathbf{F}_{\text{cmd}}(t) \;=\; \text{error\_scale} \cdot \begin{bmatrix} u_{\text{trans\_x}} \\ \vdots \\ u_{\text{rot\_z}} \end{bmatrix}
$$

Three things worth noting:

* The **D term divides by 0.02 s, not by the real cycle time.** The
  ratio $D/P$ therefore has units of seconds even though the outer
  loop runs at 250 Hz.  Rule of thumb on this arm: $D \approx P/40$
  for translational axes (lightly damps de/dt of FT noise);
  $D \approx P/10$ for rotational axes (FT moment-arm coupling
  injects more noise into the rotational channels, so they need more
  damping per unit P).
* **`error_scale` multiplies both P and D equally.** Reducing it does
  not change the closed-loop damping *ratio*; it only slows the response.
  Use it as a global "make the arm less twitchy" knob, not as a tuning
  knob in lieu of `pd_gains.*.d`.
* **`solver.iterations` is IGNORED by the force controller** &mdash; the
  base class declares it but `CartesianForceController::update` runs
  the PD + IK pipeline exactly once per cycle.  (The motion and
  compliance controllers do use it.)

#### 3. Forward-dynamics IK

`Base::computeJointControlCmds(F_cmd, 0.02s)` hands $\mathbf{F}_{\text{cmd}}$
to the forward-dynamics solver in [`ForwardDynamicsSolver::getJointControlCmds`](../../external/cartesian_controllers/cartesian_controller_base/src/ForwardDynamicsSolver.cpp).
The solver treats the robot as a fictitious rigid body with a custom
inertia distribution (last link mass = 1 kg, inertia = 1 kg·m²; all
other links use `solver.forward_dynamics.link_mass`, default 0.1 kg)
and integrates one step forward in 0.02 s using **symplectic (semi-implicit) Euler**:

$$
\ddot{\mathbf{q}} = H(\mathbf{q})^{-1}\, J(\mathbf{q})^{\top}\, \mathbf{F}_{\text{cmd}}, \qquad
\dot{\mathbf{q}}_{t+\Delta t} = 0.9 \cdot \bigl(\dot{\mathbf{q}}_t + \ddot{\mathbf{q}}\, \Delta t\bigr), \qquad
\mathbf{q}_{t+\Delta t} = \mathbf{q}_t + \dot{\mathbf{q}}_{t+\Delta t}\, \Delta t
$$

The crucial detail is that the position update uses the **new**
velocity $\dot{\mathbf{q}}_{t+\Delta t}$, not the old one.  This is a
local patch on top of the upstream FZI implementation, which used
plain explicit (forward) Euler $\mathbf{q}_{t+\Delta t} = \mathbf{q}_t + \dot{\mathbf{q}}_t\,\Delta t$
and was prone to instability when the closed-loop stiffness
$\text{error\_scale}\cdot P\cdot K\cdot\Delta t^2$ approached unity.
The change is a one-line reorder in `ForwardDynamicsSolver::getJointControlCmds`
and does not affect steady-state behaviour (at $\ddot{\mathbf{q}}=0$ both
schemes give identical results).

* $H(\mathbf{q})$ is the joint-space inertia (recomputed every cycle
  via `KDL::ChainDynParam`).
* $J(\mathbf{q})$ is the geometric jacobian of the configured chain.
* The "$\times 0.9$" on $\dot{\mathbf{q}}_{t+\Delta t}$ is an
  **implicit per-cycle damping** baked into the solver: each cycle the
  virtual joint velocities lose 10%.  With $\Delta t = 0.02$ s this
  damps null-space drift at a $\tau \approx 0.2$ s time constant and
  also provides a noise floor of damping that's independent of
  `pd_gains.*.d`.  It is also why the arm comes to rest even when the
  PD gains are very low: stop pushing, and the EE coasts to a halt in
  a few hundred milliseconds.
* The symplectic step does **not** help with pure-damper stability
  limits (the explicit-Euler bound $D\cdot\text{error\_scale}\cdot\Delta t/m < 2$
  still applies); for that you need implicit treatment of the
  velocity-proportional term &mdash; not currently implemented.  In
  practice the $\times 0.9$ multiplier and small $\Delta t$ keep the
  margin two orders of magnitude above any reasonable PD gain.

The result is a `JointTrajectoryPoint` of new joint positions and
velocities; `writeJointControlCmds` writes them onto whichever
hardware-interface handles `command_interfaces` configured (`position`
in this repo).

#### 4. Tuning knobs (live, no restart)

All parameters below live on the `/cartesian_force_controller` node
and accept `SetParameters` writes &mdash; either from the dashboard's
parameter panel or from a shell:

| parameter | type | default | meaning | live? |
|---|---|---:|---|:---:|
| `pd_gains.trans_<x\|y\|z>.p` | double | 5.0  | proportional gain, force → virtual EE acceleration (N → m/s²) | yes |
| `pd_gains.trans_<x\|y\|z>.d` | double | 0.05 | damping on $de/dt$ of the translational wrench error | yes |
| `pd_gains.rot_<x\|y\|z>.p`   | double | 50.0 | proportional gain on torque error | yes |
| `pd_gains.rot_<x\|y\|z>.d`   | double | 5.0  | damping on $de/dt$ of the torque error | yes |
| `solver.error_scale`         | double | 0.05 | global multiplier on the PD output; the inner loop's "step size" | yes |
| `solver.iterations`          | int    | 5    | **declared but ignored** for the force controller | n/a |
| `solver.publish_state_feedback` | bool | true | publish `<ctrl>/current_pose` and `<ctrl>/current_twist` | yes (re-applied at next configure) |
| `solver.forward_dynamics.link_mass` | double | 0.1 | virtual mass on every link *except* the last (which is hard-coded at 1 kg) | yes |
| `ik_solver` | string | `forward_dynamics` | IK strategy; alternatives `damped_least_squares`, `selectively_damped_least_squares`, `jacobian_transpose` | reload only |
| `robot_base_link` / `end_effector_link` / `joints` / `command_interfaces` | (various) | from YAML | KDL chain definition | reload only |
| `ft_sensor_ref_link` | string | `ft_sensor_link` | the URDF frame the FT topic is published in | reload only |
| `hand_frame_control` | bool | true | interpret `target_wrench` in EE frame (true) or base frame (false) | yes |

Tuning workflow that has worked on this arm:

1. **Start gentle.** P=5, D=0.05 on translation; P=50, D=5 on
   rotation; `error_scale = 0.05`.  Engage, push the EE around &mdash; it
   should follow your hand with no perceptible chatter.
2. **If the response feels sluggish**, raise `error_scale` by 1.5× at
   a time (0.05 → 0.075 → 0.1).  This scales both P and D together,
   so closed-loop *damping ratio* is unchanged &mdash; you only get
   snappier response, not more overshoot.  Stop when the arm starts
   to oscillate on hand-release.
3. **If you see chatter / buzz at contact**, raise the **D** term
   first, not P.  Translation: try 0.1, 0.2.  Rotation: try 10, 20.
4. **If the arm drifts in free-drive** (your hand on EE, no contact,
   FT looks zero on the dashboard but the arm slowly creeps), the FT
   gravity compensation has a DC bias.  Fix it at the source
   (`ft_sensor_gravity_compensation` calibration) rather than raising
   the PD gains &mdash; raising P only makes the controller respond to
   the bias *faster*; it does not remove it.
5. **`pd_gains.<axis>.p = 0`** disables that axis entirely (the
   controller will not move along it under wrench input).  Useful for
   constraining hand-guiding to a plane, e.g. lock $z$ for a horizontal
   wipe motion.

Shell recipe (one line at a time &mdash; chaining with `&&` can hang):

```bash
ros2 param set /cartesian_force_controller pd_gains.trans_x.p 5.0
ros2 param set /cartesian_force_controller pd_gains.trans_x.d 0.05
ros2 param set /cartesian_force_controller solver.error_scale 0.05
ros2 param set /cartesian_force_controller hand_frame_control true
```

All of these can also be set persistently in
[`config/fzi_zero_gravity.yaml`](config/fzi_zero_gravity.yaml) under
the `cartesian_force_controller:` block; live changes via
`SetParameters` survive only until the controller is unloaded.

#### What the force controller does *not* do

* It has **no spring back to the engage pose**.  Drop your hand on
  the EE and it stays wherever you let go &mdash; modulo the slow
  null-space decay from the $\times 0.9$ velocity damping.  If you
  want spring-back, switch to the compliance controller.
* It does **not** consume `<ctrl>/target_frame` &mdash; that topic is
  the motion / compliance controllers' input.
* `solver.iterations` does nothing here (see table).  Do not
  re-purpose it as a "stiffness" knob.
