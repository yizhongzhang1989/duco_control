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
