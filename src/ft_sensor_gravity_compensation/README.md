# ft_sensor_gravity_compensation

Subtracts tool weight (mass + center of mass) and bias from the raw F/T
sensor wrench using the **live TCP orientation from `/tf`**, and republishes
a compensated wrench.

* **Input:**  `geometry_msgs/WrenchStamped` on `/duco_ft_sensor/wrench_raw`
              + `/tf` (`base_link` -> `link_6`, published by
              `robot_state_publisher` from the bringup launch)
* **Output:** `geometry_msgs/WrenchStamped` on `/duco_ft_sensor/wrench_compensated`
* **Web dashboard (opt-in):** `http://<host>:8100/` -- launched only when
  `enable_dashboard:=true` (or `enable_dashboard: true` under
  `ft_sensor_gravity_compensation:` in `config/robot_config.yaml`)
  * Manage multiple end-effector profiles
  * Record raw samples at varied poses
  * Run a least-squares calibration to fit `(mass, CoM, F_bias, T_bias)`
  * Switch the active end-effector live -- the next outgoing wrench uses it
    immediately

The node is intentionally a **separate package** from `duco_ft_sensor`: the
driver keeps publishing the raw, untouched wrench, while this node owns all
calibration state. That makes both pieces easier to reason about and keeps
the raw stream available for diagnostics.

## Why TF and not a custom TCP-pose topic?

`robot_state_publisher` (started by `duco_robot_bringup`) already publishes
the full kinematic chain on `/tf`, including the
`base_link` -> `link_1` -> ... -> `link_6` transforms. This is the standard
ROS pose representation, so the compensation node simply uses
`tf2_ros.Buffer` -- no separate TCP-pose publisher is needed.

If you later add a dedicated `ft_sensor_link` to the URDF (with a static
transform from `link_6`), just set `sensor_frame:=ft_sensor_link` and the
node will use that frame for the gravity vector.

## Math

Let `R = R_sensor_in_world` be the orientation of the sensor frame in the
world frame, `g_world = (0, 0, -|g|)`, `m` the tool mass and `r` the
center-of-mass in the sensor frame. The gravitational wrench in the sensor
frame is

    F_grav = m * R^T @ g_world
    T_grav = r x F_grav

and the compensated wrench published on the output topic is

    F_out = F_raw - F_bias - F_grav
    T_out = T_raw - T_bias - T_grav

`F_bias` and `T_bias` are constant offsets that absorb sensor zero drift and
hardware mounting forces.

### Calibration (multi-pose least squares)

With `N >= 3` samples of `(R_i, F_obs_i, T_obs_i)` taken at distinctly
different orientations, the parameters are recovered in two linear LSQ
solves:

* For F: stack `[I_3 | g_S_i] [F_bias; m]^T = F_obs_i`     (4 unknowns)
* For T: stack `[I_3 | -[g_S_i]_x] [T_bias; m * r]^T = T_obs_i`  (6 unknowns)

then `r = (m * r) / m`. The dashboard reports the residual RMS and the
condition number of each LSQ matrix; if your sample set spans too few
orientations the corresponding warning shows up immediately.

For best results, record samples in at least three orientations whose
gravity-direction-in-sensor-frame vectors are *not* coplanar -- e.g. the
tool pointing down, then forward, then sideways.

## Workflow

1. Make sure the bringup launch is running (so `/tf` is alive):

       ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py

2. Make sure the F/T driver is publishing the raw wrench:

       ros2 launch duco_ft_sensor ft_sensor.launch.py

3. Launch the compensation node + dashboard:

       ros2 launch ft_sensor_gravity_compensation compensation.launch.py \
           enable_dashboard:=true

   The dashboard is **off by default** -- the node still publishes the
   compensated wrench using whichever profile is currently active in
   `end_effectors.yaml`. Pass `enable_dashboard:=true` (or set
   `enable_dashboard: true` under `ft_sensor_gravity_compensation:` in
   `config/robot_config.yaml`) to bring up the calibration web UI.

4. Open `http://localhost:8100/` and:

   * click **Add** to create a new profile (e.g. `gripper_a`),
   * move the robot to a pose, click **Record current** -- repeat for at
     least 3 well-spread orientations,
   * click **Calibrate** -- the fitted mass / CoM / bias appear and the
     RMS residual is reported,
   * click **Activate** -- the next published `wrench_compensated`
     immediately uses this profile.

Switching the active profile (or re-saving its parameters) takes effect on
the very next incoming raw frame -- no relaunch needed.

## Parameters

| name                  | type    | default                                                        |
|-----------------------|---------|----------------------------------------------------------------|
| `input_topic`         | string  | `/duco_ft_sensor/wrench_raw`                                   |
| `output_topic`        | string  | `/duco_ft_sensor/wrench_compensated`                           |
| `world_frame`         | string  | `base_link`                                                    |
| `sensor_frame`        | string  | `link_6`                                                       |
| `reliability`         | string  | `best_effort`                                                  |
| `publish_when_no_tf`  | bool    | `false`                                                        |
| `storage_path`        | string  | `~/.ros/ft_sensor_gravity_compensation/end_effectors.yaml`     |
| `enable_dashboard`    | bool    | `false` (set `true` to launch the calibration web UI)          |
| `host`                | string  | `0.0.0.0`                                                      |
| `port`                | int     | `8100`                                                         |
| `gravity`             | double  | `9.80665`                                                      |
| `tf_timeout`          | double  | `0.05` (seconds, per `lookup_transform` call)                  |
| `tf_max_age`          | double  | `1.0` (seconds before TF is considered stale and output stops) |

## Storage layout

End-effector profiles are kept as YAML at `storage_path` (default
`~/.ros/ft_sensor_gravity_compensation/end_effectors.yaml`). The file is
written atomically (temp + `os.replace`) on every change, so it is safe to
edit by hand while the node is running -- but the dashboard will overwrite
it on the next mutation. Schema:

```yaml
version: 1
active: gripper_a
end_effectors:
  default:
    description: zero mass / zero bias (no compensation)
    mass: 0.0
    com: [0.0, 0.0, 0.0]
    force_bias: [0.0, 0.0, 0.0]
    torque_bias: [0.0, 0.0, 0.0]
    gravity: 9.80665
    samples: []
  gripper_a:
    description: Robotiq 2F-85
    mass: 0.85
    com: [0.001, -0.002, 0.052]
    force_bias: [0.05, -0.10, 0.21]
    torque_bias: [0.001, 0.002, -0.001]
    gravity: 9.80665
    samples:
      - rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        force:    [0.0, 0.0, -8.34]
        torque:   [0.0, 0.0, 0.0]
        stamp:    1798765432.123
        note:     ""
    last_calibration:
      n_samples: 6
      rms_force: 0.045
      rms_torque: 0.002
      cond_force: 8.4
      cond_torque: 5.7
      warnings: []
      timestamp: 1798765500.000
```

## HTTP API (used by the dashboard)

| route                                       | method | purpose                                              |
|---------------------------------------------|--------|------------------------------------------------------|
| `/`                                         | GET    | dashboard HTML                                       |
| `/api/state`                                | GET    | full snapshot (live + all end-effectors)             |
| `/api/live`                                 | GET    | low-cost live wrench/TF snapshot (poll target)       |
| `/api/create?name=&description=`            | POST   | create a new profile                                 |
| `/api/delete?name=`                         | POST   | delete a profile (`default` is protected)            |
| `/api/rename?old_name=&new_name=`           | POST   | rename a profile                                     |
| `/api/describe?name=&description=`          | POST   | edit description                                     |
| `/api/activate?name=`                       | POST   | switch the live compensation to this profile         |
| `/api/params?name=` (JSON body)             | POST   | overwrite mass/CoM/biases manually                   |
| `/api/sample/record?name=&note=`            | POST   | capture the current `(R, F_raw, T_raw)`              |
| `/api/sample/delete?name=&index=`           | POST   | remove a single sample                               |
| `/api/sample/clear?name=`                   | POST   | remove all samples for a profile                     |
| `/api/calibrate?name=`                      | POST   | LSQ fit -> store params + diagnostics                |

All POST routes return `{"ok": true}` on success or
`{"error": "..."}` with status 400/500 on failure.
