# cartesian_controller_dashboard

Web dashboard for monitoring and tuning the FZI cartesian controllers
(currently `cartesian_force_controller`) on the Duco GCR5_910 arm.

This package is a **pure UI / monitoring** layer.  It does **not**
launch any controllers, wrench relays, or safety supervisors -- those
responsibilities live in
[`duco_cartesian_control`](../duco_cartesian_control).  The dashboard
talks to the rest of the stack purely over the standard ROS interfaces
(topics + Trigger services + the standard `rcl_interfaces` parameter
services).

## What the dashboard shows / does

* **Engaged / tripped status** (read from
  `/duco_cartesian_control/state`).
* **Live wrench** (from `/duco_ft_sensor/wrench_compensated`) with
  per-axis readouts and a magnitude pill.
* **Freshness pills** for the FT and joint_states topics.
* **Safety thresholds** (read-only display from the orchestrator's
  state JSON).
* **Engage / Disengage buttons** that call
  `/duco_cartesian_control/engage` and
  `/duco_cartesian_control/disengage` (`std_srvs/srv/Trigger`).
* **Controller parameter editor** for a curated whitelist of the
  FZI controller's gains:
  * `pd_gains.trans_x.p`, `pd_gains.trans_y.p`, `pd_gains.trans_z.p`
  * `pd_gains.rot_x.p`, `pd_gains.rot_y.p`, `pd_gains.rot_z.p`
  * `solver.error_scale`, `solver.iterations`

  Edits are applied immediately via `SetParameters` on the controller
  node -- no restart required.

## Quick start

```bash
# Bring up the orchestrator first (the dashboard is optional).
ros2 launch duco_cartesian_control cartesian_control_real.launch.py

# Then launch the dashboard.
ros2 launch cartesian_controller_dashboard dashboard.launch.py
# -> opens at http://<host>:8120/
```

The dashboard works even if the orchestrator is down -- it will show
"no orchestrator" but still display live wrench from the FT sensor
pipeline.

## Configuration

Defaults are loaded from `config/robot_config.yaml` under the
`cartesian_controller_dashboard:` key.  CLI overrides are also
supported, e.g.:

```bash
ros2 launch cartesian_controller_dashboard dashboard.launch.py \
    port:=9120 controller_name:=cartesian_compliance_controller
```

| key | default | description |
|---|---|---|
| `orchestrator_ns`     | `/duco_cartesian_control` | namespace of the orchestrator (state topic + engage / disengage services) |
| `controller_name`     | `cartesian_force_controller` | name of the FZI controller node whose parameters are edited |
| `wrench_topic`        | `/duco_ft_sensor/wrench_compensated` | live wrench input |
| `joint_states_topic`  | `/joint_states` | live joint-state freshness |
| `service_timeout_sec` | `2.0` | timeout for engage / disengage / parameter calls |
| `host`, `port`        | `0.0.0.0`, `8120` | HTTP bind address |

## HTTP API

The HTML page uses these JSON endpoints; you can hit them directly
with `curl` for scripting too:

| method | path | purpose |
|---|---|---|
| GET  | `/`             | dashboard HTML |
| GET  | `/api/state`    | snapshot incl. control state, live wrench, current parameter values |
| GET  | `/api/live`     | smaller snapshot of just the changing parts (polled at 5 Hz) |
| GET  | `/api/params`   | current values of the tunable parameters |
| POST | `/api/engage`   | call `/duco_cartesian_control/engage` |
| POST | `/api/disengage`| call `/duco_cartesian_control/disengage` |
| POST | `/api/param`    | set a single tunable parameter; body `{"name": "...", "kind": "double|integer", "value": <number>}` |

The parameter setter only accepts names from the dashboard's
`_TUNABLE_PARAMS` whitelist; arbitrary parameter writes have to go
through `ros2 param set` directly.

## Architecture

```
+------------------------------------+    +------------------------------------+
|  duco_cartesian_control            |    |  cartesian_force_controller (C++)  |
|  - publishes /state (JSON)         |    |  - rcl_interfaces parameter        |
|  - exposes ~/engage, ~/disengage   |    |    services (get/set/list)         |
+--------------------+---------------+    +----------------+-------------------+
                     |                                     |
                     v                                     v
+--------------------+-------------------------------------+-------------------+
|  cartesian_controller_dashboard (this package)                                |
|  - subscribes to /state, /wrench, /joint_states                               |
|  - calls Trigger services on engage / disengage                               |
|  - calls GetParameters / SetParameters for the gain editor                    |
|  - serves HTML + JSON API on http://host:port/                                |
+-------------------------------------------------------------------------------+
                     |
                     v  HTTP
                  browser
```

A `MultiThreadedExecutor` + `ReentrantCallbackGroup` is used so the
synchronous service calls issued from the HTTP handler thread do not
deadlock with the rclpy spin thread.
