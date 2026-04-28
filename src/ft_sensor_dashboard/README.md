# `ft_sensor_dashboard`

Live web dashboard for any ROS 2 publisher of `geometry_msgs/WrenchStamped`.

Subscribes to a configurable wrench topic, runs a small **FastAPI + uvicorn**
server, and serves a single self-contained HTML page that streams the data
to the browser via Server-Sent Events. Open the printed URL in any browser
on the LAN and you get two stacked plots:

* top — **Force** (Fx, Fy, Fz) in **N**
* bottom — **Torque** (Mx, My, Mz) in **N·m**

with live numeric readouts, a measured publish-rate badge, configurable
window length, configurable push rate, and a Freeze button. Multiple
browsers can connect simultaneously.

> **This package is sensor-agnostic.** The default `topic` parameter points
> at [`duco_ft_sensor`](../duco_ft_sensor) for convenience, but the
> dashboard works with **any** node that publishes
> `geometry_msgs/WrenchStamped`. To use it with a different publisher just
> override `topic` (and, if needed, `reliability`) — there is no
> `duco_ft_sensor` build/runtime dependency.

---

## Building

```bash
cd ~/Documents/duco_control
colcon build --symlink-install --packages-select ft_sensor_dashboard
source install/setup.bash
```

Runtime dependencies (apt-installed in `package.xml`):

| dep | what it's for |
|---|---|
| `python3-fastapi` | HTTP routing + ASGI app |
| `python3-uvicorn` | ASGI server |
| `rclpy`, `geometry_msgs` | ROS subscriber |

There is no JavaScript build step — the page is one HTML file served from
memory.

## Running

The launch file starts the node and binds the web server:

```bash
ros2 launch ft_sensor_dashboard dashboard.launch.py
```

On startup the node prints the URL, e.g.:

```
[ft_sensor_dashboard]: web dashboard at  http://10.172.102.37:8080/  (FastAPI/uvicorn bound on 0.0.0.0:8080)
```

Open that URL in any browser on the same network.

### Pointing the dashboard at a different sensor / topic

```bash
# different topic on a different node entirely
ros2 launch ft_sensor_dashboard dashboard.launch.py \
    topic:=/my_other_sensor/wrench

# RELIABLE publisher (e.g. a controller's command wrench)
ros2 launch ft_sensor_dashboard dashboard.launch.py \
    topic:=/cartesian_controller/target_wrench \
    reliability:=reliable

# multiple sensors at once: launch one dashboard per sensor on different ports
ros2 launch ft_sensor_dashboard dashboard.launch.py topic:=/left/wrench  port:=8080 title:="left arm"
ros2 launch ft_sensor_dashboard dashboard.launch.py topic:=/right/wrench port:=8081 title:="right arm"
```

## Parameters / launch args

| name | type | default | meaning |
|---|---|---|---|
| `topic` | string | `/duco_ft_sensor/wrench_raw` | wrench topic to subscribe to (any `WrenchStamped` publisher works) |
| `host` | string | `0.0.0.0` | HTTP bind address; `0.0.0.0` = LAN-visible, `127.0.0.1` = local-only |
| `port` | int | `8080` | HTTP port |
| `window_seconds` | double | `10.0` | initial time window shown in the plots |
| `push_rate` | double | `30.0` | initial SSE push frequency in Hz (caps the chart update rate; the underlying topic still streams at full speed) |
| `reliability` | string | `best_effort` | DDS reliability QoS: `best_effort` (default — for streaming sensors) or `reliable` (for command/setpoint topics) |
| `title` | string | `""` | extra label shown in the page title and tab |

> **Reliability tip.** A `reliable` subscriber will not match a
> `best_effort` publisher and vice versa. If the page connects but no data
> appears, check what QoS your publisher uses with
> `ros2 topic info -v <topic>` and set `reliability:=` accordingly. The
> dashboard accepts both flavors.

## Browser controls

The page header has live controls; changes take effect immediately for
the connected browser only (other clients are unaffected).

| control | meaning |
|---|---|
| **window** input | seconds of history shown on the plots; can be set up to 60 s and beyond, the plots use min/max-per-pixel-column downsampling so render cost is independent of window length |
| **push rate** input + **Apply** | per-client SSE push frequency (1–200 Hz). Reconnects with `/events?rate=N` |
| **Freeze / Resume** | pauses chart updates while keeping the connection live; received counter keeps advancing |
| **msg rate** badge | wall-clock arrival rate over the previous 1 s, server-measured, updated once per second, no smoothing |

## HTTP / SSE API

The same server exposes machine-readable endpoints in addition to the HTML
page — convenient for embedding in other dashboards or scripting:

| endpoint | response | notes |
|---|---|---|
| `GET /` | `text/html` | the dashboard page |
| `GET /api/info` | JSON | current config + `received` count + last `rate_hz` |
| `GET /api/latest` | JSON | most recent decoded wrench |
| `GET /events?rate=N` | `text/event-stream` | SSE; each event carries a batch of new samples and the current `rate_hz`. `N` clamps 1–200 Hz |

Example:

```bash
curl http://localhost:8080/api/info
# {"topic":"/duco_ft_sensor/wrench_raw","title":"","window_seconds":10.0,
#  "push_rate":30.0,"server":"fastapi","received":12345,"rate_hz":1000.4}

curl -N http://localhost:8080/events?rate=10
# data: {"samples":[[t,fx,fy,fz,mx,my,mz], ...], "received":..., "rate_hz":...}
# data: {"samples":[...], "received":..., "rate_hz":...}
# ...
```

## Notes on QoS and rate

* The dashboard publisher uses **`BEST_EFFORT` / `KEEP_LAST(200)`** by
  default, so a slow subscriber cannot back-pressure the underlying
  publisher. Default `ros2 topic hz` and `ros2 topic echo` use `RELIABLE`
  — if you want to inspect a `BEST_EFFORT` topic from the CLI you must
  pass `--qos-reliability best_effort`.
* `rate_hz` is computed by counting messages over a tumbling 1-second
  wall-clock bucket; updated once per second; never smoothed across
  buckets. This is what `ros2 topic hz` does conceptually, but it works
  even when the subscriber and publisher have different QoS.

## Layout

```
ft_sensor_dashboard/
├── README.md                      ← this file
├── package.xml
├── setup.py / setup.cfg
├── launch/dashboard.launch.py
└── ft_sensor_dashboard/
    └── dashboard_node.py          ← rclpy subscriber + FastAPI app + HTML/JS
```
