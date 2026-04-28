# `duco_ft_sensor`

ROS 2 driver for the Duco 6-DoF force/torque sensor (RS-422 over USB serial).

The node opens the device, streams wrench samples at the sensor's native rate
(~1 kHz), and publishes them on `~/wrench` as `geometry_msgs/WrenchStamped`.
A standalone non-ROS reader and an independent self-test script are also
provided for bench / wiring debugging.

> **Why this readme exists.** The Chinese OCR'd `manual.md` in the repo root
> describes a *different* protocol than the one actually implemented by the
> hardware shipped with this unit. The information below has been verified
> against the physical sensor and supersedes the manual where they disagree.

---

## Verified wire protocol

| | value |
|---|---|
| Serial | **460800 baud**, 8N1, no flow control |
| Frame length | 28 bytes |
| Header | `0x48 0xAA` |
| Payload | 6 × `float32` little-endian, in order **Fx, Fy, Fz, Mx, My, Mz** |
| Tail | `0x0D 0x0A` |
| Final scaling | each `float32 × 10` → forces in **N**, torques in **N·m** |
| Native sample rate | ~960 Hz |

This matches the manufacturer's C# sample (`force[i] = ToSingle(data, i*4+2) * 10`),
**not** the 12-byte packed-int format described in §4.3 of `manual.md` (that
section appears to apply to a different firmware variant — probing this device
at 230400 baud returns 0 bytes).

### Host → sensor commands (4 bytes each)

| bytes | meaning |
|---|---|
| `0x43 0xAA 0x0D 0x0A` | stop streaming |
| `0x47 0xAA 0x0D 0x0A` | tare (zero) then stream @ ~960 Hz |
| `0x48 0xAA 0x0D 0x0A` | stream @ ~960 Hz (no tare) |
| `0x49 0xAA 0x0D 0x0A` | one-shot (single frame) |

### Verified hardware setup

* USB serial adapter exposes `/dev/ttyUSB0` (group `dialout`).
* The kernel's default FTDI **`latency_timer = 16 ms`** chunks frames into
  bursts; reduce to 1 ms to get smooth ~1 kHz delivery:

  ```bash
  echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  ```

  A persistent udev rule is shipped with this repo at
  `/etc/udev/rules.d/99-ftdi-low-latency.rules` (set up once during initial
  install).

### Verified end-to-end performance

When configured as above, this driver publishes wrench messages at:

* **~1 kHz** (verified: 5012 messages received in 5.0 s = 1002.4 Hz)
* **mean inter-message gap 1.000 ms** (with `latency_timer=1`; without the fix,
  bursts would appear separated by ~16 ms)
* **0 dropped bytes** in steady-state streaming.

---

## Topics & services

The node publishes to:

| name | type | notes |
|---|---|---|
| `~/wrench` | `geometry_msgs/WrenchStamped` | sensor data, ~1 kHz, **BEST_EFFORT** QoS, KEEP_LAST(200) |

> **QoS note.** `~/wrench` uses `BEST_EFFORT` so that slow subscribers cannot
> apply back-pressure to the publisher. The default `ros2 topic hz` and
> `ros2 topic echo` use `RELIABLE`, which **will not subscribe** to a
> BEST_EFFORT publisher and will appear silent. To inspect from the CLI, run
> `ros2 topic echo --qos-reliability best_effort /duco_ft_sensor/wrench`, or
> use the dashboard (see [`ft_sensor_dashboard`](../ft_sensor_dashboard)).

Subscribed topics:

| name | type | notes |
|---|---|---|
| `~/command` | `std_msgs/String` | accepts: `start`, `stop`, `tare`, `zero` |

Services:

| name | type | notes |
|---|---|---|
| `~/start` | `std_srvs/Trigger` | begin streaming (no tare) |
| `~/stop` | `std_srvs/Trigger` | stop streaming |
| `~/tare` | `std_srvs/Trigger` | zero the sensor and stream |

Parameters (with defaults):

| name | type | default | meaning |
|---|---|---|---|
| `port` | string | `/dev/ttyUSB0` | serial device |
| `baud` | int | `460800` | baud rate |
| `frame_id` | string | `ft_sensor_link` | header.frame_id on `~/wrench` |
| `publish_rate` | double | `0.0` | 0 = publish every frame (~960 Hz) |
| `autostart` | bool | `true` | start streaming on launch |
| `tare_on_start` | bool | `false` | tare immediately on launch |

---

## Building

```bash
cd ~/Documents/duco_control
colcon build --symlink-install --packages-select duco_ft_sensor
source install/setup.bash
```

Runtime dependency: `python3-serial` (apt-installed).

## Running

```bash
ros2 launch duco_ft_sensor ft_sensor.launch.py
# or with overrides:
ros2 launch duco_ft_sensor ft_sensor.launch.py port:=/dev/ttyUSB0 frame_id:=tool0
ros2 launch duco_ft_sensor ft_sensor.launch.py tare_on_start:=true
```

Send commands at runtime:

```bash
ros2 topic pub --once /duco_ft_sensor/command std_msgs/String "{data: tare}"
ros2 service call /duco_ft_sensor/stop std_srvs/srv/Trigger {}
```

---

## Standalone tools (no ROS required)

Both tools live in this package and become available after sourcing
`install/setup.bash`:

### `read_ft_sensor` — live console reader

Streams the sensor in a tight loop and prints the latest wrench values.

```bash
ros2 run duco_ft_sensor read_ft_sensor                    # stream raw values
ros2 run duco_ft_sensor read_ft_sensor --tare             # zero first
ros2 run duco_ft_sensor read_ft_sensor --port /dev/ttyUSB0 --rate 30
# or directly without ROS:
python3 -m duco_ft_sensor.cli
```

### `tools/test_ft_sensor.py` — fully standalone self-test

A single-file diagnostic that has **no ROS dependency** and **no
dependency on this package** — only `pyserial`. Drop it on any Linux
machine to verify wiring, power, baud rate, framing, and sample rate
without installing the workspace. Lives at
[`tools/test_ft_sensor.py`](../../tools/test_ft_sensor.py) at the repo
root.

```bash
# from any directory:
python3 tools/test_ft_sensor.py                              # core checks
python3 tools/test_ft_sensor.py --port /dev/ttyUSB0 --duration 5
python3 tools/test_ft_sensor.py --check-tare                 # also verify TARE works
python3 tools/test_ft_sensor.py --check-oneshot              # also verify ONE-SHOT works

# install pyserial first if needed:
sudo apt install python3-serial          # Debian / Ubuntu
# or
pip install pyserial                     # any OS
```

Sample output:

```
Duco F/T sensor self-test  port=/dev/ttyUSB0  baud=460800

  device file present + permissions          ... PASS  (/dev/ttyUSB0)
  open serial port                           ... PASS  (opened at 460800 8N1)
  STOP command silences stream               ... PASS  (0 bytes after STOP)
  STREAM command produces data               ... PASS  (28028 bytes in 1.0 s (~1001 frames))
  frames align (header/tail) & decode        ... PASS  (501 good frames, 0 stray bytes; last Fx=-3.99 Fy=-6.33 Fz=-46.03 N)
  sample rate over 2s (~960 Hz expected)     ... PASS  (1976 frames in 2.0s -> 988.0 Hz (drops=0))

6/6 checks passed.
```

The script exits `0` on full pass, `1` otherwise — suitable for CI /
install scripts. The TARE (`0x47`) and ONE-SHOT (`0x49`) checks are
**opt-in** because not every firmware revision honours those commands as
the manual describes — on the unit verified here, `0x47` does not zero
the axes and `0x49` produces no data. Use `--check-tare` /
`--check-oneshot` only on a unit you believe supports them.

> **Important:** the sensor's serial port is exclusive. If
> `ros2 launch duco_ft_sensor ft_sensor.launch.py` is currently running,
> stop it first or the test will fail to open `/dev/ttyUSB0`.

---

## Layout

```
duco_ft_sensor/                  ← this package
├── README.md                    ← this file
├── package.xml
├── setup.py / setup.cfg
├── launch/ft_sensor.launch.py
└── duco_ft_sensor/
    ├── driver.py                ← pure-Python serial driver (no ROS)
    ├── ft_sensor_node.py        ← rclpy node: publishes ~/wrench
    └── cli.py                   ← `read_ft_sensor` console reader

../../tools/test_ft_sensor.py    ← standalone self-test (no ROS, no package import)
```
