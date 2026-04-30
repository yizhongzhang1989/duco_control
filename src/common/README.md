# `common`

Centralized configuration loader and workspace utilities for the
**duco_control** project. Every other package in this workspace should
read its parameters through `common.config_manager.ConfigManager` so
there is exactly one source of truth (`config/robot_config.yaml`) for
IPs, ports, device paths, robot kinematics, etc.

## Why a centralized config?

This workspace will host several packages that need to agree on the same
values (the F/T sensor's serial port, the robot's IP, the dashboard's
port, etc.). Without a shared config you end up either hard-coding the
same value in many launch files or duplicating launch-args in every
launch invocation. With one YAML file:

- one place to edit when you change machine, network, or hardware;
- the same values are visible to drivers, web dashboards, scripts, and
  ad-hoc tools;
- `robot_config.yaml` is gitignored, so machine-specific overrides don't
  leak into commits — the committed `robot_config.example.yaml` is the
  template.

---

## Repo layout

```
duco_control/
├── config/
│   ├── robot_config.example.yaml   (committed, the template)
│   └── robot_config.yaml           (LOCAL only, gitignored)
└── src/
    └── common/                     (this package)
        └── common/
            ├── config_manager.py
            └── workspace_utils.py
```

To bring up a new machine:

```bash
cp config/robot_config.example.yaml config/robot_config.yaml
${EDITOR:-nano} config/robot_config.yaml
```

If `robot_config.yaml` is missing, `ConfigManager` falls back to the
example file so the workspace still launches with sensible defaults.

You can also point at an explicit file with the
`DUCO_CONTROL_CONFIG=/abs/path/robot_config.yaml` environment variable.

---

## Using it from Python (drivers, nodes, scripts)

```python
from common.config_manager import get_config

cfg = get_config()                          # singleton; cheap to call
print(cfg.config_path)                      # which YAML was loaded

# dot-path access with a default
port = cfg.get("duco_ft_sensor.port",       "/dev/ttyUSB0")
baud = cfg.get("duco_ft_sensor.baud",       460800)
web  = cfg.get("ft_sensor_dashboard.port",  8080)

# scoped view: handy to pass into a sub-component
ft = cfg.section("duco_ft_sensor")          # SectionView
print(ft.get("frame_id"))                   # "ft_sensor_link"

# introspect
cfg.list_sections()                         # ['duco_ft_sensor', 'ft_sensor_dashboard']
cfg.has("duco_ft_sensor.tare_on_start")     # True
```

`get(...)` always returns the supplied `default` if any segment of the
dot path is missing, so you can roll out new keys gradually without
breaking older config files.

## Using it from a launch file

The launch files in this workspace read defaults from the central
config and let CLI overrides win:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.config_manager import get_config

def generate_launch_description():
    cfg = get_config().section("duco_ft_sensor")
    args = [
        DeclareLaunchArgument("port",     default_value=cfg.get("port", "/dev/ttyUSB0")),
        DeclareLaunchArgument("baud",     default_value=str(cfg.get("baud", 460800))),
        DeclareLaunchArgument("frame_id", default_value=cfg.get("frame_id", "ft_sensor_link")),
    ]
    return LaunchDescription([*args, Node(...)])
```

## Workspace utilities

`common.workspace_utils` finds the project root and standard sub-dirs
without baking in any user-specific path.

```python
from common.workspace_utils import (
    get_workspace_root,    # absolute path to the repo root, or None
    get_config_dir,        # <root>/config
    get_temp_dir,          # <root>/temp  (created on demand)
)
```

The root is located by trying, in order:

1. the `DUCO_CONTROL_ROOT` env var,
2. the share directory of any installed package in this workspace,
3. walking up from this file's location (development case),
4. `COLCON_PREFIX_PATH` / `ROS_WORKSPACE`,
5. fallback well-known locations
   (`~/Documents/duco_control`, `~/duco_control`).

A directory qualifies as the project root if it contains both `src/`
and `config/`.

---

## Building

```bash
cd ~/Documents/duco_control
colcon build --symlink-install --packages-select common
source install/setup.bash
```

After sourcing, any other package can `from common.config_manager
import get_config`.

## YAML conventions

- **Top-level keys are package names**, second-level keys are the
  arguments the package's launch file exposes (so they map 1-to-1 to
  `ros2 launch <pkg> <launch_file> <key>:=<value>`). Adding a new
  package = adding a new top-level section.
- Anything under a `paths:` mapping is auto-resolved against the
  project root if it isn't already absolute.
- Strings may contain `${ENV_VAR}`; unset variables are left as-is.
- The optional top-level `version:` is reserved for future schema
  migrations and is not exposed as a "section".

## Hot-reload

`ConfigManager` is a singleton. If you change the YAML at runtime
(usually only in dev/REPL), call:

```python
from common.config_manager import get_config
get_config().reload()
```
