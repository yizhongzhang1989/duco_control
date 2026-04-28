# duco_control

ROS 2 workspace for Duco robot control utilities.

## Packages

- `duco_ft_sensor`: serial driver and ROS publisher for the verified Duco F/T sensor protocol.
- `ft_sensor_dashboard`: web dashboard for any `geometry_msgs/WrenchStamped` topic.
- `duco_dashboard`: web dashboard for robot joint, controller, wrench, and TCP state.
- `common`: centralized project configuration loader.
- `duco_robot_bringup`: project-owned launch wrappers that read the central config.

The official Duco ROS 2 driver is tracked as a git submodule at
`external/duco_ros2_driver`. Its packages provide `duco_gcr5_910_moveit_config`,
`duco_support`, `duco_hardware`, `duco_ros_driver`, and related message/support
packages. Keep upstream driver changes in the submodule rather than copying them
into this repo's `src/` directory.

## Configuration

Runtime defaults are stored in `config/robot_config.yaml`, using package names as
top-level keys and launch argument names as second-level keys. Start from the
template if needed:

```bash
cp config/robot_config.example.yaml config/robot_config.yaml
```

## Build

```bash
git submodule update --init --recursive
colcon build --symlink-install
source install/setup.bash
```

## Duco Arm Fake-Hardware Check

The robot stack defaults to fake hardware in `config/robot_config.yaml` for safe
setup and validation:

```bash
ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py use_rviz:=false
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Robot-state dashboard:

```bash
ros2 launch duco_dashboard dashboard.launch.py
```

Open `http://localhost:8090`.

Set `duco_robot_bringup.use_fake_hardware` to `false` only when the
robot controller IP and network are ready for real hardware.

