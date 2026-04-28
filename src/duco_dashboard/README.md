# duco_dashboard

Web dashboard for checking whether the Duco robot state is healthy while the
`ros2_control` stack is running.

It monitors:

- `/joint_states`
- `/arm_1_controller/state`
- `/duco_ft_sensor/wrench_raw`
- approximate GCR5-910 TCP pose from local forward kinematics

Launch:

```bash
ros2 launch duco_dashboard dashboard.launch.py
```

Open:

```text
http://localhost:8090
```

The dashboard is intentionally project-owned and does not modify the upstream
`duco_ros2_driver` submodule.