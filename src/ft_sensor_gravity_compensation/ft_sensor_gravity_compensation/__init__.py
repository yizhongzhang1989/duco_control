"""ft_sensor_gravity_compensation: gravity-compensated wrench publisher.

Reads the raw F/T wrench plus the live TCP rotation from /tf, subtracts the
calibrated tool weight and bias, and republishes a compensated wrench.

Submodules:
    calibration       -- pure-Python math (compensation + least-squares fit)
    ee_store          -- YAML persistence for multiple end-effector profiles
    compensation_node -- the rclpy node and built-in HTTP dashboard
"""
