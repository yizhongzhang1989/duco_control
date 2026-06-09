"""Universal Robots UR15 -- full-stack "start all" bringup.

Brings up the COMPLETE UR15 hand-guidance / Cartesian-compliance stack with
a single command:

  Stage 0 (immediately):
    1. ur15_robot_bringup   -- wraps ur_robot_driver's ur_control.launch.py;
                               brings up controller_manager, the URDF,
                               scaled_joint_trajectory_controller (active) and
                               force_torque_sensor_broadcaster (active).  The
                               UR broadcaster IS the F/T source -- there is no
                               separate serial sensor driver like the Duco.
    2. ft_sensor_gravity_compensation -- gravity-compensated wrench (+ optional UI)

  Stage 1 (after ``cartesian_delay`` seconds, so controller_manager is up):
    3. cartesian_control_manager (real-HW conservative limits) -- FZI orchestrator
    4. cartesian_controller_dashboard -- engage / tune web UI   (optional)

Robot selection: this launch pins ``ROBOT_CONFIG_PATH`` to the UR15 config
(``config/robot_config.ur15.yaml``, falling back to
``robot_config.ur15.example.yaml``) unless ``ROBOT_CONFIG_PATH`` is already
exported in the shell.  That config points the gravity-comp input topic at
``/force_torque_sensor_broadcaster/wrench`` and the manager's JTC at
``scaled_joint_trajectory_controller``.

Usage::

    # start everything (real hardware uses the config's use_fake_hardware)
    ros2 launch robot_bringup ur15_bringup.launch.py

    # fake-hardware smoke test (loopback robot_ip is a safety net)
    ros2 launch robot_bringup ur15_bringup.launch.py \\
        use_fake_hardware:=true robot_ip:=127.0.0.1

    # headless (no web dashboards)
    ros2 launch robot_bringup ur15_bringup.launch.py \\
        ft_dashboard_port:=0 launch_cartesian_dashboard:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from robot_bringup.stack import (
    config_value,
    include,
    pin_config,
    resolve_config_path,
)


# Config files probed (relative to the workspace root), in priority order.
_CONFIG_CANDIDATES = (
    'config/robot_config.ur15.yaml',
    'config/robot_config.ur15.example.yaml',
)


def generate_launch_description():
    config_path, config_source = resolve_config_path(_CONFIG_CANDIDATES)
    pin_config(config_path)

    use_fake_default = str(
        config_value('ur15_robot_bringup', 'use_fake_hardware', False)).lower()
    robot_ip_default = str(
        config_value('ur15_robot_bringup', 'robot_ip', '192.168.1.15'))

    args = [
        DeclareLaunchArgument(
            'use_fake_hardware', default_value=use_fake_default,
            description='Use UR mock_components instead of the real arm '
                        '(default comes from ur15_robot_bringup in the config). '
                        'NOTE: the upstream arg is use_fake_hardware, NOT '
                        'use_mock_hardware.'),
        DeclareLaunchArgument(
            'robot_ip', default_value=robot_ip_default,
            description='UR robot IP address (ignored when use_fake_hardware).'),
        DeclareLaunchArgument(
            'ft_dashboard_port', default_value='8100',
            description='Gravity-comp calibration web UI port (0 disables it).'),
        DeclareLaunchArgument(
            'cartesian_dashboard_port', default_value='8120',
            description='cartesian_controller_dashboard web UI port.'),
        DeclareLaunchArgument(
            'launch_cartesian_dashboard', default_value='true',
            description='Start the cartesian_controller_dashboard web UI.'),
        DeclareLaunchArgument(
            'cartesian_delay', default_value='8.0',
            description='Seconds to wait before starting the Cartesian stack '
                        '(lets controller_manager come up first).'),
    ]

    # Stage 0 -- UR bringup (incl. FT broadcaster) + gravity compensation.
    stage0 = [
        include('ur15_robot_bringup', 'ur15_ros2_control.launch.py',
                use_fake_hardware=LaunchConfiguration('use_fake_hardware'),
                robot_ip=LaunchConfiguration('robot_ip')),
        include('ft_sensor_gravity_compensation', 'compensation.launch.py',
                dashboard_port=LaunchConfiguration('ft_dashboard_port')),
    ]

    # Stage 1 -- Cartesian orchestrator + optional dashboard (delayed).
    stage1 = TimerAction(
        period=LaunchConfiguration('cartesian_delay'),
        actions=[
            include('cartesian_control_manager',
                    'cartesian_control_real.launch.py'),
            include('cartesian_controller_dashboard', 'dashboard.launch.py',
                    condition=IfCondition(
                        LaunchConfiguration('launch_cartesian_dashboard')),
                    port=LaunchConfiguration('cartesian_dashboard_port')),
        ],
    )

    actions = []
    if config_path:
        actions.append(
            SetEnvironmentVariable('ROBOT_CONFIG_PATH', config_path))
    actions += [
        LogInfo(msg=f'[robot_bringup/ur15] {config_source}'),
        LogInfo(msg='[robot_bringup/ur15] Stage 0 now: ur15_robot_bringup '
                    '(incl. FT broadcaster) + gravity compensation. Stage 1 '
                    'after cartesian_delay s: cartesian_control_manager + '
                    'dashboard.'),
        *args,
        *stage0,
        stage1,
    ]
    return LaunchDescription(actions)
