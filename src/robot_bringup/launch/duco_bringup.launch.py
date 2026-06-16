"""Duco GCR5-910 -- full-stack "start all" bringup.

Brings up the COMPLETE Duco hand-guidance / Cartesian-compliance stack with
a single command:

  Stage 0 (immediately):
    1. duco_robot_bringup   -- controller_manager + URDF + arm_1_controller (JTC)
    2. duco_ft_sensor       -- serial F/T sensor driver (raw wrench)
    3. ft_sensor_gravity_compensation -- gravity-compensated wrench (+ optional UI)

  Stage 1 (after ``cartesian_delay`` seconds, so controller_manager is up):
    4. cartesian_control_manager (real-HW conservative limits) -- FZI orchestrator
    5. cartesian_controller_dashboard -- engage / tune web UI   (optional)
    6. duco_dashboard       -- robot-state web UI                (optional)

The Cartesian layer is staged after a short delay because its FZI spawners
need a live ``controller_manager`` (they also wait for the service, so the
delay is just a clean-startup margin).

Robot selection: this launch pins ``ROBOT_CONFIG_PATH`` to the Duco config
(``config/robot_config.yaml``, falling back to ``robot_config.example.yaml``)
unless ``ROBOT_CONFIG_PATH`` is already exported in the shell.

Usage::

    # start everything (real hardware uses the config's use_fake_hardware)
    ros2 launch robot_bringup duco_bringup.launch.py

    # fake-hardware smoke test
    ros2 launch robot_bringup duco_bringup.launch.py use_fake_hardware:=true

    # headless (no web dashboards)
    ros2 launch robot_bringup duco_bringup.launch.py \\
        ft_dashboard_port:=0 launch_cartesian_dashboard:=false \\
        launch_robot_state_dashboard:=false
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
    'config/robot_config.yaml',
    'config/robot_config.example.yaml',
)


def generate_launch_description():
    config_path, config_source = resolve_config_path(_CONFIG_CANDIDATES)
    pin_config(config_path)

    use_fake_default = str(
        config_value('duco_robot_bringup', 'use_fake_hardware', False)).lower()

    args = [
        DeclareLaunchArgument(
            'use_fake_hardware', default_value=use_fake_default,
            description='Use Duco mock_components instead of real hardware '
                        '(default comes from duco_robot_bringup in the config).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch the MoveIt RViz UI from duco_robot_bringup.'),
        DeclareLaunchArgument(
            'ft_dashboard_port', default_value='8100',
            description='Gravity-comp calibration web UI port (0 disables it).'),
        DeclareLaunchArgument(
            'cartesian_dashboard_port', default_value='8120',
            description='cartesian_controller_dashboard web UI port.'),
        DeclareLaunchArgument(
            'aux_frame_dashboard_port', default_value='',
            description='aux_frame_manager 3D web UI port (empty = disabled).'),
        DeclareLaunchArgument(
            'launch_cartesian_dashboard', default_value='true',
            description='Start the cartesian_controller_dashboard web UI.'),
        DeclareLaunchArgument(
            'launch_robot_state_dashboard', default_value='true',
            description='Start the duco_dashboard robot-state web UI.'),
        DeclareLaunchArgument(
            'cartesian_delay', default_value='8.0',
            description='Seconds to wait before starting the Cartesian stack '
                        '(lets controller_manager come up first).'),
    ]

    # Stage 0 -- robot bringup + raw F/T + gravity compensation.
    stage0 = [
        include('duco_robot_bringup', 'gcr5_910_ros2_control.launch.py',
                use_fake_hardware=LaunchConfiguration('use_fake_hardware'),
                use_rviz=LaunchConfiguration('use_rviz'),
                apply_aux_frames='false'),
        # aux_frame_manager owns the aux frames (ft_sensor_link, compliance_link
        # from robot_config.yaml::duco_robot_bringup.aux_frames) now that the
        # bringup publishes the bare manufacturer URDF. It serves the canonical
        # augmented URDF on /cartesian/robot_description for the FZI controllers
        # (urdf_from_topic) and mirrors it to robot_state_publisher for TF. The
        # guard verifies the end-effector chain before the controllers engage.
        include('aux_frame_manager', 'cartesian_urdf_source.launch.py',
                aux_frames_section='duco_robot_bringup',
                robot_base_link='base_link',
                end_effector_link='compliance_link',
                dashboard_port=LaunchConfiguration('aux_frame_dashboard_port')),
        include('duco_ft_sensor', 'ft_sensor.launch.py'),
        include('ft_sensor_gravity_compensation', 'compensation.launch.py',
                dashboard_port=LaunchConfiguration('ft_dashboard_port')),
    ]

    # Stage 1 -- Cartesian orchestrator + optional dashboards (delayed).
    stage1 = TimerAction(
        period=LaunchConfiguration('cartesian_delay'),
        actions=[
            include('cartesian_control_manager',
                    'cartesian_control_real.launch.py'),
            include('cartesian_controller_dashboard', 'dashboard.launch.py',
                    condition=IfCondition(
                        LaunchConfiguration('launch_cartesian_dashboard')),
                    port=LaunchConfiguration('cartesian_dashboard_port')),
            include('duco_dashboard', 'dashboard.launch.py',
                    condition=IfCondition(
                        LaunchConfiguration('launch_robot_state_dashboard'))),
        ],
    )

    actions = []
    if config_path:
        actions.append(
            SetEnvironmentVariable('ROBOT_CONFIG_PATH', config_path))
    actions += [
        LogInfo(msg=f'[robot_bringup/duco] {config_source}'),
        LogInfo(msg='[robot_bringup/duco] Stage 0 now: duco_robot_bringup + '
                    'duco_ft_sensor + gravity compensation. Stage 1 after '
                    'cartesian_delay s: cartesian_control_manager + dashboards.'),
        *args,
        *stage0,
        stage1,
    ]
    return LaunchDescription(actions)
