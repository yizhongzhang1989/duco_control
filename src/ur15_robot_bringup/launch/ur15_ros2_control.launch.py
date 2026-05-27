"""UR15 ros2_control bringup wrapper.

Thin wrapper around upstream ``ur_robot_driver/ur_control.launch.py``
that:

1. Reads its defaults from ``config/robot_config.yaml`` under the
   ``ur15_robot_bringup`` section (via ``common.config_manager``).
2. Forwards them as launch arguments to the upstream UR launch.
3. Leaves the controller list to the upstream config -- the FZI
   Cartesian controllers are NOT spawned here.  They are spawned by
   ``cartesian_control_manager``'s own launch file using the per-robot
   preset shipped in this package
   (``share/ur15_robot_bringup/config/fzi_preset.yaml``).

The upstream launch already activates
``force_torque_sensor_broadcaster`` (publishing on
``/force_torque_sensor_broadcaster/wrench``) along with
``joint_state_broadcaster``, ``io_and_status_controller``, and
``scaled_joint_trajectory_controller`` (the initial joint controller).
So bringing up this launch alone gives you a UR15 that:

* Streams ``/joint_states`` at ~100 Hz.
* Streams ``/force_torque_sensor_broadcaster/wrench``
  (WrenchStamped, frame ``tool0``) at ~125 Hz.
* Accepts trajectories on ``/scaled_joint_trajectory_controller/...``.

Stack the gravity-compensation + cartesian_control_manager launches on
top to add the FZI Cartesian loop.

Usage::

    ros2 launch ur15_robot_bringup ur15_ros2_control.launch.py
    # override any single arg:
    ros2 launch ur15_robot_bringup ur15_ros2_control.launch.py \\
        use_fake_hardware:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


_FALLBACKS = {
    'ur_type':                  'ur15',
    'robot_ip':                 '192.168.1.15',
    'use_fake_hardware':        False,
    'use_rviz':                 False,
    'headless_mode':            True,
    'reverse_ip':               '0.0.0.0',  # auto-detect; do NOT hardcode
                                            # unless multi-homed routing
                                            # is actually a problem.
    'initial_joint_controller': 'scaled_joint_trajectory_controller',
    'activate_joint_controller': True,
}


def _defaults():
    try:
        from common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f'FALLBACK (could not import common.config_manager: '
                f'{type(exc).__name__}: {exc})')
    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f'FALLBACK (could not load config: '
                f'{type(exc).__name__}: {exc})')
    if not cfg.has('ur15_robot_bringup'):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'ur15_robot_bringup:' section in "
                f'{cfg.config_path})')
    sec = cfg.section('ur15_robot_bringup')
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f'loaded from {cfg.config_path}')


def _bool(v):
    return str(v).lower()


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    declared_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value=str(d['ur_type']),
            description='UR model (passed to ur_control.launch.py).'),
        DeclareLaunchArgument(
            'robot_ip',
            default_value=str(d['robot_ip']),
            description='UR robot IP address.'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=_bool(d['use_fake_hardware']),
            description='Use ros2_control mock_components (fake hardware) '
                        'instead of connecting to the real UR.  This is '
                        'the upstream `ur_control.launch.py` arg name.'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=_bool(d['use_rviz']),
            description="Launch RViz (forwarded to upstream's launch_rviz)."),
        DeclareLaunchArgument(
            'headless_mode',
            default_value=_bool(d['headless_mode']),
            description='Run UR driver in headless mode (no URCap needed).'),
        DeclareLaunchArgument(
            'reverse_ip',
            default_value=str(d['reverse_ip']),
            description='Host IP for URScript reverse connection '
                        '("0.0.0.0" = auto-detect).'),
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value=str(d['initial_joint_controller']),
            description='Which trajectory controller is spawned active.'),
        DeclareLaunchArgument(
            'activate_joint_controller',
            default_value=_bool(d['activate_joint_controller']),
            description='Activate the initial joint controller at spawn.'),
    ]

    info = LogInfo(msg=[
        '[ur15_robot_bringup] config: ', source, '; ',
        'ur_type=', LaunchConfiguration('ur_type'),
        ' robot_ip=', LaunchConfiguration('robot_ip'),
        ' use_fake_hardware=', LaunchConfiguration('use_fake_hardware'),
        ' headless_mode=', LaunchConfiguration('headless_mode'),
    ])

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch', 'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type':                   LaunchConfiguration('ur_type'),
            'robot_ip':                  LaunchConfiguration('robot_ip'),
            'use_fake_hardware':         LaunchConfiguration('use_fake_hardware'),
            'launch_rviz':               LaunchConfiguration('use_rviz'),
            'headless_mode':             LaunchConfiguration('headless_mode'),
            'reverse_ip':                LaunchConfiguration('reverse_ip'),
            'initial_joint_controller':  LaunchConfiguration('initial_joint_controller'),
            'activate_joint_controller': LaunchConfiguration('activate_joint_controller'),
        }.items(),
    )

    return LaunchDescription([
        *declared_arguments,
        info,
        ur_control,
    ])
