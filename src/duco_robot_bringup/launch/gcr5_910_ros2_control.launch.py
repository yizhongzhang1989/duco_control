"""DUCO GCR5-910 bringup wrapper.

Owns the base ``robot_description`` URDF for the project. This launch
REPLACES the previous thin pass-through to the upstream
``demo_ros2_control.launch.py``: the wrapper now builds the URDF itself
by running xacro on the manufacturer's ``gcr5_910.urdf.xacro``. The
single URDF string is then handed to ``robot_state_publisher`` and
``controller_manager`` so every downstream consumer sees the same
kinematic tree.

The bringup publishes the **bare** manufacturer URDF on
``/robot_description``; ``aux_frame_manager`` is the sole owner of the
aux frames (``ft_sensor_link`` / ``compliance_link``), appending them to
the canonical URDF it serves on ``/cartesian/robot_description``.

The wrapper still includes MoveIt's ``move_group`` and ``moveit_rviz``
launches from the upstream package; those build their own URDF via
``MoveItConfigsBuilder`` and will be migrated to share the canonical
URDF in a later step.
"""

from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_FALLBACKS = {
    'robot_ip': '192.168.1.10',
    'robot_port': 7003,
    'use_fake_hardware': True,
    'use_rviz': False,
    'db': False,
    'debug': False,
    'publish_frequency': 15.0,
}

_MOVEIT_PKG = 'duco_gcr5_910_moveit_config'
_XACRO_RELPATH = 'config/gcr5_910.urdf.xacro'
_CONTROLLERS_YAML_RELPATH = 'config/ros2_controllers_hardware.yaml'


def generate_launch_description():
    defaults, source = _defaults()

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value=str(defaults['robot_ip']),
            description='Robot IP address'),
        DeclareLaunchArgument(
            'robot_port',
            default_value=str(defaults['robot_port']),
            description='Robot controller TCP port'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=_launch_bool(defaults['use_fake_hardware']),
            description='Use mock_components instead of the real Duco hardware interface'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=_launch_bool(defaults['use_rviz']),
            description='Launch the MoveIt RViz UI'),
        DeclareLaunchArgument(
            'db',
            default_value=_launch_bool(defaults['db']),
            description='Launch the MoveIt warehouse database'),
        DeclareLaunchArgument(
            'debug',
            default_value=_launch_bool(defaults['debug']),
            description='Enable debug mode (reserved; currently unused)'),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value=str(defaults['publish_frequency']),
            description='robot_state_publisher TF rate in Hz'),
    ]

    info = LogInfo(msg=(
        f'[duco_robot_bringup] config: {source}; '
        f'defaults: robot_ip={defaults["robot_ip"]} '
        f'robot_port={defaults["robot_port"]} '
        f'use_fake_hardware={defaults["use_fake_hardware"]} '
        f'use_rviz={defaults["use_rviz"]}'))

    return LaunchDescription([
        *declared_arguments,
        info,
        OpaqueFunction(function=_launch_setup),
    ])


# ---------------------------------------------------------------------------
# Launch-time setup. Runs once when the launch starts; resolves
# LaunchConfigurations to concrete strings and builds the URDF
# synchronously so every spawned node sees the same parameter value.
# ---------------------------------------------------------------------------
def _launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory(_MOVEIT_PKG))
    xacro_path = pkg_share / _XACRO_RELPATH
    controllers_yaml = str(pkg_share / _CONTROLLERS_YAML_RELPATH)

    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    robot_port = LaunchConfiguration('robot_port').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    publish_frequency = LaunchConfiguration('publish_frequency').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    use_db = LaunchConfiguration('db').perform(context).lower() == 'true'

    # Lazy import: surfaces any import error at launch time, not at
    # module import time of this launch file (which would mask it).
    from cct_common.urdf_loader import run_xacro

    # The bringup publishes the bare manufacturer URDF; aux_frame_manager
    # is the sole owner of the aux frames and serves the augmented URDF on
    # its own topic (and mirrors it to robot_state_publisher).
    urdf_xml = run_xacro(
        str(xacro_path),
        {
            'robot_ip': robot_ip,
            'robot_port': robot_port,
            'use_fake_hardware': use_fake_hardware,
        },
    )

    robot_description_param = {
        'robot_description': ParameterValue(urdf_xml, value_type=str),
    }

    actions: List = [
        LogInfo(msg=(
            f'[duco_robot_bringup] robot_description built '
            f'({len(urdf_xml)} chars, bare manufacturer URDF -- aux frames '
            f'owned by aux_frame_manager)')),
    ]

    # static_virtual_joint_tfs (optional, from upstream)
    vj_launch = pkg_share / 'launch' / 'static_virtual_joint_tfs.launch.py'
    if vj_launch.exists():
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(vj_launch))))

    # robot_state_publisher (with the bare URDF; aux_frame_manager later
    # mirrors the canonical augmented URDF here via SetParameters)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        respawn=True,
        output='screen',
        parameters=[
            robot_description_param,
            {'publish_frequency': float(publish_frequency)},
        ],
    )
    actions.append(rsp_node)

    # MoveIt move_group + RViz from upstream. These currently build
    # their own URDF via MoveItConfigsBuilder; aux frames are not yet
    # visible to MoveIt. Will be migrated in a later step.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / 'launch' / 'move_group.launch.py'))))
    if use_rviz:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg_share / 'launch' / 'moveit_rviz.launch.py'))))
    if use_db:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg_share / 'launch' / 'warehouse_db.launch.py'))))

    # controller_manager (gets the same bare URDF as robot_state_publisher)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_param, controllers_yaml],
        output='both',
    )
    actions.append(ros2_control_node)

    # Chained controller spawners: jsb -> arm_1_controller -> fpc.
    # Matches the order from the upstream demo_ros2_control.launch.py.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )
    actions.append(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner])))

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_1_controller',
                   '--controller-manager', '/controller_manager'],
    )
    actions.append(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner])))

    # forward_position_controller starts inactive; teleop / operators
    # switch to it via /controller_manager/switch_controller.
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller',
                   '--controller-manager', '/controller_manager',
                   '--inactive'],
    )
    actions.append(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=arm_controller_spawner,
            on_start=[forward_position_controller_spawner])))

    return actions


def _defaults():
    try:
        from cct_common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f'FALLBACK (could not import cct_common.config_manager: '
                f'{type(exc).__name__}: {exc})')

    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f'FALLBACK (could not load config: '
                f'{type(exc).__name__}: {exc})')

    section_name = 'duco_robot_bringup'
    if not cfg.has(section_name):
        return (dict(_FALLBACKS),
                f"FALLBACK (no '{section_name}:' section in {cfg.config_path})")

    section = cfg.section(section_name)
    return ({key: section.get(key, value) for key, value in _FALLBACKS.items()},
            f'loaded from {cfg.config_path}')


def _launch_bool(value):
    return str(value).lower()