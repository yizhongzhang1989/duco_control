"""DUCO GCR5-910 bringup wrapper.

Owns the canonical ``robot_description`` URDF for the project. This
launch REPLACES the previous thin pass-through to the upstream
``demo_ros2_control.launch.py``: the wrapper now builds the URDF itself
by running xacro on the manufacturer's ``gcr5_910.urdf.xacro`` (and, in
later steps, augmenting it with aux frames declared in
``config/robot_config.yaml``). The single URDF string is then handed to
``robot_state_publisher`` and ``controller_manager`` so every downstream
consumer sees the same kinematic tree.

The wrapper still includes MoveIt's ``move_group`` and ``moveit_rviz``
launches from the upstream package; those build their own URDF via
``MoveItConfigsBuilder`` and will be migrated to share the canonical
URDF in a later step (currently they do not see aux frames).
"""

from pathlib import Path
from typing import Dict, List

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
    'use_rviz': True,
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
    from duco_robot_bringup.urdf_loader import augment_urdf, run_xacro

    aux_frames = _load_aux_frames()
    urdf_xml = run_xacro(
        str(xacro_path),
        {
            'robot_ip': robot_ip,
            'robot_port': robot_port,
            'use_fake_hardware': use_fake_hardware,
        },
    )
    urdf_xml = augment_urdf(urdf_xml, aux_frames)

    robot_description_param = {
        'robot_description': ParameterValue(urdf_xml, value_type=str),
    }

    actions: List = [
        LogInfo(msg=(
            f'[duco_robot_bringup] robot_description built '
            f'({len(urdf_xml)} chars, aux_frames={len(aux_frames)})')),
    ]

    # static_virtual_joint_tfs (optional, from upstream)
    vj_launch = pkg_share / 'launch' / 'static_virtual_joint_tfs.launch.py'
    if vj_launch.exists():
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(vj_launch))))

    # robot_state_publisher (with the canonical URDF)
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

    # controller_manager (gets the same canonical URDF)
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


# ---------------------------------------------------------------------------
# Aux frames loader.
#
# Reads ``duco_robot_bringup.aux_frames`` from ``config/robot_config.yaml``
# (via ``common.config_manager``) and returns a list of normalised
# entries. Each entry is a dict::
#
#   { "name": str, "parent": str, "xyz": [float, float, float],
#     "rpy": [float, float, float] }
#
# Returns ``[]`` (URDF unchanged) when the config or the section is
# missing, or when the loader encounters a malformed entry. A malformed
# entry raises so the launch fails loudly rather than silently dropping
# frames the controllers expect.
# ---------------------------------------------------------------------------
def _load_aux_frames() -> List[Dict]:
    try:
        from common.config_manager import get_config  # type: ignore
    except Exception:  # noqa: BLE001
        return []
    try:
        cfg = get_config()
    except Exception:  # noqa: BLE001
        return []
    if not cfg.has('duco_robot_bringup.aux_frames'):
        return []
    raw = cfg.get('duco_robot_bringup.aux_frames')
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError(
            f"'duco_robot_bringup.aux_frames' must be a list, got "
            f"{type(raw).__name__}")

    out: List[Dict] = []
    for i, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise ValueError(
                f"aux_frames[{i}] must be a mapping, got "
                f"{type(entry).__name__}")
        if 'name' not in entry or 'parent' not in entry:
            raise ValueError(
                f"aux_frames[{i}] requires 'name' and 'parent' keys; "
                f"got keys {list(entry.keys())}")
        out.append({
            'name': str(entry['name']),
            'parent': str(entry['parent']),
            'xyz': _triple(entry.get('xyz'), default=(0.0, 0.0, 0.0),
                           where=f"aux_frames[{i}].xyz"),
            'rpy': _triple(entry.get('rpy'), default=(0.0, 0.0, 0.0),
                           where=f"aux_frames[{i}].rpy"),
        })
    return out


def _triple(value, default, where: str) -> List[float]:
    if value is None:
        return list(default)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(
            f"{where} must be a 3-element list, got {value!r}")
    return [float(v) for v in value]


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

    section_name = 'duco_robot_bringup'
    if not cfg.has(section_name):
        return (dict(_FALLBACKS),
                f"FALLBACK (no '{section_name}:' section in {cfg.config_path})")

    section = cfg.section(section_name)
    return ({key: section.get(key, value) for key, value in _FALLBACKS.items()},
            f'loaded from {cfg.config_path}')


def _launch_bool(value):
    return str(value).lower()