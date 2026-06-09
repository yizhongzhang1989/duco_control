"""Shared helpers for robot_bringup's per-robot "start all" launches.

Both ``duco_bringup.launch.py`` and ``ur15_bringup.launch.py`` follow the
same recipe:

1. Resolve the robot's config file (respecting an explicit
   ``ROBOT_CONFIG_PATH`` already set in the shell).
2. Pin ``ROBOT_CONFIG_PATH`` so every node in the stack -- and every
   included launch file's ``get_config()`` call -- loads that one config.
3. Include the robot's full launch stack.

Keeping the logic here keeps the two launch files thin and identical in
behaviour.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def resolve_config_path(candidates):
    """Pick the config file for a robot.

    Returns ``(path_or_None, human_readable_source)``.  An explicit
    ``ROBOT_CONFIG_PATH`` already present in the environment always wins, so
    the operator can override the robot's default config from the shell.
    Otherwise the first existing file from ``candidates`` (relative to the
    workspace root) is used.
    """
    existing = os.environ.get('ROBOT_CONFIG_PATH')
    if existing:
        return existing, (
            f'using ROBOT_CONFIG_PATH already set in environment: {existing}')
    root = None
    try:
        from cct_common.workspace_utils import get_workspace_root
        root = get_workspace_root()
    except Exception:  # noqa: BLE001 - cct_common may not be built yet
        root = None
    if root is None:
        root = os.getcwd()
    for rel in candidates:
        p = Path(root) / rel
        if p.is_file():
            return str(p), f'resolved config: {p}'
    return None, (
        f'no config found under {root}/config; the toolkit will fall back '
        'to its own packaged defaults')


def pin_config(config_path):
    """Pin ``ROBOT_CONFIG_PATH`` for this process and all spawned children.

    Setting ``os.environ`` here (at launch-description build time)
    guarantees that (a) the included launches' ``get_config()`` calls --
    which run in this same Python process -- load the right file, and
    (b) every node process the launch spawns inherits the variable.
    """
    if config_path:
        os.environ['ROBOT_CONFIG_PATH'] = config_path


def config_value(section, key, default):
    """Read one key from the pinned config; fall back to ``default``."""
    try:
        from cct_common.config_manager import get_config
        cfg = get_config()
        if cfg.has(section):
            return cfg.section(section).get(key, default)
    except Exception:  # noqa: BLE001 - never let config issues break parsing
        pass
    return default


def include(pkg, launch_file, condition=None, **launch_arguments):
    """Scoped ``IncludeLaunchDescription`` for ``<pkg>/launch/<launch_file>``.

    The include is wrapped in a ``scoped`` ``GroupAction`` so that launch
    argument names that are common across the sub-launches (``port``,
    ``host``, ``wrench_topic``, ``reliability``, ...) do NOT leak between
    sibling includes.  Without this, e.g. the ``port`` passed to one
    dashboard would override the default ``port`` of another dashboard
    declared later in the same launch.  ``forwarding=True`` (the default)
    keeps this meta-launch's own arguments visible inside each group.
    """
    path = os.path.join(
        get_package_share_directory(pkg), 'launch', launch_file)
    inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=launch_arguments.items(),
    )
    return GroupAction([inc], scoped=True, forwarding=True, condition=condition)
