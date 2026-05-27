"""Centralized configuration loader for cartesian_controllers_toolkit.

Reads ``config/robot_config.yaml`` (or, if missing, falls back to
``config/robot_config.example.yaml``) once and caches it in a thread-safe
singleton. Values are accessed by dot-path strings.

Typical use::

    from common.config_manager import get_config

    cfg = get_config()
    topic   = cfg.get("cartesian_control_manager.wrench_topic",
                      "/ft_sensor/wrench_compensated")
    web_port = cfg.get("cartesian_controller_dashboard.port", 8120)

There is also :class:`SectionView` for scoped access::

    sec = cfg.section("cartesian_control_manager")
    print(sec.get("wrench_topic"))
    print(sec.get("max_wrench_force", 0.0))

Strings in the YAML may use ``${ENV_VAR}`` to reference environment
variables (left as-is if the variable is unset). Any string value under
a ``paths`` key whose value is a relative path is resolved against the
project root.
"""

from __future__ import annotations

import os
import re
import tempfile
import threading
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import yaml

from .workspace_utils import get_workspace_root


class ConfigError(RuntimeError):
    """Raised when the configuration cannot be loaded or used."""


# ---------------------------------------------------------------------------
# section view
# ---------------------------------------------------------------------------
class SectionView:
    """Read-only dot-path view of a sub-tree of the config.

    Useful for handing a robot- or component-scoped slice to code that
    shouldn't care about the rest of the file.
    """

    def __init__(self, name: str, data: Any):
        self._name = name
        self._data = data

    @property
    def name(self) -> str:
        return self._name

    def get(self, path: str = "", default: Any = None) -> Any:
        """Return ``self._data`` (path="") or a nested value via dot path."""
        if path == "":
            return self._data
        node: Any = self._data
        for k in path.split("."):
            if not isinstance(node, dict) or k not in node:
                return default
            node = node[k]
        return node

    def has(self, path: str) -> bool:
        sentinel = object()
        return self.get(path, sentinel) is not sentinel

    def section(self, path: str) -> "SectionView":
        sub = self.get(path)
        if not isinstance(sub, dict):
            raise ConfigError(
                f"section '{self._name}.{path}' does not exist or is not a mapping")
        return SectionView(f"{self._name}.{path}" if self._name else path, sub)

    def as_dict(self) -> Dict[str, Any]:
        return dict(self._data) if isinstance(self._data, dict) else {"value": self._data}

    def __getitem__(self, key: str) -> Any:
        if not isinstance(self._data, dict) or key not in self._data:
            raise KeyError(f"{self._name}.{key}")
        return self._data[key]

    def __contains__(self, key: str) -> bool:
        return isinstance(self._data, dict) and key in self._data

    def __repr__(self) -> str:
        return f"SectionView({self._name!r})"


# ---------------------------------------------------------------------------
# config manager
# ---------------------------------------------------------------------------
class ConfigManager:
    """Singleton loader for ``config/robot_config.yaml``."""

    _instance: Optional["ConfigManager"] = None
    _lock = threading.Lock()

    # Filenames probed under ``<project_root>/config/``, in priority order.
    _CONFIG_FILENAMES = ("robot_config.yaml", "robot_config.example.yaml")

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    inst = super().__new__(cls)
                    inst._initialized = False
                    cls._instance = inst
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return
        with self._lock:
            if self._initialized:
                return
            self._config: Dict[str, Any] = {}
            self._config_path: Optional[Path] = None
            self._project_root: Optional[Path] = None
            self._load()
            self._initialized = True

    # --- loading ----------------------------------------------------------
    def _resolve_config_path(self) -> Path:
        """Find robot_config.yaml (preferred) or its .example fallback."""
        # 1. explicit env override (ROBOT_CONFIG_PATH preferred;
        #    DUCO_CONTROL_CONFIG accepted as a legacy alias).
        env_path = os.environ.get("ROBOT_CONFIG_PATH") or \
            os.environ.get("DUCO_CONTROL_CONFIG")
        if env_path:
            p = Path(env_path).expanduser().resolve()
            if p.is_file():
                return p
            raise ConfigError(
                f"ROBOT_CONFIG_PATH points at {env_path!r} which does not exist")

        # 2. <project_root>/config/<filename>
        root = get_workspace_root()
        if root is not None:
            self._project_root = Path(root)
            for name in self._CONFIG_FILENAMES:
                p = self._project_root / "config" / name
                if p.is_file():
                    return p

        raise ConfigError(
            "no config/robot_config.yaml found. "
            "Copy config/robot_config.example.yaml to config/robot_config.yaml, "
            "or set ROBOT_CONFIG_PATH to an explicit path.")

    def _load(self) -> None:
        path = self._resolve_config_path()
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise ConfigError(f"failed to parse YAML in {path}: {e}") from e
        except OSError as e:
            raise ConfigError(f"failed to read {path}: {e}") from e

        if not isinstance(data, dict):
            raise ConfigError(
                f"top-level YAML in {path} must be a mapping, got {type(data).__name__}")

        data = self._expand_env(data)
        data = self._resolve_paths(data)

        self._config = data
        self._config_path = path

    def _expand_env(self, value: Any) -> Any:
        """Recursively replace ``${VAR}`` in string values."""
        pattern = re.compile(r"\$\{([^}]+)\}")
        if isinstance(value, str):
            return pattern.sub(lambda m: os.environ.get(m.group(1), m.group(0)), value)
        if isinstance(value, dict):
            return {k: self._expand_env(v) for k, v in value.items()}
        if isinstance(value, list):
            return [self._expand_env(v) for v in value]
        return value

    def _resolve_paths(self, data: Any) -> Any:
        """For any ``paths:`` mapping, turn relative strings into absolute
        paths under the project root."""
        root = self._project_root or Path.cwd()

        def fixup(node: Any) -> Any:
            if isinstance(node, dict):
                out: Dict[str, Any] = {}
                for k, v in node.items():
                    if k == "paths" and isinstance(v, dict):
                        out[k] = {
                            ik: (str((root / iv).resolve())
                                 if isinstance(iv, str) and not Path(iv).is_absolute()
                                 else iv)
                            for ik, iv in v.items()
                        }
                    else:
                        out[k] = fixup(v)
                return out
            if isinstance(node, list):
                return [fixup(v) for v in node]
            return node

        return fixup(data)

    # --- introspection ----------------------------------------------------
    @property
    def config_path(self) -> Optional[str]:
        return str(self._config_path) if self._config_path else None

    @property
    def project_root(self) -> Optional[str]:
        return str(self._project_root) if self._project_root else None

    def reload(self) -> None:
        """Re-read the config file (useful in dev/REPL)."""
        with self._lock:
            self._load()

    # --- access -----------------------------------------------------------
    def get(self, path: str, default: Any = None) -> Any:
        """Return a value by dot path; ``default`` if any segment is missing."""
        node: Any = self._config
        for k in path.split("."):
            if not isinstance(node, dict) or k not in node:
                return default
            node = node[k]
        return node

    def has(self, path: str) -> bool:
        sentinel = object()
        return self.get(path, sentinel) is not sentinel

    def section(self, path: str) -> SectionView:
        """Return a :class:`SectionView` rooted at ``path``."""
        sub = self.get(path)
        if not isinstance(sub, dict):
            raise ConfigError(
                f"section '{path}' does not exist or is not a mapping")
        return SectionView(path, sub)

    def list_sections(self) -> List[str]:
        """Top-level sections in the config (excluding the ``version`` key)."""
        return [k for k in self._config.keys() if k != "version"]

    def as_dict(self) -> Dict[str, Any]:
        """Return a shallow copy of the full config."""
        return dict(self._config)

    def __repr__(self) -> str:
        return (f"ConfigManager(path={self.config_path!r}, "
                f"sections={self.list_sections()})")


# ---------------------------------------------------------------------------
# convenience
# ---------------------------------------------------------------------------
def get_config() -> ConfigManager:
    """Return the (lazily initialised) :class:`ConfigManager` singleton."""
    return ConfigManager()


# ---------------------------------------------------------------------------
# write-back: aux_frames offsets
# ---------------------------------------------------------------------------
#
# ConfigManager itself is intentionally read-only; persisting back to
# the YAML while preserving the operator's comments and field order is
# out of scope for the singleton.  But the dashboard's "Tool frames"
# editor needs to update the xyz / rpy of named aux_frames entries on
# disk so the next bringup picks them up.  ``save_aux_frames`` performs
# a targeted, line-based rewrite that touches ONLY the matched ``xyz:``
# / ``rpy:`` lines of existing entries -- comments, blank lines, key
# order, and unrelated sections are preserved byte-for-byte.
#
# Adding, renaming, or removing aux_frames entries via this helper is
# explicitly out of scope (the dashboard does not expose those edits
# either): such changes are infrequent and operators edit the YAML by
# hand.
#
# The top-level YAML key that contains the ``aux_frames`` list is
# robot-specific (e.g. ``duco_robot_bringup`` for the Duco workspace,
# ``ur_robot_bringup`` for a UR workspace).  Callers pass it via the
# ``top_key`` argument; there is no default so the toolkit does not
# bake in any single robot's package name.
_AUX_LIST_KEY = "aux_frames"
_AUX_TRIPLE_RE = re.compile(
    r"^(?P<indent>\s+)(?P<key>xyz|rpy):\s*\[[^\]]*\]\s*(?P<trail>#.*)?$"
)
_AUX_ENTRY_RE = re.compile(
    r"^(?P<indent>\s*)-\s*name:\s*(?P<name>\S+)\s*(#.*)?$"
)
_AUX_LIST_RE = re.compile(rf"^\s+{re.escape(_AUX_LIST_KEY)}:\s*$")
# A top-level YAML key (non-indented, ends with ':').  We use this as
# the terminator for both the bringup section and the
# aux_frames list within it.
_TOP_LEVEL_KEY_RE = re.compile(r"^[A-Za-z_][\w.-]*:\s*(#.*)?$")


def _fmt_triple(values: Iterable[float]) -> str:
    """Render a 3-tuple of floats as ``[a, b, c]`` with canonical YAML
    floats (always has a decimal point).
    """
    parts = []
    for v in values:
        fv = float(v)
        # ``repr`` keeps the shortest round-trippable representation
        # AND always emits a decimal point for floats; 0 -> '0.0',
        # 0.1 -> '0.1', 1.5e-08 -> '1.5e-08'.  PyYAML accepts both
        # '0.0' and scientific form, so this is safe.
        parts.append(repr(fv))
    return "[" + ", ".join(parts) + "]"


def save_aux_frames(file_path: str,
                    frames: Dict[str, Dict[str, List[float]]],
                    top_key: str) -> int:
    """Update ``xyz`` / ``rpy`` of named entries under
    ``<top_key>.aux_frames`` in ``file_path``.

    Parameters
    ----------
    file_path:
        Absolute path to the YAML file to rewrite.  Must already contain
        the ``<top_key>`` section and an ``aux_frames`` list.
    frames:
        Mapping of ``name -> {"xyz": [x, y, z], "rpy": [r, p, y]}``.
        Keys not present in the file are skipped (and counted toward the
        return value as 0 updates).  Entries in the file whose name is
        not in this dict are left untouched.
    top_key:
        Name of the top-level YAML section that owns the
        ``aux_frames`` list (e.g. ``"duco_robot_bringup"``).

    Returns
    -------
    int
        Number of entries that had at least one of ``xyz`` / ``rpy``
        rewritten.

    Raises
    ------
    ConfigError
        If the file cannot be read, the ``<top_key>`` /
        ``aux_frames`` block is missing, or the write fails.

    Notes
    -----
    The rewrite is **atomic**: the new content is written to a temp
    file in the same directory and ``os.replace``\ d into place, so a
    concurrent reader either sees the old or new file but never a
    half-written one.
    """
    p = Path(file_path)
    try:
        text = p.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"failed to read {p}: {exc}") from exc

    lines = text.splitlines(keepends=True)

    section_re = re.compile(rf"^{re.escape(top_key)}:\s*$")

    # Locate the <top_key>: line, then aux_frames: within it.
    section_start = next(
        (i for i, ln in enumerate(lines) if section_re.match(ln)),
        None,
    )
    if section_start is None:
        raise ConfigError(
            f"{p}: no top-level '{top_key}:' section to update")

    section_end = len(lines)
    for j in range(section_start + 1, len(lines)):
        if _TOP_LEVEL_KEY_RE.match(lines[j]):
            section_end = j
            break

    aux_start = None
    for j in range(section_start + 1, section_end):
        if _AUX_LIST_RE.match(lines[j]):
            aux_start = j
            break
    if aux_start is None:
        raise ConfigError(
            f"{p}: no '{_AUX_LIST_KEY}:' list under '{top_key}:'")

    # Walk entries from aux_start+1 until the section ends or a
    # sibling key at the same indent as aux_frames: is hit.
    # We track which entry we're currently in by its name.
    aux_indent_match = re.match(r"^(\s+)", lines[aux_start])
    aux_indent = aux_indent_match.group(1) if aux_indent_match else "  "

    updated_per_name: Dict[str, int] = {n: 0 for n in frames.keys()}
    current_name: Optional[str] = None
    out_lines = list(lines)

    i = aux_start + 1
    while i < section_end:
        ln = out_lines[i]
        # Sibling key at the aux_frames indent terminates the list.
        sib = re.match(rf"^{aux_indent}([A-Za-z_][\w.-]*):", ln)
        if sib and not ln.lstrip().startswith("-"):
            break
        entry = _AUX_ENTRY_RE.match(ln)
        if entry:
            current_name = entry.group("name").strip()
            i += 1
            continue
        if current_name and current_name in frames:
            m = _AUX_TRIPLE_RE.match(ln)
            if m:
                key = m.group("key")
                target = frames[current_name].get(key)
                if target is not None and len(list(target)) == 3:
                    indent = m.group("indent")
                    trail = m.group("trail") or ""
                    trail_sep = "  " + trail if trail else ""
                    new_line = (f"{indent}{key}: "
                                f"{_fmt_triple(target)}"
                                f"{trail_sep}\n")
                    if out_lines[i] != new_line:
                        out_lines[i] = new_line
                        updated_per_name[current_name] = (
                            updated_per_name[current_name] | 1
                            if key == "xyz"
                            else updated_per_name[current_name] | 2)
        i += 1

    updated_count = sum(1 for v in updated_per_name.values() if v)

    new_text = "".join(out_lines)
    if new_text == text:
        return 0  # nothing changed; skip the write

    # Atomic write: temp file in same directory, then rename.
    fd, tmp_path = tempfile.mkstemp(
        prefix=p.name + ".", suffix=".tmp", dir=str(p.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(new_text)
        os.replace(tmp_path, p)
    except OSError as exc:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise ConfigError(f"failed to write {p}: {exc}") from exc

    return updated_count


def read_aux_frames(file_path: str, top_key: str) -> List[Dict[str, Any]]:
    """Read aux_frames entries directly from ``file_path`` via PyYAML.

    Returns the raw list as currently on disk (each entry is the dict
    YAML parsed from the file).  Returns ``[]`` if the section is
    missing.  Unlike :func:`get_config`, this does NOT go through the
    singleton's cache, so it reflects pending on-disk edits.

    Parameters
    ----------
    file_path:
        Absolute path to the YAML file to read.
    top_key:
        Name of the top-level YAML section that owns the
        ``aux_frames`` list (e.g. ``"duco_robot_bringup"``).
    """
    p = Path(file_path)
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"failed to parse YAML in {p}: {exc}") from exc
    except OSError as exc:
        raise ConfigError(f"failed to read {p}: {exc}") from exc
    if not isinstance(data, dict):
        return []
    section = data.get(top_key)
    if not isinstance(section, dict):
        return []
    frames = section.get(_AUX_LIST_KEY)
    if not isinstance(frames, list):
        return []
    return [f for f in frames if isinstance(f, dict)]
