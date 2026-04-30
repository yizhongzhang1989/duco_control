"""Centralized configuration loader for the duco_control project.

Reads ``config/robot_config.yaml`` (or, if missing, falls back to
``config/robot_config.example.yaml``) once and caches it in a thread-safe
singleton. Values are accessed by dot-path strings.

Typical use::

    from common.config_manager import get_config

    cfg = get_config()
    ip      = cfg.get("duco.network.ip", "192.168.1.10")
    port    = cfg.get("duco.ft_sensor.port", "/dev/ttyUSB0")
    web_port = cfg.get("dashboards.ft_sensor.port", 8080)

There is also :class:`SectionView` for scoped access::

    ft = cfg.section("duco.ft_sensor")
    print(ft.get("port"))     # "/dev/ttyUSB0"
    print(ft.get("baud", 0))  # 460800

Strings in the YAML may use ``${ENV_VAR}`` to reference environment
variables (left as-is if the variable is unset). Any string value under
a ``paths`` key whose value is a relative path is resolved against the
project root.
"""

from __future__ import annotations

import os
import re
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

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
        # 1. explicit env override
        env_path = os.environ.get("DUCO_CONTROL_CONFIG")
        if env_path:
            p = Path(env_path).expanduser().resolve()
            if p.is_file():
                return p
            raise ConfigError(
                f"DUCO_CONTROL_CONFIG points at {env_path!r} which does not exist")

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
            "or set DUCO_CONTROL_CONFIG to an explicit path.")

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
