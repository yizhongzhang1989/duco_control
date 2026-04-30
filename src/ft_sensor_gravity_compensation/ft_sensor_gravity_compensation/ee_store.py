"""Persistent storage for end-effector calibration profiles.

The store keeps a YAML file that holds:

* ``active``         -- name of the currently active end-effector
* ``end_effectors``  -- mapping from name to a profile dict containing the
                         calibrated CompensationParams *plus* the recorded
                         samples (so the user can recompute or extend a
                         calibration later without re-recording everything).

Thread-safety: a single :class:`EndEffectorStore` instance serialises all
mutations through an internal lock, so the dashboard's HTTP threads and the
ROS callback threads can both touch it concurrently.

This module is ROS-free and can be used / tested standalone.
"""

from __future__ import annotations

import os
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

from .calibration import CompensationParams


_DEFAULT_NAME = "default"


@dataclass
class Sample:
    """One recorded calibration observation (sensor-frame wrench + sensor pose)."""
    rotation: List[List[float]]   # 3x3 rotation: sensor frame in world frame
    force: List[float]            # raw force, sensor frame, N
    torque: List[float]           # raw torque, sensor frame, N*m
    stamp: float                  # wall-clock seconds when captured (informational)
    note: str = ""

    def as_dict(self) -> dict:
        return {
            "rotation": [[float(v) for v in row] for row in self.rotation],
            "force": [float(v) for v in self.force],
            "torque": [float(v) for v in self.torque],
            "stamp": float(self.stamp),
            "note": str(self.note),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Sample":
        return cls(
            rotation=[list(map(float, row)) for row in data.get("rotation", [])],
            force=list(map(float, data.get("force", [0.0, 0.0, 0.0]))),
            torque=list(map(float, data.get("torque", [0.0, 0.0, 0.0]))),
            stamp=float(data.get("stamp", 0.0)),
            note=str(data.get("note", "")),
        )


@dataclass
class EndEffector:
    """A named end-effector profile."""
    name: str
    description: str = ""
    params: CompensationParams = field(default_factory=CompensationParams.zero)
    samples: List[Sample] = field(default_factory=list)
    last_calibration: Optional[dict] = None  # diagnostic info from the last fit

    def as_dict(self) -> dict:
        out = {
            "description": self.description,
            **self.params.as_dict(),
            "samples": [s.as_dict() for s in self.samples],
        }
        if self.last_calibration is not None:
            out["last_calibration"] = self.last_calibration
        return out

    @classmethod
    def from_dict(cls, name: str, data: dict) -> "EndEffector":
        return cls(
            name=name,
            description=str(data.get("description", "")),
            params=CompensationParams.from_dict(data),
            samples=[Sample.from_dict(s) for s in data.get("samples", [])],
            last_calibration=data.get("last_calibration"),
        )


class EndEffectorStore:
    """File-backed, lock-protected collection of :class:`EndEffector` profiles."""

    def __init__(self, path: str):
        self._path = Path(os.path.expanduser(path)).resolve()
        self._lock = threading.RLock()
        self._effectors: Dict[str, EndEffector] = {}
        self._active: str = _DEFAULT_NAME
        self._load()

    # --- introspection ----------------------------------------------------
    @property
    def path(self) -> Path:
        return self._path

    @property
    def active_name(self) -> str:
        with self._lock:
            return self._active

    def names(self) -> List[str]:
        with self._lock:
            return sorted(self._effectors.keys())

    def get(self, name: str) -> Optional[EndEffector]:
        with self._lock:
            return self._effectors.get(name)

    def active(self) -> EndEffector:
        with self._lock:
            return self._effectors[self._active]

    def snapshot(self) -> dict:
        """Return a JSON-serialisable summary used by the dashboard UI."""
        with self._lock:
            return {
                "path": str(self._path),
                "active": self._active,
                "end_effectors": {
                    name: ee.as_dict() for name, ee in self._effectors.items()
                },
            }

    # --- mutators ---------------------------------------------------------
    def create(self, name: str, description: str = "") -> EndEffector:
        name = _validate_name(name)
        with self._lock:
            if name in self._effectors:
                raise ValueError(f"end-effector '{name}' already exists")
            ee = EndEffector(name=name, description=description)
            self._effectors[name] = ee
            self._save_locked()
            return ee

    def delete(self, name: str) -> None:
        with self._lock:
            if name == _DEFAULT_NAME:
                raise ValueError("the 'default' end-effector cannot be deleted")
            if name not in self._effectors:
                raise KeyError(f"unknown end-effector '{name}'")
            del self._effectors[name]
            if self._active == name:
                self._active = _DEFAULT_NAME
            self._save_locked()

    def rename(self, old_name: str, new_name: str) -> None:
        new_name = _validate_name(new_name)
        with self._lock:
            if old_name == _DEFAULT_NAME:
                raise ValueError("the 'default' end-effector cannot be renamed")
            if old_name not in self._effectors:
                raise KeyError(f"unknown end-effector '{old_name}'")
            if new_name in self._effectors:
                raise ValueError(f"end-effector '{new_name}' already exists")
            ee = self._effectors.pop(old_name)
            ee.name = new_name
            self._effectors[new_name] = ee
            if self._active == old_name:
                self._active = new_name
            self._save_locked()

    def set_description(self, name: str, description: str) -> None:
        with self._lock:
            ee = self._require(name)
            ee.description = description
            self._save_locked()

    def set_active(self, name: str) -> EndEffector:
        with self._lock:
            ee = self._require(name)
            self._active = name
            self._save_locked()
            return ee

    def set_params(self, name: str, params: CompensationParams,
                   diag: Optional[dict] = None) -> None:
        with self._lock:
            ee = self._require(name)
            ee.params = params
            if diag is not None:
                ee.last_calibration = dict(diag)
                ee.last_calibration["timestamp"] = time.time()
            self._save_locked()

    def add_sample(self, name: str, sample: Sample) -> EndEffector:
        with self._lock:
            ee = self._require(name)
            ee.samples.append(sample)
            self._save_locked()
            return ee

    def delete_sample(self, name: str, index: int) -> None:
        with self._lock:
            ee = self._require(name)
            if not (0 <= index < len(ee.samples)):
                raise IndexError(f"sample index {index} out of range "
                                 f"(end-effector '{name}' has {len(ee.samples)})")
            ee.samples.pop(index)
            self._save_locked()

    def clear_samples(self, name: str) -> None:
        with self._lock:
            ee = self._require(name)
            ee.samples.clear()
            self._save_locked()

    # --- internals --------------------------------------------------------
    def _require(self, name: str) -> EndEffector:
        if name not in self._effectors:
            raise KeyError(f"unknown end-effector '{name}'")
        return self._effectors[name]

    def _load(self) -> None:
        if self._path.is_file():
            try:
                with open(self._path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
            except (OSError, yaml.YAMLError) as exc:
                raise RuntimeError(
                    f"could not read end-effector store at {self._path}: {exc}") from exc
        else:
            data = {}

        ees = data.get("end_effectors") or {}
        if not isinstance(ees, dict):
            raise RuntimeError(
                f"malformed end-effector store at {self._path}: "
                "'end_effectors' must be a mapping")

        for name, raw in ees.items():
            if not isinstance(raw, dict):
                continue
            self._effectors[str(name)] = EndEffector.from_dict(str(name), raw)

        # Always have a 'default' entry that means "no compensation".
        if _DEFAULT_NAME not in self._effectors:
            self._effectors[_DEFAULT_NAME] = EndEffector(
                name=_DEFAULT_NAME,
                description="zero mass / zero bias (no compensation)",
            )

        active = str(data.get("active", _DEFAULT_NAME))
        if active not in self._effectors:
            active = _DEFAULT_NAME
        self._active = active

        if not self._path.is_file():
            # Persist the default entry on first run so the file exists for
            # subsequent edits.
            self._save_locked()

    def _save_locked(self) -> None:
        payload = {
            "version": 1,
            "active": self._active,
            "end_effectors": {
                name: ee.as_dict() for name, ee in self._effectors.items()
            },
        }
        self._path.parent.mkdir(parents=True, exist_ok=True)
        # atomic write: temp + rename (same dir to keep it on the same FS).
        tmp = tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", delete=False,
            dir=str(self._path.parent), prefix=".end_effectors_", suffix=".tmp")
        try:
            yaml.safe_dump(payload, tmp, sort_keys=False)
            tmp.flush()
            os.fsync(tmp.fileno())
        finally:
            tmp.close()
        os.replace(tmp.name, self._path)


def _validate_name(name: str) -> str:
    name = str(name).strip()
    if not name:
        raise ValueError("end-effector name must not be empty")
    if any(ch in name for ch in "/\\\n\r\t"):
        raise ValueError(f"invalid character in end-effector name: {name!r}")
    if len(name) > 64:
        raise ValueError("end-effector name too long (max 64 chars)")
    return name
