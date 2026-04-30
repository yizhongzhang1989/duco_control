"""Gravity-compensation math for a 6-DoF F/T sensor.

This module is intentionally ROS-free so the math can be unit-tested and
reasoned about in isolation. The :class:`CompensationParams` dataclass holds
the per-end-effector calibration; :func:`compensate` applies it; and
:func:`calibrate_least_squares` recovers it from samples taken at varied
orientations.

Conventions
-----------
* The sensor frame is some frame ``S`` whose orientation in the world frame
  ``W`` is known at every measurement (we ask /tf for it).
* Gravity in the world frame is ``g_world = (0, 0, -|g|)``.
* The sensor reports the wrench it experiences in its own frame, so the
  gravitational contribution from a tool of mass ``m`` and center of mass
  ``r_com`` (expressed in the sensor frame) is:

      F_grav_S = m * R_S_W^T @ g_world
      T_grav_S = r_com x F_grav_S

* ``F_bias`` and ``T_bias`` are constant offsets (in the sensor frame) that
  remain after gravity compensation; they absorb sensor zero drift,
  sensor-side hardware mounting forces, and any constant unmodelled
  effects.

Compensated wrench
------------------
::

    F_comp = F_raw - F_bias - F_grav_S
    T_comp = T_raw - T_bias - T_grav_S

Calibration
-----------
Given ``N >= 3`` samples ``(R_i, F_obs_i, T_obs_i)`` with ``R_i`` the rotation
of the sensor frame in the world frame, the parameters are recovered in two
linear least-squares solves:

* For ``F``: stack ``[I_3 | g_S_i] [F_bias; m]^T = F_obs_i`` -> 4 unknowns.
* For ``T``: stack ``[I_3 | -[g_S_i]_x] [T_bias; m * r_com]^T = T_obs_i`` ->
  6 unknowns. Then ``r_com = (m * r_com) / m``.

Sample orientations must span at least three non-collinear gravity
directions in the sensor frame, otherwise the system is rank-deficient and
mass / CoM are not identifiable. The solver reports the condition number so
the caller can warn the user.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np


# Standard gravity (m/s^2). 9.80665 is the ISO standard.
DEFAULT_GRAVITY = 9.80665


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def skew(v: Sequence[float]) -> np.ndarray:
    """Return the 3x3 cross-product matrix ``[v]_x`` so that ``[v]_x @ u = v x u``."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def quat_to_rotation(q: Sequence[float]) -> np.ndarray:
    """Convert a quaternion ``(x, y, z, w)`` (ROS order) to a 3x3 rotation matrix."""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = s * x * x, s * y * y, s * z * z
    xy, xz, yz = s * x * y, s * x * z, s * y * z
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


# ---------------------------------------------------------------------------
# parameter container
# ---------------------------------------------------------------------------
@dataclass
class CompensationParams:
    """Per-end-effector calibration.

    Attributes
    ----------
    mass : float
        Tool mass in kg.
    com : numpy.ndarray
        3-vector: tool centre of mass expressed in the sensor frame, in metres.
    force_bias : numpy.ndarray
        3-vector: residual force offset (sensor frame, N).
    torque_bias : numpy.ndarray
        3-vector: residual torque offset (sensor frame, N*m).
    gravity : float
        Magnitude of gravitational acceleration (m/s^2). Default: 9.80665.
    """

    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    force_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    torque_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gravity: float = DEFAULT_GRAVITY

    @classmethod
    def zero(cls) -> "CompensationParams":
        return cls()

    def as_dict(self) -> dict:
        return {
            "mass": float(self.mass),
            "com": [float(v) for v in self.com],
            "force_bias": [float(v) for v in self.force_bias],
            "torque_bias": [float(v) for v in self.torque_bias],
            "gravity": float(self.gravity),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "CompensationParams":
        return cls(
            mass=float(data.get("mass", 0.0)),
            com=np.asarray(data.get("com", [0.0, 0.0, 0.0]), dtype=float).reshape(3),
            force_bias=np.asarray(
                data.get("force_bias", [0.0, 0.0, 0.0]), dtype=float).reshape(3),
            torque_bias=np.asarray(
                data.get("torque_bias", [0.0, 0.0, 0.0]), dtype=float).reshape(3),
            gravity=float(data.get("gravity", DEFAULT_GRAVITY)),
        )


# ---------------------------------------------------------------------------
# core math
# ---------------------------------------------------------------------------
def gravity_in_sensor(R_sensor_in_world: np.ndarray, gravity: float) -> np.ndarray:
    """Return ``g`` expressed in the sensor frame (3-vector, m/s^2).

    ``R_sensor_in_world`` is the 3x3 rotation matrix of the sensor frame as
    seen from the world frame (i.e. ``v_world = R @ v_sensor``).
    """
    g_world = np.array([0.0, 0.0, -float(gravity)])
    return R_sensor_in_world.T @ g_world


def gravity_wrench(R_sensor_in_world: np.ndarray,
                   params: CompensationParams) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(F_grav, T_grav)`` -- the wrench gravity exerts in the sensor frame."""
    g_s = gravity_in_sensor(R_sensor_in_world, params.gravity)
    f = params.mass * g_s
    t = np.cross(params.com, f)
    return f, t


def compensate(force_raw: Sequence[float], torque_raw: Sequence[float],
               R_sensor_in_world: np.ndarray,
               params: CompensationParams) -> Tuple[np.ndarray, np.ndarray]:
    """Compute ``(F_compensated, T_compensated)`` for a single sample."""
    f_raw = np.asarray(force_raw, dtype=float).reshape(3)
    t_raw = np.asarray(torque_raw, dtype=float).reshape(3)
    f_grav, t_grav = gravity_wrench(R_sensor_in_world, params)
    f = f_raw - params.force_bias - f_grav
    t = t_raw - params.torque_bias - t_grav
    return f, t


# ---------------------------------------------------------------------------
# calibration
# ---------------------------------------------------------------------------
@dataclass
class CalibrationResult:
    params: CompensationParams
    rms_force: float        # residual force RMS over the input samples (N)
    rms_torque: float       # residual torque RMS (N*m)
    n_samples: int
    cond_force: float       # condition number of the force LSQ matrix
    cond_torque: float      # condition number of the torque LSQ matrix
    warnings: List[str]


def calibrate_least_squares(
    samples: Sequence[Tuple[np.ndarray, Sequence[float], Sequence[float]]],
    *, gravity: float = DEFAULT_GRAVITY,
) -> CalibrationResult:
    """Fit ``CompensationParams`` from observation samples.

    Parameters
    ----------
    samples : sequence of ``(R_sensor_in_world, F_obs, T_obs)``
        ``R`` is a 3x3 rotation matrix; ``F_obs`` and ``T_obs`` are 3-vectors
        of the observed wrench (in the sensor frame).
    gravity : float, optional
        Magnitude of gravitational acceleration (m/s^2).

    Returns
    -------
    CalibrationResult
    """
    n = len(samples)
    warnings: List[str] = []
    if n < 3:
        warnings.append(
            f"only {n} sample(s) provided; >=3 distinct orientations recommended")

    A_F = np.zeros((3 * n, 4))      # cols: bx, by, bz, m
    b_F = np.zeros(3 * n)
    A_T = np.zeros((3 * n, 6))      # cols: tbx, tby, tbz, cx, cy, cz   (c = m*r_com)
    b_T = np.zeros(3 * n)
    g_world = np.array([0.0, 0.0, -float(gravity)])

    for i, (R, F_obs, T_obs) in enumerate(samples):
        R = np.asarray(R, dtype=float).reshape(3, 3)
        f = np.asarray(F_obs, dtype=float).reshape(3)
        t = np.asarray(T_obs, dtype=float).reshape(3)
        g_s = R.T @ g_world

        A_F[3 * i:3 * i + 3, 0:3] = np.eye(3)
        A_F[3 * i:3 * i + 3, 3] = g_s
        b_F[3 * i:3 * i + 3] = f

        A_T[3 * i:3 * i + 3, 0:3] = np.eye(3)
        A_T[3 * i:3 * i + 3, 3:6] = -skew(g_s)
        b_T[3 * i:3 * i + 3] = t

    sol_F, _, rank_F, sing_F = np.linalg.lstsq(A_F, b_F, rcond=None)
    sol_T, _, rank_T, sing_T = np.linalg.lstsq(A_T, b_T, rcond=None)

    cond_F = float(_safe_cond(sing_F))
    cond_T = float(_safe_cond(sing_T))

    force_bias = sol_F[0:3]
    mass = float(sol_F[3])
    torque_bias = sol_T[0:3]
    c_vec = sol_T[3:6]

    if abs(mass) < 1e-6:
        com = np.zeros(3)
        warnings.append(
            "fitted mass is ~0; CoM not identifiable -- record samples in "
            "more varied orientations or check that gravity is reaching the sensor")
    else:
        com = c_vec / mass

    if mass < 0.0:
        warnings.append(
            f"fitted mass is negative ({mass:+.4f} kg); the sensor's force "
            "sign convention may be inverted relative to the model -- "
            "the magnitudes are still usable for compensation")

    if rank_F < 4:
        warnings.append(
            f"force-equation matrix is rank-deficient ({rank_F}/4); "
            "samples do not span enough orientations to identify mass")
    if rank_T < 6:
        warnings.append(
            f"torque-equation matrix is rank-deficient ({rank_T}/6); "
            "samples do not span enough orientations to identify CoM")

    params = CompensationParams(
        mass=mass, com=com,
        force_bias=force_bias, torque_bias=torque_bias,
        gravity=float(gravity),
    )

    rms_force, rms_torque = _residual_rms(samples, params)

    return CalibrationResult(
        params=params,
        rms_force=rms_force, rms_torque=rms_torque,
        n_samples=n,
        cond_force=cond_F, cond_torque=cond_T,
        warnings=warnings,
    )


def _safe_cond(singular_values: np.ndarray) -> float:
    if singular_values is None or len(singular_values) == 0:
        return float("inf")
    smin = float(np.min(singular_values))
    smax = float(np.max(singular_values))
    if smin <= 0.0:
        return float("inf")
    return smax / smin


def _residual_rms(samples, params: CompensationParams) -> Tuple[float, float]:
    if not samples:
        return 0.0, 0.0
    f_sq = 0.0
    t_sq = 0.0
    for R, F_obs, T_obs in samples:
        R = np.asarray(R, dtype=float).reshape(3, 3)
        f_pred, t_pred = gravity_wrench(R, params)
        f_pred = f_pred + params.force_bias
        t_pred = t_pred + params.torque_bias
        f_err = np.asarray(F_obs, dtype=float).reshape(3) - f_pred
        t_err = np.asarray(T_obs, dtype=float).reshape(3) - t_pred
        f_sq += float(f_err @ f_err)
        t_sq += float(t_err @ t_err)
    n = len(samples)
    return (np.sqrt(f_sq / (3 * n)), np.sqrt(t_sq / (3 * n)))
