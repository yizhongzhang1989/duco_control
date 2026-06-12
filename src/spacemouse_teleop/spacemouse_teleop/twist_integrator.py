"""Pure-math twist -> pose integrator for SpaceMouse Cartesian jogging.

ROS-free: depends only on ``numpy`` so it can be unit-tested offline without a
running ROS graph or a robot. The :mod:`servo_node` ROS layer feeds live
SpaceMouse twists + the captured end-effector pose into these functions.

Quaternions are ``(x, y, z, w)`` to match ``geometry_msgs/Quaternion``.
"""
from __future__ import annotations

import numpy as np

__all__ = [
    "quat_normalize",
    "quat_mul",
    "quat_from_rotvec",
    "quat_to_rotmat",
    "deadband",
    "clamp_norm",
    "shape_twist",
    "integrate_pose",
]


def quat_normalize(q) -> np.ndarray:
    """Return ``q`` normalised to unit length; identity if (near) zero."""
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_mul(a, b) -> np.ndarray:
    """Hamilton product ``a (x) b`` for ``(x, y, z, w)`` quaternions."""
    ax, ay, az, aw = (float(v) for v in a)
    bx, by, bz, bw = (float(v) for v in b)
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_from_rotvec(r) -> np.ndarray:
    """Quaternion ``(x, y, z, w)`` from a rotation vector (axis * angle)."""
    r = np.asarray(r, dtype=float)
    angle = float(np.linalg.norm(r))
    if angle < 1e-9:
        # First-order approximation for tiny angles, then renormalise.
        return quat_normalize(np.array([r[0] / 2.0, r[1] / 2.0, r[2] / 2.0, 1.0]))
    axis = r / angle
    s = np.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle / 2.0)])


def quat_to_rotmat(q) -> np.ndarray:
    """3x3 rotation matrix R (body->world) for a ``(x, y, z, w)`` quaternion."""
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def deadband(v, threshold) -> np.ndarray:
    """Zero each component of ``v`` whose magnitude is below ``threshold``."""
    v = np.asarray(v, dtype=float).copy()
    if threshold <= 0.0:
        return v
    v[np.abs(v) < float(threshold)] = 0.0
    return v


def clamp_norm(v, max_norm) -> np.ndarray:
    """Scale vector ``v`` down so ``||v|| <= max_norm`` (direction preserved)."""
    v = np.asarray(v, dtype=float)
    if max_norm is None or max_norm <= 0.0:
        return v
    n = float(np.linalg.norm(v))
    if n > float(max_norm):
        return v * (float(max_norm) / n)
    return v


def shape_twist(lin, ang, lin_scale, ang_scale,
                deadband_lin=0.0, deadband_ang=0.0,
                max_lin=None, max_ang=None):
    """Apply deadband, per-axis scale, then a magnitude clamp to a raw twist.

    Returns ``(v_lin, v_ang)`` as physical velocities (m/s, rad/s).
    """
    lin = deadband(lin, deadband_lin) * np.asarray(lin_scale, dtype=float)
    ang = deadband(ang, deadband_ang) * np.asarray(ang_scale, dtype=float)
    lin = clamp_norm(lin, max_lin)
    ang = clamp_norm(ang, max_ang)
    return lin, ang


def integrate_pose(position, orientation, v_lin, v_ang, dt, frame="tool"):
    """Integrate a pose by one step of a Cartesian velocity.

    Parameters
    ----------
    position : (3,) current position in the base frame.
    orientation : (4,) current orientation quaternion (x, y, z, w) in the base frame.
    v_lin : (3,) linear velocity (m/s).
    v_ang : (3,) angular velocity (rad/s).
    dt : timestep (s).
    frame : ``"tool"`` (velocities expressed in the moving body frame) or
        ``"base"`` (velocities expressed in the fixed base frame).

    Returns
    -------
    (new_position (3,), new_orientation (4,)) with the orientation renormalised.
    """
    position = np.asarray(position, dtype=float).copy()
    orientation = quat_normalize(orientation)
    v_lin = np.asarray(v_lin, dtype=float)
    v_ang = np.asarray(v_ang, dtype=float)
    dp = v_lin * float(dt)
    dq = quat_from_rotvec(v_ang * float(dt))

    if frame == "tool":
        rot = quat_to_rotmat(orientation)
        new_position = position + rot @ dp
        # Body-frame increment -> right-multiply.
        new_orientation = quat_normalize(quat_mul(orientation, dq))
    elif frame == "base":
        new_position = position + dp
        # World-frame increment -> left-multiply.
        new_orientation = quat_normalize(quat_mul(dq, orientation))
    else:
        raise ValueError("frame must be 'tool' or 'base', got %r" % (frame,))
    return new_position, new_orientation
