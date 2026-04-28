"""Forward kinematics helper for the Duco GCR5-910 dashboard."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple


Matrix = List[List[float]]


def _identity() -> Matrix:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matmul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[row][k] * b[k][col] for k in range(4))
             for col in range(4)] for row in range(4)]


def _transform(rpy: Sequence[float], xyz: Sequence[float]) -> Matrix:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # URDF fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    matrix = _identity()
    for row in range(3):
        for col in range(3):
            matrix[row][col] = rotation[row][col]
        matrix[row][3] = xyz[row]
    return matrix


def _rot_axis(axis: Sequence[float], angle: float) -> Matrix:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        return _identity()
    x, y, z = x / norm, y / norm, z / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    matrix = _identity()
    matrix[0][0] = c + x * x * one_minus_c
    matrix[0][1] = x * y * one_minus_c - z * s
    matrix[0][2] = x * z * one_minus_c + y * s
    matrix[1][0] = y * x * one_minus_c + z * s
    matrix[1][1] = c + y * y * one_minus_c
    matrix[1][2] = y * z * one_minus_c - x * s
    matrix[2][0] = z * x * one_minus_c - y * s
    matrix[2][1] = z * y * one_minus_c + x * s
    matrix[2][2] = c + z * z * one_minus_c
    return matrix


_JOINTS: Tuple[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.122), (0.0, 0.0, 1.0)),
    ((1.5708, -1.5708, 3.1416), (0.0, 0.1445, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.425, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 1.5708), (0.392, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((1.5708, 0.0, 0.0), (0.0, -0.1, 0.0), (0.0, 0.0, 1.0)),
    ((-1.5708, 0.0, 0.0), (0.0, 0.105, 0.0), (0.0, 0.0, 1.0)),
)


def joint_frames(joint_angles: Iterable[float]) -> List[Matrix]:
    """Return base-to-joint-frame transforms for each commanded joint."""
    angles = list(joint_angles)
    if len(angles) != 6:
        raise ValueError(f'expected 6 joint angles, got {len(angles)}')
    transform = _identity()
    frames: List[Matrix] = []
    for angle, (rpy, xyz, axis) in zip(angles, _JOINTS):
        transform = _matmul(transform, _transform(rpy, xyz))
        transform = _matmul(transform, _rot_axis(axis, angle))
        frames.append(transform)
    return frames


def forward_kinematics(joint_angles: Iterable[float]) -> Matrix:
    """Return the approximate base_link-to-link_6 transform."""
    return joint_frames(joint_angles)[-1]


def rotation_to_rpy(matrix: Matrix) -> Tuple[float, float, float]:
    """Convert a homogeneous transform rotation matrix to roll, pitch, yaw."""
    r00 = matrix[0][0]
    r10 = matrix[1][0]
    r20 = matrix[2][0]
    r21 = matrix[2][1]
    r22 = matrix[2][2]
    r11 = matrix[1][1]
    r12 = matrix[1][2]
    sy = math.sqrt(r00 * r00 + r10 * r10)
    if sy > 1e-9:
        return math.atan2(r21, r22), math.atan2(-r20, sy), math.atan2(r10, r00)
    return math.atan2(-r12, r11), math.atan2(-r20, sy), 0.0