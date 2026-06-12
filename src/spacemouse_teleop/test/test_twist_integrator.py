"""Offline unit tests for the ROS-free twist -> pose integrator.

Run without ROS::

    PYTHONPATH=external/3dconnexion_ros2/spacemouse_teleop \
        python3 -m pytest external/3dconnexion_ros2/spacemouse_teleop/test/test_twist_integrator.py -q
"""
import numpy as np

from spacemouse_teleop.twist_integrator import (
    clamp_norm,
    deadband,
    integrate_pose,
    quat_from_rotvec,
    quat_mul,
    quat_normalize,
    quat_to_rotmat,
    shape_twist,
)

IDENTITY = np.array([0.0, 0.0, 0.0, 1.0])


def test_quat_normalize_unit_and_zero():
    np.testing.assert_allclose(quat_normalize(IDENTITY), IDENTITY)
    np.testing.assert_allclose(quat_normalize([0, 0, 0, 0]), IDENTITY)
    np.testing.assert_allclose(quat_normalize([0, 0, 0, 5]), IDENTITY)


def test_quat_mul_identity():
    q = quat_normalize([0.1, -0.2, 0.3, 0.9])
    np.testing.assert_allclose(quat_mul(IDENTITY, q), q, atol=1e-12)
    np.testing.assert_allclose(quat_mul(q, IDENTITY), q, atol=1e-12)


def test_quat_from_rotvec_zero_is_identity():
    np.testing.assert_allclose(quat_from_rotvec([0, 0, 0]), IDENTITY)


def test_quat_from_rotvec_90_about_z():
    q = quat_from_rotvec([0, 0, np.pi / 2])
    s = np.sqrt(0.5)
    np.testing.assert_allclose(q, [0, 0, s, s], atol=1e-9)


def test_quat_to_rotmat_90_about_z():
    q = quat_from_rotvec([0, 0, np.pi / 2])
    rot = quat_to_rotmat(q)
    # +x rotated +90deg about z -> +y
    np.testing.assert_allclose(rot @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-9)


def test_deadband_zeros_small_components():
    np.testing.assert_allclose(deadband([0.05, 0.2, -0.3], 0.1), [0.0, 0.2, -0.3])
    np.testing.assert_allclose(deadband([0.05, 0.2, -0.3], 0.0), [0.05, 0.2, -0.3])


def test_clamp_norm_scales_down_preserving_direction():
    v = clamp_norm([3.0, 4.0, 0.0], 1.0)   # norm 5 -> 1
    np.testing.assert_allclose(np.linalg.norm(v), 1.0, atol=1e-12)
    np.testing.assert_allclose(v, [0.6, 0.8, 0.0], atol=1e-12)
    # under the cap -> unchanged
    np.testing.assert_allclose(clamp_norm([0.3, 0.0, 0.0], 1.0), [0.3, 0, 0])


def test_shape_twist_applies_scale_and_clamp():
    lin, ang = shape_twist(
        [1.0, 0.0, 0.0], [0.0, 0.0, 1.0],
        lin_scale=[0.1, 0.1, 0.1], ang_scale=[0.5, 0.5, 0.5],
        max_lin=0.05, max_ang=10.0)
    np.testing.assert_allclose(lin, [0.05, 0, 0], atol=1e-12)   # 0.1 clamped to 0.05
    np.testing.assert_allclose(ang, [0, 0, 0.5], atol=1e-12)


def test_integrate_base_frame_pure_translation():
    pos, quat = integrate_pose(
        [0, 0, 0], IDENTITY, v_lin=[1.0, 0, 0], v_ang=[0, 0, 0],
        dt=0.5, frame="base")
    np.testing.assert_allclose(pos, [0.5, 0, 0], atol=1e-12)
    np.testing.assert_allclose(quat, IDENTITY, atol=1e-12)


def test_integrate_tool_frame_translation_with_rotated_orientation():
    # Tool rotated +90deg about z: a +x tool velocity moves +y in base.
    q = quat_from_rotvec([0, 0, np.pi / 2])
    pos, quat = integrate_pose(
        [1.0, 2.0, 3.0], q, v_lin=[1.0, 0, 0], v_ang=[0, 0, 0],
        dt=0.2, frame="tool")
    np.testing.assert_allclose(pos, [1.0, 2.2, 3.0], atol=1e-9)
    np.testing.assert_allclose(quat, q, atol=1e-12)


def test_integrate_base_frame_rotation_accumulates():
    # Two +90deg/step world-frame z rotations -> 180deg about z.
    q = IDENTITY
    for _ in range(2):
        _, q = integrate_pose([0, 0, 0], q, v_lin=[0, 0, 0],
                              v_ang=[0, 0, np.pi / 2], dt=1.0, frame="base")
    np.testing.assert_allclose(np.abs(q), [0, 0, 1, 0], atol=1e-9)


def test_integrate_orientation_stays_unit():
    q = IDENTITY
    pos = np.zeros(3)
    rng = np.random.default_rng(0)
    for _ in range(100):
        pos, q = integrate_pose(pos, q, v_lin=rng.normal(size=3),
                                v_ang=rng.normal(size=3), dt=0.01, frame="tool")
    np.testing.assert_allclose(np.linalg.norm(q), 1.0, atol=1e-9)


def test_integrate_rejects_bad_frame():
    try:
        integrate_pose([0, 0, 0], IDENTITY, [0, 0, 0], [0, 0, 0], 0.1, frame="world")
    except ValueError:
        return
    raise AssertionError("expected ValueError for unknown frame")
