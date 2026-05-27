#!/usr/bin/env python3
"""
Robot-agnostic auto-tuner for ``cartesian_force_controller`` PD gains.

Resolves the active robot from ``ROBOT_CONFIG_PATH`` (a top-level
``config/robot_config.<robot>.yaml`` referenced by
``cartesian_control_manager``), then:

1. Pre-flight: measures ``/ft_compensated`` mean for 3 s WITHOUT
   engaging.  Aborts only on |F|>15 N / |T|>2 Nm (broken sensor).
2. Bias-cancel: sets the orchestrator's heartbeated
   ``target_wrench_force_*`` / ``target_wrench_torque_*`` params to
   the observed bias so steady-state controller error ~= 0 even when
   gravity compensation is uncalibrated.  Restored to 0 in finally.
3. Ladder ``trans_p in [0.5..5.0]`` with proportional d, rot_p, rot_d
   and ``solver.error_scale = 0.02``.  Each step:
   ``set_parameters`` -> engage -> 0.5 s settle -> 2 s measurement ->
   disengage -> 2.5 s settle (under the manager's |qdot| engage gate).
4. Stability metrics: joint position range, joint velocity std-dev,
   growth factor vs previous trial.  Aborts on range > 3 deg,
   vel-std > 20 deg/s, or growth > 4x (true oscillation, distinct
   from linear noise-tracking that just scales with p).
5. Live-sets best safe gains.
6. Persists tuned values to the per-robot FZI preset YAML pointed to
   by ``cartesian_control_manager.fzi_controller_yaml_package`` /
   ``...yaml_relpath`` in the top-level config.  Comments preserved.

Usage:
    export ROBOT_CONFIG_PATH=$PWD/config/robot_config.<robot>.yaml
    python3 tools/cartesian_auto_tune.py [--no-writeback]

The controller is left disengaged on any error or Ctrl-C.
"""

import argparse
import math
import os
import re
from pathlib import Path
import sys
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


# --- Robot-config resolution -------------------------------------------

def find_workspace_root():
    """Walk up from this script to find the colcon workspace root.

    The script ships at ``<ws>/tools/cartesian_auto_tune.py``."""
    here = Path(__file__).resolve().parent
    for p in (here, here.parent, here.parent.parent):
        if (p / 'src').is_dir() and (p / 'external').is_dir():
            return p
    return Path.cwd()


def resolve_preset_path(robot_config_path):
    """Read the top-level robot config and return ``(preset_path, package_name)``.

    Looks up ``cartesian_control_manager.fzi_controller_yaml_package``
    and ``fzi_controller_yaml_relpath``, then resolves to
    ``<workspace>/src/<package>/<relpath>``."""
    cfg = yaml.safe_load(Path(robot_config_path).read_text())
    mgr = (cfg or {}).get('cartesian_control_manager', {}) or {}
    pkg = mgr.get('fzi_controller_yaml_package')
    rel = mgr.get('fzi_controller_yaml_relpath')
    if not pkg or not rel:
        raise ValueError(
            f"{robot_config_path}: missing\n"
            f"  cartesian_control_manager.fzi_controller_yaml_package\n"
            f"  cartesian_control_manager.fzi_controller_yaml_relpath")
    ws = find_workspace_root()
    candidate = ws / 'src' / pkg / rel
    if not candidate.is_file():
        raise FileNotFoundError(
            f"FZI preset not found at {candidate}\n"
            f"  (resolved from {robot_config_path}\n"
            f"   via fzi_controller_yaml_package={pkg!r}\n"
            f"   and fzi_controller_yaml_relpath={rel!r})")
    return candidate, pkg


def _slice_top_level_section(text, section_key):
    """Return ``(start, end)`` byte offsets of a top-level YAML section.

    A 'top-level section' starts at ``^<section_key>:`` and ends at the
    next top-level key (a line starting with a non-space, non-``#``
    character followed by ``:``) or end-of-file."""
    m = re.search(rf'^{re.escape(section_key)}:\s*$', text, re.MULTILINE)
    if not m:
        raise ValueError(f"top-level key {section_key!r} not found")
    start = m.end()
    nxt = re.search(r'^[A-Za-z_][A-Za-z0-9_]*:\s*$', text[start:], re.MULTILINE)
    end = start + (nxt.start() if nxt else len(text) - start)
    return m.start(), end


def write_back_preset(path, trans_p, trans_d, rot_p, rot_d, error_scale):
    """Persist tuned ``cartesian_force_controller`` gains to the per-robot
    preset YAML.  Updates only the six ``pd_gains.*`` axis lines plus
    ``solver.error_scale``; ``solver.iterations`` is left untouched (the
    auto-tuner does not sweep it).  Does NOT touch
    ``cartesian_motion_controller`` or ``cartesian_compliance_controller``
    sections -- only the controller we actually tuned."""
    text = path.read_text()
    sec_start, sec_end = _slice_top_level_section(text, 'cartesian_force_controller')
    section = text[sec_start:sec_end]

    fmt_trans = f"{{p: {trans_p}, d: {trans_d}}}"
    fmt_rot   = f"{{p: {rot_p}, d: {rot_d}}}"
    edits = 0
    for ax in ('trans_x', 'trans_y', 'trans_z'):
        section, n = re.subn(
            rf'(?m)^([ \t]+){ax}:\s*\{{[^}}]*\}}([ \t]*#.*)?$',
            lambda m, ax=ax: f"{m.group(1)}{ax}: {fmt_trans}{m.group(2) or ''}",
            section, count=1)
        if n == 0:
            raise ValueError(f"could not locate {ax}: {{...}} line under cartesian_force_controller")
        edits += n
    for ax in ('rot_x', 'rot_y', 'rot_z'):
        section, n = re.subn(
            rf'(?m)^([ \t]+){ax}:\s*\{{[^}}]*\}}([ \t]*#.*)?$',
            lambda m, ax=ax: f"{m.group(1)}{ax}: {fmt_rot}{m.group(2) or ''}",
            section, count=1)
        if n == 0:
            raise ValueError(f"could not locate {ax}: {{...}} line under cartesian_force_controller")
        edits += n

    section, n = re.subn(
        r'(?m)^([ \t]+)error_scale:[ \t]*[+\-0-9.]+([ \t]*#.*)?$',
        lambda m: f"{m.group(1)}error_scale: {error_scale}{m.group(2) or ''}",
        section, count=1)
    if n:
        edits += n

    new_text = text[:sec_start] + section + text[sec_end:]
    if new_text == text:
        return False, edits, None
    backup = path.with_suffix(path.suffix + '.autotune.bak')
    backup.write_text(text)
    path.write_text(new_text)
    return True, edits, backup


# --- Safety thresholds --------------------------------------------------
FT_BIAS_FORCE_MAX_N  = 15.0   # hard abort (very large -- protects against
FT_BIAS_TORQUE_MAX_NM = 2.0   # truly broken FT data, e.g. unplugged sensor)
JOINT_RANGE_ABORT_RAD = math.radians(3.0)   # 3 deg per-joint range = abort
JOINT_VEL_STD_ABORT   = math.radians(20.0)  # 20 deg/s std-dev of joint velocity = oscillation
GROWTH_FACTOR_ABORT   = 4.0                 # if range grows >4x vs previous trial = instability onset
ENGAGE_SETTLE_SEC = 0.5
ENGAGE_MEASURE_SEC = 2.0
DISENGAGE_SETTLE_SEC = 2.5      # wait for arm |qdot| to drop below the manager's 0.02 rad/s gate
PREFLIGHT_SEC = 3.0
LADDER = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0]
ERROR_SCALE = 0.02


class AutoTuner(Node):
    def __init__(self):
        super().__init__('cartesian_auto_tuner')
        # /joint_states uses BEST_EFFORT (sensor default)
        qos_be = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        # /ft_compensated uses RELIABLE
        qos_r = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.joint_buf = []
        self.wrench_buf = []
        self.recording = False
        self.create_subscription(JointState, '/joint_states', self._joint_cb, qos_be)
        self.create_subscription(WrenchStamped, '/ft_compensated', self._wrench_cb, qos_r)
        self.param_cli     = self.create_client(SetParameters, '/cartesian_force_controller/set_parameters')
        self.mgr_param_cli = self.create_client(SetParameters, '/cartesian_control_manager/set_parameters')
        self.engage_cli    = self.create_client(Trigger,        '/cartesian_control_manager/engage')
        self.disengage_cli = self.create_client(Trigger,        '/cartesian_control_manager/disengage')
        for cli, name in [
            (self.param_cli,     '/cartesian_force_controller/set_parameters'),
            (self.mgr_param_cli, '/cartesian_control_manager/set_parameters'),
            (self.engage_cli,    '/cartesian_control_manager/engage'),
            (self.disengage_cli, '/cartesian_control_manager/disengage'),
        ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"Service {name} not available")

    def _joint_cb(self, msg):
        if self.recording:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.joint_buf.append((stamp, list(msg.position)))

    def _wrench_cb(self, msg):
        if self.recording:
            f = msg.wrench.force
            t = msg.wrench.torque
            self.wrench_buf.append([f.x, f.y, f.z, t.x, t.y, t.z])

    def _spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def measure(self, duration):
        self.joint_buf.clear()
        self.wrench_buf.clear()
        self.recording = True
        self._spin_for(duration)
        self.recording = False
        if self.joint_buf:
            ts = np.array([t for (t, _) in self.joint_buf])
            pos = np.array([p for (_, p) in self.joint_buf])
        else:
            ts = np.empty((0,))
            pos = np.empty((0, 6))
        w = np.array(self.wrench_buf) if self.wrench_buf else np.empty((0, 6))
        return ts, pos, w

    def set_gains(self, trans_p, error_scale):
        d_trans = round(trans_p * 0.05, 4)
        rot_p   = round(trans_p * 20.0, 4)
        d_rot   = round(rot_p * 0.1, 4)
        params = []
        for ax in ('trans_x', 'trans_y', 'trans_z'):
            params.append(Parameter(name=f'pd_gains.{ax}.p',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(trans_p))))
            params.append(Parameter(name=f'pd_gains.{ax}.d',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(d_trans))))
        for ax in ('rot_x', 'rot_y', 'rot_z'):
            params.append(Parameter(name=f'pd_gains.{ax}.p',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(rot_p))))
            params.append(Parameter(name=f'pd_gains.{ax}.d',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(d_rot))))
        params.append(Parameter(name='solver.error_scale',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(error_scale))))
        fut = self.param_cli.call_async(SetParameters.Request(parameters=params))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            raise RuntimeError("set_parameters timed out")
        for i, r in enumerate(res.results):
            if not r.successful:
                raise RuntimeError(f"set_parameters[{i}]={params[i].name} failed: {r.reason}")
        return d_trans, rot_p, d_rot

    def engage(self, retries=3):
        for attempt in range(retries):
            fut = self.engage_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            r = fut.result()
            if r is not None and r.success:
                return
            msg = r.message if r else 'timeout'
            if 'arm is moving' in (msg or '') and attempt < retries - 1:
                print(f"  engage retry {attempt+1}/{retries-1}: {msg!r} -- waiting another {DISENGAGE_SETTLE_SEC}s")
                time.sleep(DISENGAGE_SETTLE_SEC)
                continue
            raise RuntimeError(f"engage failed: {msg}")

    def disengage(self):
        fut = self.disengage_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        if r is None or not r.success:
            self.get_logger().warn(f"disengage returned: {r.message if r else 'timeout'}")

    def set_target_wrench(self, fx, fy, fz, tx, ty, tz):
        """Set the orchestrator's heartbeated target_wrench so the
        force controller's error = target - measured equals zero when
        the measured /ft_compensated equals (fx, fy, fz, tx, ty, tz).
        Call with the observed bias to neutralise it during tuning."""
        names_vals = [
            ('target_wrench_force_x',  fx), ('target_wrench_force_y',  fy), ('target_wrench_force_z',  fz),
            ('target_wrench_torque_x', tx), ('target_wrench_torque_y', ty), ('target_wrench_torque_z', tz),
        ]
        params = [
            Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                   double_value=float(v)))
            for n, v in names_vals
        ]
        fut = self.mgr_param_cli.call_async(SetParameters.Request(parameters=params))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            raise RuntimeError("manager set_parameters timed out")
        for i, r in enumerate(res.results):
            if not r.successful:
                raise RuntimeError(f"manager set_parameters[{i}]={params[i].name} failed: {r.reason}")


def main():
    parser = argparse.ArgumentParser(description="Robot-aware auto-tuner for cartesian_force_controller.")
    parser.add_argument('--no-writeback', action='store_true',
                        help="Do not modify the per-robot FZI preset YAML; just live-set best p.")
    parser.add_argument('--robot-config',
                        help="Override path to top-level robot config (otherwise read ROBOT_CONFIG_PATH).")
    args = parser.parse_args()

    robot_config_path = args.robot_config or os.environ.get('ROBOT_CONFIG_PATH', '')
    if not robot_config_path:
        print("ERROR: ROBOT_CONFIG_PATH not set and --robot-config not given.", file=sys.stderr)
        print("  e.g. export ROBOT_CONFIG_PATH=$PWD/config/robot_config.ur15.yaml", file=sys.stderr)
        return 2
    try:
        preset_path, robot_pkg = resolve_preset_path(robot_config_path)
    except (ValueError, FileNotFoundError) as e:
        print(f"ERROR resolving per-robot FZI preset:\n  {e}", file=sys.stderr)
        return 2
    print(f"[robot] config       : {robot_config_path}")
    print(f"[robot] bringup pkg  : {robot_pkg}")
    print(f"[robot] FZI preset   : {preset_path}")
    print(f"[robot] writeback    : {'OFF' if args.no_writeback else 'ON (with .autotune.bak backup)'}")

    rclpy.init()
    tuner = AutoTuner()
    engaged = False
    bias_applied = False
    try:
        print("\n=== PHASE 1: pre-flight FT bias (3s, NOT engaged) ===")
        ts_pre, j_pre, w_pre = tuner.measure(PREFLIGHT_SEC)
        if w_pre.shape[0] < 10:
            print(f"  ABORT: only {w_pre.shape[0]} wrench samples received.")
            print("  Is ft_sensor_gravity_compensation publishing /ft_compensated?")
            return 2
        f_mean = np.mean(w_pre[:, :3], axis=0)
        t_mean = np.mean(w_pre[:, 3:], axis=0)
        f_mag = float(np.linalg.norm(f_mean))
        t_mag = float(np.linalg.norm(t_mean))
        print(f"  /ft_compensated samples: {w_pre.shape[0]}")
        print(f"  mean force  (N):  fx={f_mean[0]:+.3f}  fy={f_mean[1]:+.3f}  fz={f_mean[2]:+.3f}  |F|={f_mag:.3f}")
        print(f"  mean torque (Nm): tx={t_mean[0]:+.3f}  ty={t_mean[1]:+.3f}  tz={t_mean[2]:+.3f}  |T|={t_mag:.3f}")
        if f_mag > FT_BIAS_FORCE_MAX_N or t_mag > FT_BIAS_TORQUE_MAX_NM:
            print(f"\n  ABORT: FT bias unreasonably large (|F|>{FT_BIAS_FORCE_MAX_N}N or |T|>{FT_BIAS_TORQUE_MAX_NM}Nm).")
            print("  This usually means the FT sensor is broken or unplugged.")
            return 2
        # Cancel the bias via orchestrator's target_wrench heartbeat.
        # The controller minimises (target - measured); setting target =
        # measured_mean makes the steady-state error zero so the arm sits
        # still during tuning even if gravity comp is uncalibrated.
        print("\n=== PHASE 1b: bias-cancel via target_wrench heartbeat ===")
        tuner.set_target_wrench(f_mean[0], f_mean[1], f_mean[2],
                                t_mean[0], t_mean[1], t_mean[2])
        bias_applied = True
        print(f"  set /cartesian_control_manager target_wrench_force_*  = ({f_mean[0]:+.3f}, {f_mean[1]:+.3f}, {f_mean[2]:+.3f}) N")
        print(f"  set /cartesian_control_manager target_wrench_torque_* = ({t_mean[0]:+.4f}, {t_mean[1]:+.4f}, {t_mean[2]:+.4f}) Nm")
        print("  (will be restored to 0 in the finally block)")
        if j_pre.shape[0] >= 5:
            jr_idle = np.ptp(j_pre, axis=0)
            print(f"  idle joint range (deg): {[f'{math.degrees(x):.4f}' for x in jr_idle]}")

        print("\n=== PHASE 2: tuning ladder ===")
        results = []
        last_safe_p = None
        last_range = None
        for p in LADDER:
            d_t, rp, d_r = tuner.set_gains(p, ERROR_SCALE)
            print(f"\n--- trial trans_p={p}  d={d_t}  rot_p={rp}  rot_d={d_r}  es={ERROR_SCALE} ---")
            tuner.engage()
            engaged = True
            time.sleep(ENGAGE_SETTLE_SEC)
            ts_eng, j_eng, w_eng = tuner.measure(ENGAGE_MEASURE_SEC)
            tuner.disengage()
            engaged = False
            # Let arm decelerate below the manager's 0.02 rad/s engage gate.
            time.sleep(DISENGAGE_SETTLE_SEC)

            if j_eng.shape[0] < 10:
                print(f"  skip: only {j_eng.shape[0]} joint samples during engage")
                continue
            jr = np.ptp(j_eng, axis=0)
            jr_max = float(np.max(jr))
            jr_max_deg = math.degrees(jr_max)
            # joint velocity (numerical, central diff)
            if ts_eng.shape[0] >= 3 and (ts_eng[-1] - ts_eng[0]) > 0.1:
                dt = np.diff(ts_eng)
                # protect against zero-dt artefacts
                dt = np.where(dt > 1e-4, dt, 1e-4)
                vel = np.diff(j_eng, axis=0) / dt[:, None]
                vel_std_max = float(np.max(np.std(vel, axis=0)))
                vel_std_max_dps = math.degrees(vel_std_max)
            else:
                vel_std_max = 0.0
                vel_std_max_dps = 0.0
            w_std = float(np.std(w_eng[:, :3])) if w_eng.size else 0.0
            growth = (jr_max / last_range) if last_range and last_range > 1e-6 else 1.0
            print(f"  joint range max = {jr_max_deg:.3f} deg  ({jr_max:.5f} rad)")
            print(f"  per-joint range (deg): {[f'{math.degrees(x):.3f}' for x in jr]}")
            print(f"  joint vel std max = {vel_std_max_dps:.2f} deg/s")
            print(f"  wrench std (combined force, N): {w_std:.3f}")
            print(f"  growth vs previous: x{growth:.2f}")

            range_unstable  = jr_max > JOINT_RANGE_ABORT_RAD
            vel_unstable    = vel_std_max > JOINT_VEL_STD_ABORT
            growth_unstable = growth > GROWTH_FACTOR_ABORT and last_range is not None
            unstable = range_unstable or vel_unstable or growth_unstable
            reasons = []
            if range_unstable:  reasons.append(f"range {jr_max_deg:.2f}>{math.degrees(JOINT_RANGE_ABORT_RAD):.1f}deg")
            if vel_unstable:    reasons.append(f"vel_std {vel_std_max_dps:.1f}>{math.degrees(JOINT_VEL_STD_ABORT):.0f}deg/s")
            if growth_unstable: reasons.append(f"growth x{growth:.1f}>{GROWTH_FACTOR_ABORT:.1f}")
            results.append((p, jr_max_deg, vel_std_max_dps, growth, unstable, reasons))
            if unstable:
                print(f"  ===> UNSTABLE: {', '.join(reasons)}")
                break
            print(f"  ===> STABLE at p={p}")
            last_safe_p = p
            last_range = jr_max

        print("\n=== PHASE 3: apply final gains ===")
        if last_safe_p is None:
            print("  No safe gain found in ladder; restoring p=1.0 as a conservative default.")
            last_safe_p = 1.0
        tuner.set_gains(last_safe_p, ERROR_SCALE)
        trans_d_final = round(last_safe_p * 0.05, 4)
        rot_p_final   = round(last_safe_p * 20.0, 4)
        rot_d_final   = round(rot_p_final * 0.1, 4)
        print(f"  Live gains now: trans_p={last_safe_p}  trans_d={trans_d_final}  rot_p={rot_p_final}  rot_d={rot_d_final}  error_scale={ERROR_SCALE}")

        if not args.no_writeback:
            print("\n=== PHASE 4: persist to per-robot preset YAML ===")
            try:
                changed, n_edits, backup = write_back_preset(
                    preset_path,
                    trans_p=last_safe_p,
                    trans_d=trans_d_final,
                    rot_p=rot_p_final,
                    rot_d=rot_d_final,
                    error_scale=ERROR_SCALE,
                )
            except Exception as e:
                print(f"  WRITEBACK FAILED: {e}", file=sys.stderr)
                print("  Live gains remain applied; YAML was NOT modified.", file=sys.stderr)
            else:
                if changed:
                    print(f"  wrote {n_edits} edits to {preset_path}")
                    print(f"  backup saved to {backup}")
                else:
                    print(f"  preset already matched tuned values; no file change.")

        print("\n=== SUMMARY ===")
        for r in results:
            p, jr, vs, gf, unstable, reasons = r
            mark = ("UNSTABLE: " + ", ".join(reasons)) if unstable else "  stable"
            print(f"  p={p:.2f}  range={jr:.3f}deg  vel_std={vs:.2f}deg/s  growth=x{gf:.2f}  {mark}")
        print(f"\nBEST_SAFE_P={last_safe_p}")
        return 0

    finally:
        if engaged:
            try:
                tuner.disengage()
                print("[finally] controller disengaged on exit")
            except Exception as e:
                print(f"[finally] WARN: disengage on exit failed: {e}", file=sys.stderr)
        if bias_applied:
            try:
                tuner.set_target_wrench(0, 0, 0, 0, 0, 0)
                print("[finally] target_wrench restored to (0,0,0,0,0,0)")
            except Exception as e:
                print(f"[finally] WARN: target_wrench restore failed: {e}", file=sys.stderr)
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
