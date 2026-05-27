#!/usr/bin/env python3
"""Live tuning probe for cartesian_compliance_controller.

Adds (relative to first revision):
  * Home pose snapshotted at startup (or supplied via --home).
  * Between every test, sends a 3 s JTC goal to drive every joint back
    to the home pose; waits for the JTC to settle before the next test.
  * Logs starting/ending pose and per-test joint drift.
  * Aborts a test early if any per-joint velocity exceeds an abort
    threshold (default 0.5 rad/s) -- this catches the unsafe runaway
    seen in the previous session.

Workflow per test point:
  0. (At loop start) Send JTC goal home and wait until settled.
  1. /duco_cartesian_control/disengage (best-effort).
  2. Snapshot start pose.
  3. SetParameters atomically on /cartesian_compliance_controller.
  4. /duco_cartesian_control/engage.
  5. Settle window: 1.0 s.  Records velocity already, monitors abort
     threshold; aborts and disengages early on runaway.
  6. Record window: 2.0 s.
  7. /duco_cartesian_control/disengage.
  8. Snapshot end pose; print shake metric + drift.

The "shake" metric is the L2 norm of per-joint velocity RMS over the
record window.  Baseline with controller fully disengaged is ~3.4
mrad/s (encoder noise).
"""
import argparse
import math
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration as DurationMsg


COMPLIANCE_NODE = "/cartesian_compliance_controller"
ORCHESTRATOR = "/duco_cartesian_control"
JTC_NODE = "/arm_1_controller"

JOINT_NAMES = [
    "arm_1_joint_1",
    "arm_1_joint_2",
    "arm_1_joint_3",
    "arm_1_joint_4",
    "arm_1_joint_5",
    "arm_1_joint_6",
]

ABORT_VEL_RAD_S = 0.5    # any joint instantaneously above this triggers abort
ABORT_DRIFT_RAD = 0.10   # any joint drifting more than this from home triggers abort


def make_double(name: str, value: float) -> Parameter:
    p = Parameter()
    p.name = name
    p.value.type = ParameterType.PARAMETER_DOUBLE
    p.value.double_value = float(value)
    return p


def make_int(name: str, value: int) -> Parameter:
    p = Parameter()
    p.name = name
    p.value.type = ParameterType.PARAMETER_INTEGER
    p.value.integer_value = int(value)
    return p


class TuneProbe(Node):
    def __init__(self, home: Optional[List[float]] = None):
        super().__init__("compliance_tune_probe")

        self._set_params_cli = self.create_client(
            SetParameters, f"{COMPLIANCE_NODE}/set_parameters")
        self._engage_cli = self.create_client(
            Trigger, f"{ORCHESTRATOR}/engage")
        self._disengage_cli = self.create_client(
            Trigger, f"{ORCHESTRATOR}/disengage")
        self._jtc_cli = ActionClient(
            self, FollowJointTrajectory, f"{JTC_NODE}/follow_joint_trajectory")

        for cli, label in [(self._set_params_cli, "set_parameters"),
                           (self._engage_cli, "engage"),
                           (self._disengage_cli, "disengage")]:
            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"service {label} not available")
        if not self._jtc_cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "JTC follow_joint_trajectory action not available")

        # Joint state buffer / latest snapshot.
        self._js_lock = threading.Lock()
        self._latest_pos: Dict[str, float] = {}
        self._latest_vel: Dict[str, float] = {}
        self._latest_stamp: float = 0.0
        self._js_samples: List[Tuple[float, List[float]]] = []
        self._recording = False
        self._abort_flag = False

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos)

        # Capture home: either user-supplied, or wait for first JointState.
        if home is not None:
            self._home = list(home)
        else:
            self._home = self._wait_for_pose(timeout=5.0)
        print(f"[home] {[round(p, 6) for p in self._home]}", flush=True)

    # ---- joint-state ---------------------------------------------------------

    def _on_joint_state(self, msg: JointState):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest_pos[name] = pos
            if msg.velocity:
                for name, vel in zip(msg.name, msg.velocity):
                    self._latest_vel[name] = vel
            self._latest_stamp = self.get_clock().now().nanoseconds * 1e-9
            if self._recording and msg.velocity:
                v_ordered = [self._latest_vel.get(j, 0.0) for j in JOINT_NAMES]
                self._js_samples.append((self._latest_stamp, v_ordered))
                if any(abs(v) > ABORT_VEL_RAD_S for v in v_ordered):
                    self._abort_flag = True
                cur = [self._latest_pos.get(j, self._home[i])
                       for i, j in enumerate(JOINT_NAMES)]
                if any(abs(c - h) > ABORT_DRIFT_RAD
                       for c, h in zip(cur, self._home)):
                    self._abort_flag = True

    def _wait_for_pose(self, timeout: float) -> List[float]:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._js_lock:
                if all(j in self._latest_pos for j in JOINT_NAMES):
                    return [self._latest_pos[j] for j in JOINT_NAMES]
        raise RuntimeError("no JointState received within timeout")

    def current_pose(self) -> List[float]:
        with self._js_lock:
            return [self._latest_pos.get(j, float("nan")) for j in JOINT_NAMES]

    def max_drift_from_home(self) -> Tuple[float, int]:
        cur = self.current_pose()
        diffs = [abs(c - h) for c, h in zip(cur, self._home)]
        idx = max(range(len(diffs)), key=lambda i: diffs[i])
        return diffs[idx], idx

    # ---- trajectory ----------------------------------------------------------

    def go_home(self, duration: float = 3.0, settle_tol: float = 0.01,
                settle_timeout: float = 6.0) -> bool:
        """Drive every joint back to the recorded home pose via JTC."""
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(self._home)
        pt.time_from_start = DurationMsg(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_fut = self._jtc_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=5.0)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            print("[home] JTC goal rejected", flush=True)
            return False
        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_fut, timeout_sec=duration + 5.0)
        end = time.monotonic() + settle_timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            d, _ = self.max_drift_from_home()
            if d < settle_tol:
                return True
        d, idx = self.max_drift_from_home()
        print(f"[home] WARNING: max drift {d:.4f} rad at "
              f"{JOINT_NAMES[idx]}", flush=True)
        return False

    # ---- recording -----------------------------------------------------------

    def start_recording(self):
        with self._js_lock:
            self._js_samples = []
            self._recording = True
            self._abort_flag = False

    def stop_recording_and_score(self) -> Tuple[float, List[float], int, bool]:
        with self._js_lock:
            self._recording = False
            samples = list(self._js_samples)
            aborted = self._abort_flag
        if not samples:
            return float("nan"), [], 0, aborted
        n_j = len(samples[0][1])
        sums = [0.0] * n_j
        for _, v in samples:
            for j in range(n_j):
                sums[j] += v[j] * v[j]
        rms = [math.sqrt(s / len(samples)) for s in sums]
        l2 = math.sqrt(sum(r * r for r in rms))
        return l2, rms, len(samples), aborted

    @property
    def aborted(self) -> bool:
        with self._js_lock:
            return self._abort_flag

    # ---- triggers ------------------------------------------------------------

    def _call_trigger(self, client, label: str, timeout: float = 5.0):
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError(f"{label} timed out")
        if not fut.result().success:
            self.get_logger().info(
                f"{label} returned success=false: {fut.result().message}")

    def disengage(self):
        self._call_trigger(self._disengage_cli, "disengage")

    def engage(self):
        self._call_trigger(self._engage_cli, "engage")

    # ---- parameters ----------------------------------------------------------

    def set_params(self, params: Dict[str, float]) -> None:
        req = SetParameters.Request()
        # solver.iterations must be pushed as INTEGER; everything else
        # is double.
        msgs: List[Parameter] = []
        for n, v in params.items():
            if n == "solver.iterations":
                msgs.append(make_int(n, int(v)))
            else:
                msgs.append(make_double(n, float(v)))
        req.parameters = msgs
        fut = self._set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            raise RuntimeError("set_parameters timed out")
        for name, r in zip(params.keys(), fut.result().results):
            if not r.successful:
                raise RuntimeError(
                    f"set_parameters failed for {name}: {r.reason}")


# ---- candidate builder -------------------------------------------------------

def candidate(P_t, D_t, K_t, P_r, D_r, K_r, scale=0.05,
              iters: Optional[int] = None) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for ax in ("trans_x", "trans_y", "trans_z"):
        out[f"pd_gains.{ax}.p"] = float(P_t)
        out[f"pd_gains.{ax}.d"] = float(D_t)
        out[f"stiffness.{ax}"] = float(K_t)
    for ax in ("rot_x", "rot_y", "rot_z"):
        out[f"pd_gains.{ax}.p"] = float(P_r)
        out[f"pd_gains.{ax}.d"] = float(D_r)
        out[f"stiffness.{ax}"] = float(K_r)
    out["solver.error_scale"] = float(scale)
    if iters is not None:
        out["solver.iterations"] = float(iters)
    return out


def fmt_label(params: Dict[str, float]) -> str:
    iters_part = (f" it={int(params['solver.iterations'])}"
                  if 'solver.iterations' in params else "")
    return (f"K_t={params['stiffness.trans_x']:>5.1f} "
            f"P_t={params['pd_gains.trans_x.p']:>5.2f} "
            f"D_t={params['pd_gains.trans_x.d']:>5.3f}  "
            f"K_r={params['stiffness.rot_x']:>4.1f} "
            f"P_r={params['pd_gains.rot_x.p']:>5.1f} "
            f"D_r={params['pd_gains.rot_x.d']:>5.3f}  "
            f"scale={params['solver.error_scale']:.3f}{iters_part}")


def run_one(probe: TuneProbe, params: Dict[str, float],
            settle_s: float = 1.0,
            record_s: float = 2.0) -> Tuple[float, bool, float]:
    # 0. Always start by returning to home.
    probe.go_home(duration=3.0)
    time.sleep(0.2)

    probe.disengage()
    time.sleep(0.2)
    probe.set_params(params)
    time.sleep(0.1)

    probe.start_recording()
    probe.engage()

    end_settle = time.monotonic() + settle_s
    while time.monotonic() < end_settle:
        rclpy.spin_once(probe, timeout_sec=0.02)
        if probe.aborted:
            break

    if not probe.aborted:
        with probe._js_lock:
            probe._js_samples = []
        end_record = time.monotonic() + record_s
        while time.monotonic() < end_record:
            rclpy.spin_once(probe, timeout_sec=0.02)
            if probe.aborted:
                break

    score, per_joint, n, aborted = probe.stop_recording_and_score()

    probe.disengage()
    time.sleep(0.3)

    drift, idx = probe.max_drift_from_home()
    per_joint_str = (" ".join(f"{r*1000:5.1f}" for r in per_joint)
                     if per_joint else "")
    flag = "ABORT " if aborted else "      "
    print(f"{flag}{fmt_label(params)}  ->  shake={score*1000:6.2f} mrad/s   "
          f"drift={drift*1000:5.1f} mrad @ J{idx+1}   "
          f"(n={n}: {per_joint_str})", flush=True)
    return score, aborted, drift


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--record", type=float, default=2.0)
    ap.add_argument("--single", action="store_true",
                    help="Run only the first candidate then exit.")
    ap.add_argument("--home", type=float, nargs=6, default=None,
                    help="6 joint positions to use as home; default = current.")
    args = ap.parse_args()

    rclpy.init()
    probe = TuneProbe(home=args.home)

    # Each row: (P_t, D_t, K_t, P_r, D_r, K_r, scale [, iters])
    #
    # First sweep established the stability bound:
    #   P_t * K_t * scale * iters <= ~30  for stable inner loop.
    # We now sweep higher K (firmer operator feel) while keeping the
    # product near the bound, to find the firmest stiffness we can get.
    candidates: List[Tuple] = [
        # 1: K=50 with iters=1 -- product = 10*50*0.05*1 = 25
        (10.0, 0.0, 50.0, 50.0, 0.0, 3.0, 0.05, 1),
        # 2: K=50 with iters=5 and scale=0.01 -- product = 10*50*0.01*5 = 25
        (10.0, 0.0, 50.0, 50.0, 0.0, 3.0, 0.01, 5),
        # 3: K=100 with iters=1 and scale=0.02 -- product = 10*100*0.02*1 = 20
        (10.0, 0.0, 100.0, 80.0, 0.0, 5.0, 0.02, 1),
        # 4: K=100 with iters=1 and scale=0.05 -- product = 5*100*0.05*1 = 25
        (5.0, 0.0, 100.0, 50.0, 0.0, 5.0, 0.05, 1),
        # 5: K=30 (current YAML K) raised to iters=2 -- product = 10*30*0.05*2 = 30
        (10.0, 0.0, 30.0, 50.0, 0.0, 2.0, 0.05, 2),
        # 6: Verify boundary -- iters=1 product = 30
        (10.0, 0.0, 60.0, 50.0, 0.0, 3.0, 0.05, 1),
    ]

    try:
        scores = []
        for c in candidates:
            params = candidate(*c)
            score, aborted, drift = run_one(
                probe, params, settle_s=args.settle, record_s=args.record)
            scores.append((score, aborted, drift, params))
            if args.single:
                break

        if not args.single:
            print()
            print("=== ALL results (lowest shake first, aborted excluded) ===")
            ok = [s for s in scores if not s[1]]
            ok.sort(key=lambda x: x[0])
            for s, ab, dr, p in ok:
                print(f"  shake={s*1000:6.2f} mrad/s  "
                      f"drift={dr*1000:5.1f} mrad   {fmt_label(p)}")
            aborted_runs = [s for s in scores if s[1]]
            if aborted_runs:
                print("--- aborted runs ---")
                for s, ab, dr, p in aborted_runs:
                    print(f"  ABORT  drift={dr*1000:5.1f} mrad  "
                          f"{fmt_label(p)}")
    finally:
        try:
            probe.disengage()
        except Exception:
            pass
        try:
            probe.go_home(duration=3.0)
        except Exception:
            pass
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
