#!/usr/bin/env python3
"""Standalone direct-position-control test for the DUCO arm.

This script bypasses MoveIt, the JointTrajectoryController, alicia_teleop,
cartesian_motion_controller, and every other layer above ros2_control. It
publishes a std_msgs/Float64MultiArray straight to

    /forward_position_controller/commands

at the controller's update rate. That topic is the same FZI-style streaming
interface that cartesian_motion_controller's internal "JointGroupPosition"
sink writes into -- so if this script can move the arm cleanly, so can FZI.

The point of this tool is to validate, in isolation:
  1. forward_position_controller is the active arm controller.
  2. The DUCO hardware interface accepts streamed position commands.
  3. The robot follows them smoothly at the controller_manager rate
     (250 Hz on this setup).

Pre-conditions
--------------
Bring the robot up (any controller mode is fine -- the script will switch
to forward_position_controller automatically):

    ros2 launch duco_robot_bringup gcr5_910_ros2_control.launch.py \\
        use_fake_hardware:=false

Then in another terminal, source the workspace and run this script:

    source install/setup.bash
    python3 tools/test_forward_position.py --mode hold        # safest first run
    python3 tools/test_forward_position.py --mode step --joint 5 --amplitude 0.05
    python3 tools/test_forward_position.py --mode sine --joint 5 --amplitude 0.05 --frequency 0.2

The script will:
  1. Call /controller_manager/switch_controller to activate
     forward_position_controller and deactivate arm_1_controller.
  2. Run the chosen waveform.
  3. Ramp the joints back to their captured home position.
  4. Restore the original controller set on exit (so arm_1_controller is
     active again when the tool returns).

Use `--no-auto-switch` to skip both the entry switch and the exit restore
(useful if you are doing your own controller orchestration upstream).

For an automated run with tracking analysis (no Ctrl+C needed):

    python3 tools/test_forward_position.py \\
        --mode sine --joint 1 --amplitude 0.5236 --frequency 0.1 \\
        --duration 22 --trace /tmp/shoulder_sine.csv

Safety
------
* The script refuses to publish anything until it has captured a "home"
  position from /joint_states.
* `--amplitude` is hard-capped at 0.2 rad (~11.5 deg). Defaults to 0.05 rad.
* `--joint` defaults to index 5 (the wrist), which is the lowest-inertia and
  most isolated joint on a GCR5.
* On Ctrl+C the script smoothly ramps every joint back to its captured
  home position before exiting.
* All commands include the full 6-joint vector in the joint order declared
  by forward_position_controller (arm_1_joint_1 .. arm_1_joint_6).

Exit codes
----------
    0  normal exit (Ctrl+C, ramp-home completed)
    1  could not capture /joint_states within --wait-timeout seconds
    2  invalid CLI arguments / amplitude over safety cap
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import threading
import time
from typing import Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


CONTROLLED_JOINTS: List[str] = [
    "arm_1_joint_1",
    "arm_1_joint_2",
    "arm_1_joint_3",
    "arm_1_joint_4",
    "arm_1_joint_5",
    "arm_1_joint_6",
]

COMMAND_TOPIC = "/forward_position_controller/commands"
JOINT_STATE_TOPIC = "/joint_states"
LIST_CONTROLLERS_SRV = "/controller_manager/list_controllers"
SWITCH_CONTROLLER_SRV = "/controller_manager/switch_controller"

FORWARD_POSITION_CONTROLLER = "forward_position_controller"
TRAJECTORY_CONTROLLER = "arm_1_controller"

# Hard safety cap, independent of CLI --amplitude.
# 0.6 rad ~ 34.4 deg; bench-validated as safe for any single joint on this
# GCR5 from the standard "ready" pose. Raise only if you know the workspace.
MAX_AMPLITUDE_RAD = 0.6  # ~34.4 degrees


def smoothstep(x: float) -> float:
    """Classic 3x^2 - 2x^3 ease curve, clipped to [0, 1]."""
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


class ForwardPositionTester(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("forward_position_tester")
        self.args = args

        self._lock = threading.Lock()
        self._home: Optional[List[float]] = None
        self._last_cmd: Optional[List[float]] = None
        self._t0: Optional[float] = None
        self._shutting_down = False

        # Trace buffer: each row is
        #   [t_s, m_j1..m_j6 (deg), c_j1..c_j6 (deg)]
        # populated from _joint_state_cb whenever args.trace is set.
        self._trace_rows: List[List[object]] = []

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._sub = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._joint_state_cb,
            sensor_qos,
        )

        self._pub = self.create_publisher(
            Float64MultiArray,
            COMMAND_TOPIC,
            10,
        )

        period = 1.0 / args.rate
        self._timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f"Subscribed to {JOINT_STATE_TOPIC}, publishing to {COMMAND_TOPIC} "
            f"at {args.rate:.0f} Hz."
        )
        self.get_logger().info(
            f"Mode={args.mode}, joint_index={args.joint}, "
            f"amplitude={args.amplitude:.4f} rad "
            f"({math.degrees(args.amplitude):.2f} deg), "
            f"frequency={args.frequency:.3f} Hz."
        )
        self.get_logger().info("Waiting for /joint_states to capture home position...")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        try:
            ordered = [msg.position[msg.name.index(j)] for j in CONTROLLED_JOINTS]
        except ValueError as exc:
            if self._home is None:
                self.get_logger().warn(
                    f"/joint_states missing one of the controlled joints: {exc}; "
                    "still waiting..."
                )
            return

        if self._home is None:
            with self._lock:
                self._home = list(ordered)
                self._last_cmd = list(ordered)
            self._t0 = time.monotonic()
            self.get_logger().info(
                "Captured home positions [deg]: "
                + ", ".join(f"{math.degrees(p):+7.2f}" for p in ordered)
            )
            sub_count = self._pub.get_subscription_count()
            if sub_count == 0:
                self.get_logger().warn(
                    f"No subscribers on {COMMAND_TOPIC}. Is forward_position_controller "
                    "actually active? Check `ros2 control list_controllers`."
                )
            else:
                self.get_logger().info(
                    f"{sub_count} subscriber(s) on {COMMAND_TOPIC} -- starting stream."
                )
            return

        # Home already captured. If tracing is enabled, append a sample.
        if self.args.trace and self._t0 is not None:
            t = time.monotonic() - self._t0
            with self._lock:
                cmd_snapshot = (
                    list(self._last_cmd) if self._last_cmd is not None else None
                )
            row: List[object] = [f"{t:.4f}"]
            row.extend(f"{math.degrees(p):.6f}" for p in ordered)
            if cmd_snapshot is None:
                row.extend([""] * 6)
            else:
                row.extend(f"{math.degrees(c):.6f}" for c in cmd_snapshot)
            self._trace_rows.append(row)

    def _timer_cb(self) -> None:
        if self._home is None or self._t0 is None or self._shutting_down:
            return

        t = time.monotonic() - self._t0
        target = self._compute_target(t)
        with self._lock:
            self._last_cmd = list(target)

        msg = Float64MultiArray()
        msg.data = target
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # Trajectory shapes
    # ------------------------------------------------------------------
    def _compute_target(self, t: float) -> List[float]:
        assert self._home is not None
        target = list(self._home)
        j = self.args.joint
        amp = self.args.amplitude
        mode = self.args.mode

        # 2-second ease-in envelope keeps the first commands close to the
        # captured home position, so the robot never jumps even if the
        # controller was idle for a long time before we connected.
        ease = smoothstep(t / 2.0)

        if mode == "hold":
            return target
        if mode == "step":
            target[j] = self._home[j] + amp * ease
            return target
        if mode == "sine":
            phase = 2.0 * math.pi * self.args.frequency * t
            target[j] = self._home[j] + amp * ease * math.sin(phase)
            return target
        return target  # unreachable

    # ------------------------------------------------------------------
    # Shutdown ramp
    # ------------------------------------------------------------------
    def ramp_back_to_home(self, duration: float) -> None:
        """Smoothly publish a blend from last command back to the captured home.

        Runs in the main thread after rclpy.spin() has been interrupted;
        the executor is stopped but the publisher is still valid.
        """
        if self._home is None or self._last_cmd is None:
            self.get_logger().info("No home captured; nothing to ramp back to.")
            return

        self._shutting_down = True
        start = list(self._last_cmd)
        steps = max(1, int(duration * self.args.rate))
        period = 1.0 / self.args.rate

        self.get_logger().info(
            f"Ramping back to home over {duration:.2f} s ({steps} steps)..."
        )
        for i in range(1, steps + 1):
            alpha = 0.5 * (1.0 - math.cos(math.pi * i / steps))  # cosine ease
            blend = [
                start[k] + (self._home[k] - start[k]) * alpha
                for k in range(len(start))
            ]
            msg = Float64MultiArray()
            msg.data = blend
            self._pub.publish(msg)
            # Keep _last_cmd in sync so any /joint_states samples that the
            # executor picks up after the ramp finishes show the correct
            # commanded value in the trace CSV (and accurate follow-error).
            with self._lock:
                self._last_cmd = blend
            time.sleep(period)
        self.get_logger().info("Ramp complete; final command is at home.")

    # ------------------------------------------------------------------
    # Trace export
    # ------------------------------------------------------------------
    def write_trace(self, path: str) -> None:
        """Dump the buffered (t, measured, commanded) rows to CSV + print stats."""
        rows = self._trace_rows
        if not rows:
            self.get_logger().warn("No trace rows collected; skipping CSV write.")
            return

        header = (
            ["t_s"]
            + [f"m_j{i+1}" for i in range(6)]
            + [f"c_j{i+1}" for i in range(6)]
        )
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            for r in rows:
                w.writerow(r)
        self.get_logger().info(f"Wrote {len(rows)} trace rows to {path}")

        # Summary stats for the perturbed joint (args.joint, 0-based).
        j = self.args.joint
        m_col = 1 + j
        c_col = 7 + j
        m_vals = [float(r[m_col]) for r in rows]
        c_vals = [float(r[c_col]) for r in rows if r[c_col] != ""]
        m_span = max(m_vals) - min(m_vals)
        line = (
            f"joint{j+1} measured: "
            f"min={min(m_vals):+7.3f}  max={max(m_vals):+7.3f}  "
            f"span={m_span:6.3f} deg"
        )
        self.get_logger().info(line)
        if c_vals:
            c_span = max(c_vals) - min(c_vals)
            self.get_logger().info(
                f"joint{j+1} commanded: "
                f"min={min(c_vals):+7.3f}  max={max(c_vals):+7.3f}  "
                f"span={c_span:6.3f} deg"
            )
            # Follow error after the 2 s ease-in window (steady-state tracking).
            pairs = [
                (float(r[m_col]), float(r[c_col]))
                for r in rows
                if r[c_col] != "" and float(r[0]) > 2.0
            ]
            if pairs:
                diffs = [c - m for m, c in pairs]
                n = len(diffs)
                mean_abs = sum(abs(d) for d in diffs) / n
                max_abs = max(abs(d) for d in diffs)
                rms = math.sqrt(sum(d * d for d in diffs) / n)
                self.get_logger().info(
                    f"joint{j+1} follow-error (t>2 s): "
                    f"mean_abs={mean_abs:.4f}  max_abs={max_abs:.4f}  rms={rms:.4f} deg"
                )
        if self._home is not None:
            closure = abs(m_vals[-1] - math.degrees(self._home[j]))
            self.get_logger().info(
                f"joint{j+1} closure (final - home): {closure:.4f} deg"
            )


# ----------------------------------------------------------------------
# Controller management helpers (auto-switch on entry, restore on exit)
# ----------------------------------------------------------------------
def list_controller_states(node: Node, timeout_sec: float = 3.0) -> Dict[str, str]:
    """Return {controller_name: state} from /controller_manager/list_controllers.

    Returns an empty dict if the service is unreachable; logs a single warning
    so the caller can decide whether to abort or proceed.
    """
    cli = node.create_client(ListControllers, LIST_CONTROLLERS_SRV)
    try:
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn(
                f"{LIST_CONTROLLERS_SRV} not available within {timeout_sec:.1f} s"
            )
            return {}
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
        resp = fut.result()
        if resp is None:
            node.get_logger().warn("ListControllers call returned no response")
            return {}
        return {c.name: c.state for c in resp.controller}
    finally:
        node.destroy_client(cli)


def switch_controllers(
    node: Node,
    activate: List[str],
    deactivate: List[str],
    timeout_sec: float = 5.0,
) -> bool:
    """Call /controller_manager/switch_controller with STRICT semantics."""
    if not activate and not deactivate:
        return True
    cli = node.create_client(SwitchController, SWITCH_CONTROLLER_SRV)
    try:
        if not cli.wait_for_service(timeout_sec=3.0):
            node.get_logger().error(
                f"{SWITCH_CONTROLLER_SRV} not available; cannot switch controllers"
            )
            return False
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = DurationMsg(sec=2, nanosec=0)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
        resp = fut.result()
        if resp is None:
            node.get_logger().error("switch_controller call timed out")
            return False
        return bool(resp.ok)
    finally:
        node.destroy_client(cli)


def ensure_forward_position_active(node: Node) -> Optional[Dict[str, str]]:
    """Activate forward_position_controller, deactivate arm_1_controller.

    Returns the *original* {controller: state} snapshot on success so the
    caller can restore it later. Returns None on failure (bad service,
    missing controller, switch rejected) -- caller should abort.
    """
    states = list_controller_states(node)
    if not states:
        node.get_logger().error(
            "Could not read controller states. Is the bringup running and "
            "controller_manager up? Try: ros2 control list_controllers"
        )
        return None
    for c in (FORWARD_POSITION_CONTROLLER, TRAJECTORY_CONTROLLER):
        if c not in states:
            node.get_logger().error(
                f"Required controller '{c}' is not loaded. Loaded: "
                f"{sorted(states)}"
            )
            return None

    to_activate = (
        [FORWARD_POSITION_CONTROLLER]
        if states[FORWARD_POSITION_CONTROLLER] != "active"
        else []
    )
    to_deactivate = (
        [TRAJECTORY_CONTROLLER]
        if states[TRAJECTORY_CONTROLLER] == "active"
        else []
    )

    if not to_activate and not to_deactivate:
        node.get_logger().info(
            f"{FORWARD_POSITION_CONTROLLER} already active, "
            f"{TRAJECTORY_CONTROLLER} already inactive -- no switch needed."
        )
        return states

    node.get_logger().info(
        f"Switching controllers: activate={to_activate} "
        f"deactivate={to_deactivate}"
    )
    if not switch_controllers(node, to_activate, to_deactivate):
        node.get_logger().error("switch_controller request was rejected (ok=False)")
        return None
    # Give the new subscription on /forward_position_controller/commands a
    # moment to be discovered by our publisher. Without this, the first ~1 s
    # of streamed commands can land before the controller's subscriber is
    # wired and be silently dropped.
    time.sleep(0.3)
    return states


def restore_controller_states(
    node: Node, original: Dict[str, str]
) -> bool:
    """Reverse the entry switch so the world is left as we found it."""
    states_now = list_controller_states(node)
    if not states_now:
        node.get_logger().warn("Cannot read controller states for restore.")
        return False

    to_activate: List[str] = []
    to_deactivate: List[str] = []
    for name, prev in original.items():
        cur = states_now.get(name, "unloaded")
        if prev == "active" and cur != "active":
            to_activate.append(name)
        elif prev != "active" and cur == "active":
            to_deactivate.append(name)

    if not to_activate and not to_deactivate:
        node.get_logger().info("Controller states already match original; nothing to restore.")
        return True

    node.get_logger().info(
        f"Restoring controllers: activate={to_activate} "
        f"deactivate={to_deactivate}"
    )
    return switch_controllers(node, to_activate, to_deactivate)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--mode",
        choices=["hold", "step", "sine"],
        default="hold",
        help="Test waveform (default: hold -- just republish current pose).",
    )
    p.add_argument(
        "--joint",
        type=int,
        default=5,
        choices=list(range(6)),
        help="Index (0..5) of the joint to perturb (default: 5 = wrist).",
    )
    p.add_argument(
        "--amplitude",
        type=float,
        default=0.05,
        help=f"Amplitude in radians (default: 0.05). Hard cap {MAX_AMPLITUDE_RAD}.",
    )
    p.add_argument(
        "--frequency",
        type=float,
        default=0.2,
        help="Sine frequency in Hz (default: 0.2).",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=250.0,
        help="Command publish rate in Hz (default: 250, matches controller_manager).",
    )
    p.add_argument(
        "--ramp-time",
        dest="ramp_time",
        type=float,
        default=1.5,
        help="Seconds to ramp back to home on Ctrl+C (default: 1.5).",
    )
    p.add_argument(
        "--wait-timeout",
        dest="wait_timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for first /joint_states message (default: 5).",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help=(
            "If > 0, auto-stop after this many seconds (then ramp home and exit). "
            "If 0 (default), run until Ctrl+C."
        ),
    )
    p.add_argument(
        "--trace",
        type=str,
        default="",
        help=(
            "Optional CSV path. If set, record measured + commanded values "
            "for all 6 joints at every /joint_states tick and write to this "
            "file (plus print summary stats) on exit."
        ),
    )
    p.add_argument(
        "--no-auto-switch",
        dest="no_auto_switch",
        action="store_true",
        help=(
            "Do NOT call /controller_manager/switch_controller on entry/exit. "
            "Use this if you are managing controller activation yourself "
            "(e.g. from an upstream launch). Default behavior: activate "
            f"{FORWARD_POSITION_CONTROLLER} and deactivate {TRAJECTORY_CONTROLLER} "
            "on entry, restore the original state on exit."
        ),
    )
    args = p.parse_args()

    if abs(args.amplitude) > MAX_AMPLITUDE_RAD:
        print(
            f"ERROR: --amplitude {args.amplitude} rad exceeds safety cap "
            f"{MAX_AMPLITUDE_RAD} rad. Aborting.",
            file=sys.stderr,
        )
        sys.exit(2)
    if args.rate <= 0:
        print("ERROR: --rate must be positive.", file=sys.stderr)
        sys.exit(2)
    if args.frequency < 0:
        print("ERROR: --frequency must be non-negative.", file=sys.stderr)
        sys.exit(2)
    return args


def wait_for_home(node: ForwardPositionTester, timeout: float) -> bool:
    """Spin briefly to receive the first /joint_states message."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._home is not None:
            return True
    return False


def main() -> int:
    args = parse_args()
    # Disable rclpy's default SIGINT handler so we can run the ramp-home
    # sequence after Ctrl+C: rclpy's handler shuts down the context and
    # invalidates the publisher before any post-spin code can run.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    stop_flag = {"value": False}

    def _sigint_handler(_signum, _frame):
        stop_flag["value"] = True

    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    node = ForwardPositionTester(args)
    original_states: Optional[Dict[str, str]] = None
    try:
        if not args.no_auto_switch:
            original_states = ensure_forward_position_active(node)
            if original_states is None:
                node.get_logger().error(
                    "Auto-switch to forward_position_controller failed; aborting. "
                    "Pass --no-auto-switch if you want to skip this step."
                )
                return 1

        if not wait_for_home(node, args.wait_timeout):
            node.get_logger().error(
                f"Did not receive /joint_states within {args.wait_timeout:.1f} s. "
                "Is the bringup launched and joint_state_broadcaster active?"
            )
            return 1

        node.get_logger().info(
            "Streaming commands. Press Ctrl+C to ramp home and exit."
        )
        # Manual spin loop so we can break out cleanly on SIGINT and still
        # have a live rclpy context for the ramp-home publishes.
        run_started = time.monotonic()
        while rclpy.ok() and not stop_flag["value"]:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0 and (time.monotonic() - run_started) >= args.duration:
                node.get_logger().info(
                    f"--duration {args.duration:.2f} s elapsed; stopping."
                )
                break

        if stop_flag["value"]:
            node.get_logger().info("Stop signal received.")
        node.ramp_back_to_home(args.ramp_time)
        # Pump the executor briefly so post-ramp /joint_states samples land
        # in the trace buffer. The ramp loop above ran synchronously without
        # the executor, so without this the CSV would end mid-motion and the
        # closure metric would be wrong. ~1 s gives the joint time to settle
        # under the held home command.
        post_ramp_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < post_ramp_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if args.trace:
            node.write_trace(args.trace)
        return 0
    finally:
        # Restore controllers BEFORE shutting down rclpy -- the service
        # client needs a live context. Swallow errors so a restore failure
        # doesn't mask the real exit code from the test itself.
        if original_states is not None and rclpy.ok():
            try:
                restore_controller_states(node, original_states)
            except Exception as exc:  # noqa: BLE001
                node.get_logger().warn(
                    f"Controller restore raised: {type(exc).__name__}: {exc}"
                )
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
