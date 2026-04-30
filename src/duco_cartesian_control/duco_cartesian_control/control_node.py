"""Orchestrator for FZI's ``cartesian_force_controller``.

The Cartesian admittance hot path is owned by FZI's
``cartesian_force_controller`` ros2_control plugin -- a C++ controller
that runs inside ``controller_manager`` and writes joint position
commands directly to the hardware interface (~250 Hz).  This node
provides the operational glue around it:

* an **engage / disengage** UX (``Trigger`` services) that atomically
  switches ``arm_1_controller`` (the ``JointTrajectoryController``) and
  ``cartesian_force_controller`` via the ``controller_manager``'s
  ``switch_controller`` service;
* a **wrench relay** that republishes the gravity-compensated
  ``WrenchStamped`` from the FT sensor pipeline (BEST_EFFORT) onto
  FZI's RELIABLE ``ft_sensor_wrench`` topic so FZI's solver sees it;
* a **zero ``target_wrench`` heartbeat** at a configurable rate, which
  drives FZI's controller into pure free-drive (minimise the operator
  wrench);
* a **safety supervisor** that monitors FT staleness, joint-state
  staleness, and force / torque limits, auto-disengaging on any trip;
* a **status topic** (``~/state``, ``std_msgs/String`` with a JSON
  payload) so external tooling such as
  :package:`cartesian_controller_dashboard` can render the current
  engaged / tripped / idle state without an in-process API.

This node is self-contained: the system runs end-to-end without any
dashboard or external UI.  Use it with the companion launch file
:file:`cartesian_control.launch.py`, which also pre-loads FZI's plugin
into the controller_manager (inactive).

Operator interfaces::

    /duco_cartesian_control/engage      (std_srvs/srv/Trigger)
    /duco_cartesian_control/disengage   (std_srvs/srv/Trigger)
    /duco_cartesian_control/state       (std_msgs/String, JSON)
"""

from __future__ import annotations

import json
import sys
import threading
import time
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------
_PARAM_DECLARATIONS: List[Tuple[str, object]] = [
    # connectivity ---------------------------------------------------------
    ("wrench_topic",          "/duco_ft_sensor/wrench_compensated"),
    ("joint_states_topic",    "/joint_states"),
    ("controller_manager_ns", "/controller_manager"),
    ("engaged_default",       False),
    # FZI controller wiring ------------------------------------------------
    ("fzi_controller_name",     "cartesian_force_controller"),
    ("fzi_jtc_controller_name", "arm_1_controller"),
    ("fzi_ft_topic",            "/cartesian_force_controller/ft_sensor_wrench"),
    ("fzi_target_topic",        "/cartesian_force_controller/target_wrench"),
    ("fzi_target_frame",        "link_6"),
    ("fzi_target_rate_hz",      10.0),
    ("fzi_service_timeout_sec", 2.0),
    # supervisor loop ------------------------------------------------------
    ("loop_rate_hz",            50.0),
    # state-publish loop ---------------------------------------------------
    ("state_publish_rate_hz",   5.0),
    # safety ---------------------------------------------------------------
    ("max_wrench_force",          80.0),
    ("max_wrench_torque",         10.0),
    ("engage_max_joint_velocity", 0.05),  # rad/s; refuse engage above this
    ("ft_stale_after",            0.25),
    ("joint_states_stale_after",  0.25),
]


class CartesianControlNode(Node):
    """ROS 2 orchestrator that wraps FZI's ``cartesian_force_controller``."""

    # safety / supervisor knobs that are refreshed live by
    # ``_on_param_change`` (must all be >= 0).
    _SAFETY_KNOBS = (
        "max_wrench_force", "max_wrench_torque",
        "engage_max_joint_velocity",
        "ft_stale_after", "joint_states_stale_after",
    )

    def __init__(self) -> None:
        super().__init__("duco_cartesian_control")
        for name, default in _PARAM_DECLARATIONS:
            self.declare_parameter(name, default)
        self._read_params()

        # ---- state ------------------------------------------------------
        self._lock = threading.RLock()
        self._engaged: bool = False
        self._trip_reason: str = ""
        self._last_wrench: Optional[np.ndarray] = None
        self._last_wrench_mono: Optional[float] = None
        self._last_q_mono: Optional[float] = None
        self._last_qdot_max: Optional[float] = None  # max |qdot| over all joints

        # Live parameter callbacks for safety knobs.
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---- callback group ---------------------------------------------
        # Engage/disengage handlers issue synchronous switch_controller
        # calls; they need to be on a Reentrant group + a
        # MultiThreadedExecutor so the response can be dispatched while
        # the handler is still on the stack.  See ``main()``.
        self._cbgroup = ReentrantCallbackGroup()

        # ---- pubs / subs ------------------------------------------------
        wrench_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._sub_wrench = self.create_subscription(
            WrenchStamped, self._wrench_topic, self._on_wrench, wrench_qos)
        self._sub_js = self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states, 50)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub_fzi_ft = self.create_publisher(
            WrenchStamped, self._fzi_ft_topic, reliable_qos)
        self._pub_fzi_target = self.create_publisher(
            WrenchStamped, self._fzi_target_topic, reliable_qos)

        # State topic: TRANSIENT_LOCAL so late subscribers (e.g. a
        # dashboard launched after the orchestrator) see the most
        # recent state immediately.
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_state = self.create_publisher(
            String, "~/state", state_qos)

        # ---- service clients (controller_manager) -----------------------
        self._fzi_switch_cli = self.create_client(
            SwitchController,
            f"{self._controller_manager_ns}/switch_controller",
            callback_group=self._cbgroup)

        # ---- services (engage / disengage) ------------------------------
        self._srv_engage = self.create_service(
            Trigger, "~/engage", self._cb_engage,
            callback_group=self._cbgroup)
        self._srv_disengage = self.create_service(
            Trigger, "~/disengage", self._cb_disengage,
            callback_group=self._cbgroup)

        # ---- timers -----------------------------------------------------
        rate = max(self._fzi_target_rate_hz, 0.1)
        self._fzi_target_timer = self.create_timer(
            1.0 / rate, self._publish_fzi_target_zero,
            callback_group=self._cbgroup)
        self._loop_timer = self.create_timer(
            1.0 / max(self._loop_rate_hz, 1.0), self._on_supervisor_tick,
            callback_group=self._cbgroup)
        self._state_timer = self.create_timer(
            1.0 / max(self._state_publish_rate_hz, 0.1),
            self._publish_state, callback_group=self._cbgroup)

        self.get_logger().info(
            f"orchestrating FZI: engage will switch "
            f"{self._fzi_jtc_controller_name!r} -> "
            f"{self._fzi_controller_name!r}; relaying wrench "
            f"{self._wrench_topic!r} -> {self._fzi_ft_topic!r}; "
            f"target_wrench=0 on {self._fzi_target_topic!r} @ {rate:.0f} Hz")
        self.get_logger().info(
            f"engage via `ros2 service call /duco_cartesian_control/engage "
            f"std_srvs/srv/Trigger`")

    # ------------------------------------------------------------------
    # parameter loading + live updates
    # ------------------------------------------------------------------
    def _read_params(self) -> None:
        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self._wrench_topic = str(gp("wrench_topic"))
        self._joint_states_topic = str(gp("joint_states_topic"))
        self._controller_manager_ns = str(gp("controller_manager_ns")).rstrip("/")
        self._engaged_default = bool(gp("engaged_default"))

        self._fzi_controller_name = str(gp("fzi_controller_name"))
        self._fzi_jtc_controller_name = str(gp("fzi_jtc_controller_name"))
        self._fzi_ft_topic = str(gp("fzi_ft_topic"))
        self._fzi_target_topic = str(gp("fzi_target_topic"))
        self._fzi_target_frame = str(gp("fzi_target_frame"))
        self._fzi_target_rate_hz = float(gp("fzi_target_rate_hz"))
        self._fzi_service_timeout = float(gp("fzi_service_timeout_sec"))

        self._loop_rate_hz = float(gp("loop_rate_hz"))
        self._state_publish_rate_hz = float(gp("state_publish_rate_hz"))

        self._max_wrench_force = float(gp("max_wrench_force"))
        self._max_wrench_torque = float(gp("max_wrench_torque"))
        self._engage_max_joint_velocity = float(gp("engage_max_joint_velocity"))
        self._ft_stale_after = float(gp("ft_stale_after"))
        self._joint_states_stale_after = float(gp("joint_states_stale_after"))

    def _on_param_change(self, params) -> SetParametersResult:
        # Validate non-negative safety knobs.
        for p in params:
            if p.name in self._SAFETY_KNOBS and float(p.value) < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name!r} must be >= 0")
        # Apply.
        for p in params:
            if p.name == "max_wrench_force":
                self._max_wrench_force = float(p.value)
            elif p.name == "max_wrench_torque":
                self._max_wrench_torque = float(p.value)
            elif p.name == "engage_max_joint_velocity":
                self._engage_max_joint_velocity = float(p.value)
            elif p.name == "ft_stale_after":
                self._ft_stale_after = float(p.value)
            elif p.name == "joint_states_stale_after":
                self._joint_states_stale_after = float(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def _on_wrench(self, msg: WrenchStamped) -> None:
        # Forward to FZI's RELIABLE input verbatim -- FZI does its own
        # conditioning (no LP / deadband on our side).
        self._pub_fzi_ft.publish(msg)
        # Cache for the safety supervisor.
        with self._lock:
            self._last_wrench = np.array([
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
            ])
            self._last_wrench_mono = time.monotonic()

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            self._last_q_mono = time.monotonic()
            if msg.velocity:
                try:
                    self._last_qdot_max = float(
                        max(abs(v) for v in msg.velocity))
                except (TypeError, ValueError):
                    self._last_qdot_max = None
            else:
                self._last_qdot_max = None
            # Auto-engage on the first complete state pair, if requested.
            if (self._engaged_default and not self._engaged
                    and self._trip_reason == ""
                    and self._last_wrench is not None):
                self._engage_locked("(auto-engaged from engaged_default=true)")

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def _cb_engage(self, request: Trigger.Request, response: Trigger.Response):
        with self._lock:
            ok, msg = self._engage_locked("(operator request)")
        response.success = ok
        response.message = msg
        return response

    def _cb_disengage(self, request: Trigger.Request, response: Trigger.Response):
        ok, why = self._disengage()
        response.success = ok
        response.message = why if not ok else "disengaged"
        return response

    def _engage_locked(self, why: str) -> Tuple[bool, str]:
        """Switch JTC -> FZI atomically; caller must hold ``self._lock``.

        Preconditions: at least one wrench + one joint_states must have
        arrived, and (if ``engage_max_joint_velocity`` > 0) the arm
        must be effectively stationary.
        """
        if self._last_wrench is None:
            return False, "no wrench received yet"
        if self._last_q_mono is None:
            return False, "no joint_states received yet"

        # Stationary-at-engage gate.
        if (self._engage_max_joint_velocity > 0.0
                and self._last_qdot_max is not None
                and self._last_qdot_max > self._engage_max_joint_velocity):
            return False, (
                f"arm is moving (max |qdot|={self._last_qdot_max:.3f} rad/s "
                f"> {self._engage_max_joint_velocity:.3f}); wait for the "
                "robot to settle before engaging")

        # Drop the lock for the (potentially blocking) service call --
        # the multi-threaded executor will dispatch the response.
        self._lock.release()
        try:
            ok, switch_why = self._switch_to_fzi_sync()
        finally:
            self._lock.acquire()
        if not ok:
            return False, f"controller switch failed: {switch_why}"
        self._engaged = True
        self._trip_reason = ""
        self.get_logger().info(
            f"engaged {why}; {self._fzi_jtc_controller_name!r} deactivated, "
            f"{self._fzi_controller_name!r} active. FZI is now writing "
            "position commands directly to the hardware interface; this "
            "node is the safety supervisor.")
        # Publish the new state immediately so dashboards reflect the
        # transition without waiting for the next periodic publish.
        self._publish_state()
        return True, "engaged"

    def _disengage(self) -> Tuple[bool, str]:
        """Switch FZI -> JTC atomically.  Best-effort idempotent."""
        with self._lock:
            engaged = self._engaged
        if engaged:
            ok, why = self._switch_to_jtc_sync()
            if not ok:
                self.get_logger().error(
                    f"disengage: controller switch failed ({why}); "
                    "FZI may still be active -- arm could remain compliant!")
                return False, why
        with self._lock:
            self._engaged = False
            self._trip_reason = ""
        self.get_logger().info("disengaged (operator request)")
        self._publish_state()
        return True, ""

    # ------------------------------------------------------------------
    # FZI orchestration helpers (also used by tests)
    # ------------------------------------------------------------------
    def _publish_fzi_target_zero(self) -> None:
        """Send an all-zero ``WrenchStamped`` to FZI's setpoint topic.

        FZI minimises (target - measured), so an identically zero
        target means FZI tries to drive the operator-applied wrench to
        zero -- pure free-drive.  Always running so the topic stays
        warm; harmless when FZI is inactive.
        """
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._fzi_target_frame
        # All wrench fields default to 0.0 (explicit for clarity).
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = 0.0
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self._pub_fzi_target.publish(msg)

    def _service_call_sync(self, client, request, timeout_s: float):
        """Issue ``call_async`` and block until done or timeout.

        Relies on :class:`MultiThreadedExecutor` (set up in :func:`main`)
        + :class:`ReentrantCallbackGroup` so the response can be
        delivered while this method's caller is still on the stack.
        Returns the service response, or ``None`` on timeout / error.
        """
        if not client.wait_for_service(timeout_sec=min(0.5, timeout_s)):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception:  # noqa: BLE001
            return None

    def _switch_to_fzi_sync(self) -> Tuple[bool, str]:
        """Atomically deactivate the JTC and activate FZI's controller."""
        return self._switch_controllers_sync(
            activate=[self._fzi_controller_name],
            deactivate=[self._fzi_jtc_controller_name])

    def _switch_to_jtc_sync(self) -> Tuple[bool, str]:
        """Atomically deactivate FZI and activate the JTC."""
        return self._switch_controllers_sync(
            activate=[self._fzi_jtc_controller_name],
            deactivate=[self._fzi_controller_name])

    def _switch_controllers_sync(self, activate: List[str],
                                 deactivate: List[str]) -> Tuple[bool, str]:
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        # STRICT: fail the whole call if either side fails (no half-
        # switched state).  ``activate_asap`` lets the new controller
        # take effect on the next update cycle.
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(seconds=self._fzi_service_timeout).to_msg()
        resp = self._service_call_sync(
            self._fzi_switch_cli, req, self._fzi_service_timeout)
        if resp is None:
            return False, (f"controller_manager/switch_controller did not "
                           f"respond within {self._fzi_service_timeout:.1f} s "
                           f"(is the controller_manager running?)")
        if not resp.ok:
            return False, ("switch_controller returned ok=false "
                           f"(activate={activate}, deactivate={deactivate}; "
                           "is FZI's plugin loaded and configured? "
                           "Try `ros2 control list_controllers`)")
        return True, "ok"

    # ------------------------------------------------------------------
    # safety supervisor
    # ------------------------------------------------------------------
    def _on_supervisor_tick(self) -> None:
        """Stale-data + force-limit checks; auto-disengages on any trip."""
        with self._lock:
            if not self._engaged:
                return
            if self._last_wrench is None or self._last_wrench_mono is None:
                return
            now = time.monotonic()
            ft_age = now - self._last_wrench_mono
            q_age = now - (self._last_q_mono or 0.0)
            wrench = self._last_wrench.copy()
            ft_stale = self._ft_stale_after
            q_stale = self._joint_states_stale_after
            f_lim = self._max_wrench_force
            t_lim = self._max_wrench_torque

        if ft_age > ft_stale:
            self._trip(f"FT topic stale ({ft_age:.2f} s)")
            return
        if q_age > q_stale:
            self._trip(f"joint_states stale ({q_age:.2f} s)")
            return
        f_mag = float(np.linalg.norm(wrench[:3]))
        t_mag = float(np.linalg.norm(wrench[3:]))
        if f_mag > f_lim:
            self._trip(f"force {f_mag:.1f} N exceeds limit {f_lim} N")
            return
        if t_mag > t_lim:
            self._trip(f"torque {t_mag:.2f} Nm exceeds limit {t_lim} Nm")
            return

    def _trip(self, reason: str) -> None:
        """Disengage due to a safety violation.

        Reverses the controller switch so the JTC takes over and the
        arm holds its current pose; if that switch fails we log loudly
        because the arm may otherwise stay compliant.
        """
        with self._lock:
            was_engaged = self._engaged
            self._engaged = False
            self._trip_reason = reason
        self.get_logger().error(f"safety trip: {reason}; disengaged")
        if was_engaged:
            ok, why = self._switch_to_jtc_sync()
            if not ok:
                self.get_logger().error(
                    f"safety trip: controller switch back to "
                    f"{self._fzi_jtc_controller_name!r} FAILED ({why}); "
                    "FZI is still active -- arm may continue compliant "
                    "motion! Operator must intervene manually.")
        self._publish_state()

    # ------------------------------------------------------------------
    # state topic
    # ------------------------------------------------------------------
    def _publish_state(self) -> None:
        """Publish a JSON snapshot of orchestrator state.

        Wrench / joint_states freshness are observed by the dashboard
        directly via its own subscriptions; only orchestrator-owned
        flags travel on this topic.
        """
        with self._lock:
            now = time.monotonic()
            ft_age = (now - self._last_wrench_mono
                      if self._last_wrench_mono is not None else None)
            q_age = (now - self._last_q_mono
                     if self._last_q_mono is not None else None)
            payload = {
                "engaged":              self._engaged,
                "trip_reason":          self._trip_reason,
                "wrench_topic":         self._wrench_topic,
                "joint_states_topic":   self._joint_states_topic,
                "fzi_controller":       self._fzi_controller_name,
                "fzi_jtc":              self._fzi_jtc_controller_name,
                "fzi_ft_topic":         self._fzi_ft_topic,
                "fzi_target_topic":     self._fzi_target_topic,
                "controller_manager_ns": self._controller_manager_ns,
                "loop_rate_hz":         self._loop_rate_hz,
                "ft_age":               ft_age,
                "joint_states_age":     q_age,
                "joint_velocity_max":   self._last_qdot_max,
                "ft_ok": (ft_age is not None
                          and ft_age <= self._ft_stale_after),
                "joint_states_ok": (q_age is not None
                                    and q_age <= self._joint_states_stale_after),
                "limits": {
                    "max_wrench_force":          self._max_wrench_force,
                    "max_wrench_torque":         self._max_wrench_torque,
                    "engage_max_joint_velocity": self._engage_max_joint_velocity,
                    "ft_stale_after":            self._ft_stale_after,
                    "joint_states_stale_after":  self._joint_states_stale_after,
                },
            }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._pub_state.publish(msg)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = CartesianControlNode()
    # MultiThreadedExecutor + ReentrantCallbackGroup is required so the
    # synchronous switch_controller call inside the engage Trigger
    # handler can be serviced by a different executor thread; a single-
    # threaded executor would deadlock.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort: if FZI is still engaged at shutdown, hand the
        # joints back to the JTC so the arm is in a sane state for the
        # next session.
        try:
            if node._engaged:
                node._switch_to_jtc_sync()
        except Exception:  # noqa: BLE001
            pass
        executor.shutdown()
        node.destroy_node()
        # rclpy's default SIGINT handler already shuts the context down
        # before the KeyboardInterrupt reaches us; calling shutdown()
        # again would raise 'rcl_shutdown already called'.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
