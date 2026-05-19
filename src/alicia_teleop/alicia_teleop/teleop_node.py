"""Teleoperate the Duco GCR5_910 from Alicia-D leader arm joint states.

Subscribes to /arm_joint_state (ArmJointState) from the Alicia driver,
maps the 6 leader joint angles to the follower robot's joints, and
publishes them either as a JointTrajectory on the JTC's command topic
(``command_mode == 'trajectory'``) or as a std_msgs/Float64MultiArray on
the forward_command_controller's command topic (``command_mode ==
'forward_position'``, the default).

Per-joint velocity and acceleration limits are enforced by a rate-limited
command interpolator. On every engagement, ``q_cmd`` is seeded from the
robot's actual pose (read from ``/joint_states``) and then ramped toward
the leader target with bounded velocity (``max_velocity``) and bounded
acceleration (``max_acceleration``). This prevents the "current pose
differs too much from target" safety stop that the robot driver raises
when SYNC engages with the leader far from the follower.

By default this node also activates the controller required for the
configured mode and deactivates its sibling at startup, and restores the
original controller states on shutdown -- so a fresh bringup followed by
``ros2 launch alicia_teleop alicia_teleop.launch.py`` is enough to start
teleoperating. Set ``auto_switch_controller:=false`` to opt out.

The leader-side calibration (direction flip / zero offset / continuous
unwrap per joint) is handled inside the Alicia driver itself via its
``joint_config`` YAML; by the time the angles reach this node they are
already in the follower robot's joint frame. ``joint_scale`` /
``joint_offset`` here are an optional second-stage tweak.

Default joint mapping (Alicia -> Duco GCR5_910):
  joint1 -> arm_1_joint_1
  joint2 -> arm_1_joint_2
  joint3 -> arm_1_joint_3
  joint4 -> arm_1_joint_4
  joint5 -> arm_1_joint_5
  joint6 -> arm_1_joint_6

Gating: ``ArmJointState.but1`` and ``but2`` are latched physical-button
states published by the Alicia driver (see alicia_leader/README.md):
  * ``but1`` -- lock state  (0 = released, 1 = locked)
  * ``but2`` -- sync state  (0 = unsync,   1 = sync)
Engagement policy:
  * sync == 1                    -> ENGAGE (regardless of lock).
  * sync == 0 and lock == 1      -> transparent (keep current state); the
                                    leader joints are frozen so the follower
                                    just holds its last commanded pose.
  * sync == 0 and lock == 0      -> DISENGAGE.
This lets the user briefly lock the leader (e.g. to take a break) without
losing the teleop link.
"""

import numpy as np
import rclpy
import signal
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration as DurationMsg
from controller_manager_msgs.srv import ListControllers, SwitchController
from alicia_duo_leader_driver.msg import ArmJointState


DEFAULT_JOINT_NAMES = [
    "arm_1_joint_1",
    "arm_1_joint_2",
    "arm_1_joint_3",
    "arm_1_joint_4",
    "arm_1_joint_5",
    "arm_1_joint_6",
]
DEFAULT_TRAJECTORY_TOPIC = "/arm_1_controller/joint_trajectory"
DEFAULT_FORWARD_TOPIC = "/forward_position_controller/commands"
DEFAULT_LEADER_TOPIC = "/arm_joint_state"

# Controller names matched to each command_mode. Keep in sync with the
# duco_gcr5_910_moveit_config submodule's ros2_controllers_hardware.yaml.
TRAJECTORY_CONTROLLER = "arm_1_controller"
FORWARD_POSITION_CONTROLLER = "forward_position_controller"
LIST_CONTROLLERS_SRV = "/controller_manager/list_controllers"
SWITCH_CONTROLLER_SRV = "/controller_manager/switch_controller"


class AliciaTeleop(Node):
    def __init__(self):
        super().__init__("alicia_teleop")

        # Parameters
        self.declare_parameter("rate", 100.0)  # Hz — command publish rate
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        # 'forward_position' (DEFAULT) -> publish std_msgs/Float64MultiArray
        # to the forward_command_controller, which streams positions
        # straight to the position command interface every controller tick
        # (no spline, no time_from_start). Same write pattern as FZI's
        # cartesian_*_controller family.
        # 'trajectory' -> publish trajectory_msgs/JointTrajectory to JTC.
        self.declare_parameter("command_mode", "forward_position")
        # Activate the controller required for `command_mode` and
        # deactivate its sibling at startup; restore on shutdown. Set
        # False if you are managing controller activation yourself.
        self.declare_parameter("auto_switch_controller", True)
        self.declare_parameter("trajectory_topic", DEFAULT_TRAJECTORY_TOPIC)
        self.declare_parameter("forward_position_topic", DEFAULT_FORWARD_TOPIC)
        self.declare_parameter("leader_topic", DEFAULT_LEADER_TOPIC)
        self.declare_parameter("joint_scale", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("trajectory_time", 0.05)  # seconds — time_from_start per point (trajectory mode only)
        # Velocity feedforward: send the rate-limited command velocity
        # (v_cmd, see _timer_cb) with each trajectory point so JTC's
        # spline matches the commanded velocity profile instead of
        # stop-and-go ramping between successive position points.
        # Only used in trajectory mode; ignored in forward_position mode.
        self.declare_parameter("velocity_feedforward", True)
        # Per-joint hard caps on the rate-limited command interpolator.
        # Together they bound how aggressively the follower closes the
        # gap between the robot's current pose and the (possibly far)
        # leader target -- so engaging SYNC when the leader is far from
        # the follower ramps in smoothly instead of tripping the robot's
        # "position deviation too large" safety stop. Both must be > 0.
        self.declare_parameter("max_velocity", 3.0)       # rad/s, per joint
        self.declare_parameter("max_acceleration", 10.0)  # rad/s^2, per joint

        rate = self.get_parameter("rate").value
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._command_mode = str(self.get_parameter("command_mode").value)
        if self._command_mode not in ("trajectory", "forward_position"):
            raise ValueError(
                f"alicia_teleop: unknown command_mode '{self._command_mode}'. "
                f"Expected 'trajectory' or 'forward_position'.")
        self._auto_switch_controller = bool(
            self.get_parameter("auto_switch_controller").value)
        trajectory_topic = self.get_parameter("trajectory_topic").value
        forward_topic = self.get_parameter("forward_position_topic").value
        leader_topic = self.get_parameter("leader_topic").value
        self._joint_scale = np.array(self.get_parameter("joint_scale").value, dtype=np.float64)
        self._joint_offset = np.array(self.get_parameter("joint_offset").value, dtype=np.float64)
        self._traj_time = self.get_parameter("trajectory_time").value
        self._vff_enabled = bool(self.get_parameter("velocity_feedforward").value)
        self._vmax = float(self.get_parameter("max_velocity").value)
        self._amax = float(self.get_parameter("max_acceleration").value)
        self._rate = float(rate)
        self._dt = 1.0 / self._rate

        if self._vmax <= 0.0 or self._amax <= 0.0:
            raise ValueError(
                f"alicia_teleop: max_velocity and max_acceleration must be "
                f"positive, got vmax={self._vmax}, amax={self._amax}")

        if len(self._joint_names) != 6:
            raise ValueError(
                f"alicia_teleop expects 6 joint_names, got {len(self._joint_names)}: "
                f"{self._joint_names}")
        if self._joint_scale.shape != (6,) or self._joint_offset.shape != (6,):
            raise ValueError(
                "joint_scale and joint_offset must each have 6 elements")

        # State
        self._leader_joints = None      # latest leader arm joint angles (6,)
        self._follower_joints = None    # latest follower joint positions (6,)
                                        # from /joint_states (matched by name).
        self._q_cmd = None              # rate-limited commanded position (6,)
        self._v_cmd = None              # rate-limited commanded velocity (6,)
        self._active = False
        self._last_lock = None   # last seen but1, for transition logging
        self._last_sync = None   # last seen but2, for transition logging

        # Subscribe to Alicia leader arm
        self.create_subscription(
            ArmJointState,
            leader_topic,
            self._leader_cb,
            10,
        )

        # Subscribe to follower /joint_states so the rate limiter can
        # seed its starting position from the actual robot pose on each
        # SYNC engagement -- this is what prevents the "current pose
        # differs too much from target" safety stop. Match by name:
        # joint_state_broadcaster does NOT guarantee the canonical
        # [j1..j6] ordering (we've observed [j1, j3, j2, j4, j5, j6]).
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
        )

        # Publisher for joint commands. In trajectory mode this is a
        # JointTrajectory to JTC; in forward_position mode this is a
        # Float64MultiArray to forward_command_controller, which writes
        # straight to the position command interface every controller
        # tick (same write pattern as FZI's cartesian controllers).
        if self._command_mode == "trajectory":
            self._command_topic = trajectory_topic
            self._traj_pub = self.create_publisher(
                JointTrajectory, self._command_topic, 10)
            self._fwd_pub = None
        else:  # forward_position
            self._command_topic = forward_topic
            self._traj_pub = None
            self._fwd_pub = self.create_publisher(
                Float64MultiArray, self._command_topic, 10)

        # Timer to send commands at fixed rate
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info(
            f"Alicia teleop started in '{self._command_mode}' mode at "
            f"{rate:.0f} Hz, "
            f"traj_time={self._traj_time:.3f}s, "
            f"vff={'on' if self._vff_enabled else 'off'} "
            f"vmax={self._vmax:.2f} rad/s, "
            f"amax={self._amax:.2f} rad/s^2, "
            f"leader_topic={leader_topic}, "
            f"command_topic={self._command_topic}, "
            f"joint_names={self._joint_names}"
        )
        if self._command_mode == "forward_position":
            self.get_logger().info(
                "forward_position mode: positions are written directly to "
                "the position command interface every controller tick "
                "(no JTC spline, no time_from_start). "
                "trajectory_time / velocity_feedforward are ignored.")
        self.get_logger().info(
            "Teleop gating: SYNC (but2=1) engages; otherwise disengaged. "
            "Press the SYNC button on the leader to engage."
        )

    def _leader_cb(self, msg: ArmJointState):
        """Store latest leader arm joints and update active state."""
        self._leader_joints = np.array([
            msg.joint1, msg.joint2, msg.joint3,
            msg.joint4, msg.joint5, msg.joint6,
        ], dtype=np.float64)

        lock = bool(msg.but1)
        sync = bool(msg.but2)

        # Log button-state transitions so the user can see why teleop
        # isn't streaming when the leader isn't in SYNC. Overheat is
        # tracked via ArmJointState.overheat and surfaced by the
        # dashboard; we intentionally do NOT log it here to avoid
        # spam when the leader oscillates in and out of a thermal state.
        if lock != self._last_lock or sync != self._last_sync:
            self.get_logger().info(
                f"Leader buttons: lock={'ON' if lock else 'off'}  "
                f"sync={'ON' if sync else 'off'}"
            )
            self._last_lock = lock
            self._last_sync = sync

        # Engagement policy: SYNC engages teleop, anything else disengages.
        # LOCK (but1) is informational only -- now that the buttons are
        # reliable there's no need for a transparent / hold-last state.
        if sync:
            if not self._active:
                self._active = True
                # _q_cmd / _v_cmd are deliberately left as None until
                # the next timer tick seeds them from /joint_states.
                # That keeps the rate limiter starting from the robot's
                # actual pose every time SYNC is pressed.
                self.get_logger().info("Teleop ENGAGED (leader SYNC)")
        else:
            if self._active:
                self._active = False
                self._q_cmd = None
                self._v_cmd = None
                self.get_logger().info(
                    "Teleop DISENGAGED (leader SYNC released)"
                )

    def _joint_state_cb(self, msg: JointState):
        """Capture follower joint positions for rate-limiter seeding.

        Matches by joint name -- joint_state_broadcaster does not
        guarantee the canonical [j1..j6] order. Silently ignores
        messages that don't carry all of our joints (e.g. partial
        publishers).
        """
        pos_by_name = dict(zip(msg.name, msg.position))
        try:
            self._follower_joints = np.array(
                [pos_by_name[n] for n in self._joint_names],
                dtype=np.float64,
            )
        except KeyError:
            return

    def _timer_cb(self):
        """Send joint command from leader arm data, rate-limited.

        Per-joint velocity and acceleration caps are enforced on a
        commanded position (``_q_cmd``) that is seeded from the robot's
        actual pose at engagement time and then ramped toward the
        leader target. The published value is ``_q_cmd``, NOT the raw
        leader target -- so a large leader-vs-follower gap becomes a
        smooth bounded approach instead of an instantaneous jump.
        """
        if self._leader_joints is None or not self._active:
            return

        # Map leader joints to follower joints. ``joint_offset`` is a
        # constant bias, ``joint_scale`` includes any sign flips for
        # mirrored axes.
        target = self._leader_joints * self._joint_scale + self._joint_offset

        # Seed the rate limiter on the first tick after engagement so
        # we start from the robot's actual pose (no jump). If
        # /joint_states hasn't been heard yet we throttle-warn and hold
        # off publishing until it arrives -- otherwise we'd have to
        # fall back to the leader target and lose the safety guarantee.
        if self._q_cmd is None:
            if self._follower_joints is None:
                self.get_logger().warn(
                    "Engaged but /joint_states not received yet; "
                    "holding off commands until the robot's joint "
                    "states arrive. Is joint_state_broadcaster active?",
                    throttle_duration_sec=2.0,
                )
                return
            self._q_cmd = self._follower_joints.copy()
            self._v_cmd = np.zeros(6, dtype=np.float64)
            gap = float(np.max(np.abs(target - self._q_cmd)))
            self.get_logger().info(
                f"Rate limiter seeded from /joint_states; "
                f"max initial gap = {gap:.3f} rad "
                f"(vmax={self._vmax:.2f} rad/s, "
                f"amax={self._amax:.2f} rad/s^2 will close it smoothly)"
            )

        # Per-joint velocity- and acceleration-limited tracking using
        # a trapezoidal profile. The "stopping velocity" cap
        # sqrt(2*amax*|err|) is what makes this overshoot-free: it is
        # the maximum speed from which we can still decelerate to zero
        # at the target given amax, so v_cmd starts braking BEFORE we
        # reach the target instead of trying (and failing) to brake on
        # the last tick.
        #   err   = target - q_cmd
        #   v_des = sign(err) * min(|err|/dt, sqrt(2*amax*|err|), vmax)
        #   dv    = clip(v_des - v_cmd, -amax*dt, amax*dt)
        #   v_cmd = clip(v_cmd + dv, -vmax, vmax)
        #   q_cmd = q_cmd + v_cmd * dt
        dt = self._dt
        err = target - self._q_cmd
        abs_err = np.abs(err)
        v_stop = np.sqrt(2.0 * self._amax * abs_err)
        v_des = np.sign(err) * np.minimum(
            np.minimum(abs_err / dt, v_stop), self._vmax)
        dv = np.clip(
            v_des - self._v_cmd, -self._amax * dt, self._amax * dt)
        self._v_cmd = np.clip(self._v_cmd + dv, -self._vmax, self._vmax)
        self._q_cmd = self._q_cmd + self._v_cmd * dt

        if self._command_mode == "trajectory":
            traj = JointTrajectory()
            traj.joint_names = self._joint_names

            point = JointTrajectoryPoint()
            point.positions = self._q_cmd.tolist()
            if self._vff_enabled:
                # vff = the rate-limited command velocity (matches the
                # position derivative we are actually publishing) so
                # JTC's spline tracks our profile without overshoot.
                point.velocities = self._v_cmd.tolist()
            sec = int(self._traj_time)
            nanosec = int((self._traj_time - sec) * 1e9)
            point.time_from_start = DurationMsg(sec=sec, nanosec=nanosec)
            traj.points = [point]
            self._traj_pub.publish(traj)
        else:  # forward_position
            # FZI-style: just hand the latest rate-limited position to
            # the controller. forward_command_controller writes whatever
            # we send straight to the position command interface on
            # every controller tick.
            msg = Float64MultiArray()
            msg.data = self._q_cmd.tolist()
            self._fwd_pub.publish(msg)


# ----------------------------------------------------------------------
# Controller management helpers (auto-switch on entry, restore on exit)
# ----------------------------------------------------------------------
def _list_controller_states(node: Node, timeout_sec: float = 3.0):
    """Return ``{controller_name: state}`` or an empty dict on failure."""
    cli = node.create_client(ListControllers, LIST_CONTROLLERS_SRV)
    try:
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn(
                f"{LIST_CONTROLLERS_SRV} not available within "
                f"{timeout_sec:.1f} s; auto-switch will be skipped.")
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


def _switch_controllers(node: Node, activate, deactivate,
                        timeout_sec: float = 5.0) -> bool:
    """Call /controller_manager/switch_controller with STRICT semantics."""
    if not activate and not deactivate:
        return True
    cli = node.create_client(SwitchController, SWITCH_CONTROLLER_SRV)
    try:
        if not cli.wait_for_service(timeout_sec=3.0):
            node.get_logger().error(
                f"{SWITCH_CONTROLLER_SRV} not available; cannot switch.")
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


def _controllers_for_mode(mode: str):
    """Return (wanted_active, wanted_inactive) controllers for ``mode``."""
    if mode == "forward_position":
        return FORWARD_POSITION_CONTROLLER, TRAJECTORY_CONTROLLER
    return TRAJECTORY_CONTROLLER, FORWARD_POSITION_CONTROLLER


def _ensure_controller_for_mode(node: Node, mode: str):
    """Activate the controller required for ``mode``; return original states.

    On any failure, logs a clear error and returns ``None`` so the caller
    can decide whether to abort. Returning a snapshot dict signals success
    and is consumed by :func:`_restore_controller_states` on exit.
    """
    wanted_active, wanted_inactive = _controllers_for_mode(mode)
    states = _list_controller_states(node)
    if not states:
        return None
    for name in (wanted_active, wanted_inactive):
        if name not in states:
            node.get_logger().error(
                f"Required controller '{name}' is not loaded. "
                f"Loaded: {sorted(states)}. Is the duco bringup running?")
            return None

    to_activate = [wanted_active] if states[wanted_active] != "active" else []
    to_deactivate = (
        [wanted_inactive] if states[wanted_inactive] == "active" else [])

    if not to_activate and not to_deactivate:
        node.get_logger().info(
            f"Controllers already in the right state for "
            f"command_mode='{mode}'; no switch needed.")
        return states

    node.get_logger().info(
        f"Switching controllers for command_mode='{mode}': "
        f"activate={to_activate} deactivate={to_deactivate}")
    if not _switch_controllers(node, to_activate, to_deactivate):
        node.get_logger().error("switch_controller request was rejected.")
        return None
    return states


def _restore_controller_states(node: Node, original) -> bool:
    """Reverse the entry switch so the world is left as we found it."""
    states_now = _list_controller_states(node)
    if not states_now:
        node.get_logger().warn("Cannot read controller states for restore.")
        return False
    to_activate, to_deactivate = [], []
    for name, prev in original.items():
        cur = states_now.get(name, "unloaded")
        if prev == "active" and cur != "active":
            to_activate.append(name)
        elif prev != "active" and cur == "active":
            to_deactivate.append(name)
    if not to_activate and not to_deactivate:
        node.get_logger().info(
            "Controller states already match original; nothing to restore.")
        return True
    node.get_logger().info(
        f"Restoring controllers: activate={to_activate} "
        f"deactivate={to_deactivate}")
    return _switch_controllers(node, to_activate, to_deactivate)


def main(args=None):
    # Disable rclpy's default SIGINT handler so we can run the controller
    # restore AFTER Ctrl+C: rclpy's handler shuts down the context and
    # invalidates the service client before the finally block can use it.
    rclpy.init(args=args,
               signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    stop_flag = {"value": False}

    def _sigint_handler(_signum, _frame):
        stop_flag["value"] = True

    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    node = AliciaTeleop()
    original_states = None
    try:
        if node._auto_switch_controller:
            original_states = _ensure_controller_for_mode(
                node, node._command_mode)
            if original_states is None:
                node.get_logger().warn(
                    "Auto-switch failed; the teleop will still run, but the "
                    "controller it publishes to may not be active. Either "
                    "start the duco bringup and run "
                    "'ros2 control switch_controllers ...' manually, or pass "
                    "auto_switch_controller:=false to silence this warning.")

        # Manual spin loop so we can break out cleanly on SIGINT and keep
        # the rclpy context live for the controller restore in `finally`.
        while rclpy.ok() and not stop_flag["value"]:
            rclpy.spin_once(node, timeout_sec=0.1)
        if stop_flag["value"]:
            node.get_logger().info("Stop signal received; shutting down.")
    finally:
        # Restore controllers BEFORE shutting down rclpy -- the service
        # client needs a live context.
        if original_states is not None and rclpy.ok():
            try:
                _restore_controller_states(node, original_states)
            except Exception as exc:  # noqa: BLE001
                node.get_logger().warn(
                    f"Controller restore raised: "
                    f"{type(exc).__name__}: {exc}")
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
