"""Teleoperate the Duco GCR5_910 from Alicia-D leader arm joint states.

Subscribes to /arm_joint_state (ArmJointState) from the Alicia driver,
maps the 6 leader joint angles to the follower robot's joints, and
publishes them either as a JointTrajectory on the JTC's command topic
(``command_mode == 'trajectory'``) or as a std_msgs/Float64MultiArray on
the forward_command_controller's command topic (``command_mode ==
'forward_position'``, the default).

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
        # Velocity feedforward: estimate leader joint velocity from
        # successive samples and send it with each trajectory point so
        # JTC interpolates a smooth velocity profile instead of
        # stop-and-go ramps. Cap protects against finite-difference
        # noise / large timestamp gaps.
        # Only used in trajectory mode; ignored in forward_position mode.
        self.declare_parameter("velocity_feedforward", True)
        self.declare_parameter("max_velocity", 3.0)  # rad/s, per joint

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

        if len(self._joint_names) != 6:
            raise ValueError(
                f"alicia_teleop expects 6 joint_names, got {len(self._joint_names)}: "
                f"{self._joint_names}")
        if self._joint_scale.shape != (6,) or self._joint_offset.shape != (6,):
            raise ValueError(
                "joint_scale and joint_offset must each have 6 elements")

        # State
        self._leader_joints = None  # latest leader arm joint angles (6,)
        self._leader_velocity = np.zeros(6, dtype=np.float64)  # rad/s, estimated
        self._prev_leader_joints = None
        self._prev_leader_stamp = None  # rclpy time when last sample arrived
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
        leader_joints = np.array([
            msg.joint1, msg.joint2, msg.joint3,
            msg.joint4, msg.joint5, msg.joint6,
        ], dtype=np.float64)

        # Estimate leader velocity from the timestamp delta. Guards
        # against zero/huge dt (timer skew, first sample, etc.).
        now = self.get_clock().now()
        if (self._vff_enabled and self._prev_leader_joints is not None
                and self._prev_leader_stamp is not None):
            dt = (now - self._prev_leader_stamp).nanoseconds * 1e-9
            if 1e-4 < dt < 0.1:
                vel = (leader_joints - self._prev_leader_joints) / dt
                # Clamp to vmax to suppress finite-difference spikes.
                vel = np.clip(vel, -self._vmax, self._vmax)
                # Light low-pass (alpha=0.5) for further noise rejection.
                self._leader_velocity = 0.5 * self._leader_velocity + 0.5 * vel
            else:
                self._leader_velocity[:] = 0.0
        self._prev_leader_joints = leader_joints
        self._prev_leader_stamp = now
        self._leader_joints = leader_joints

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
                self.get_logger().info("Teleop ENGAGED (leader SYNC)")
        else:
            if self._active:
                self._active = False
                self.get_logger().info(
                    "Teleop DISENGAGED (leader SYNC released)"
                )

    def _timer_cb(self):
        """Send joint command from leader arm data."""
        if self._leader_joints is None or not self._active:
            return

        # Map leader joints to follower joints. Velocity is mapped by
        # the same scale (sign flips track) and offset is irrelevant
        # since it's a constant.
        target = self._leader_joints * self._joint_scale + self._joint_offset

        if self._command_mode == "trajectory":
            target_vel = self._leader_velocity * self._joint_scale

            traj = JointTrajectory()
            traj.joint_names = self._joint_names

            point = JointTrajectoryPoint()
            point.positions = target.tolist()
            if self._vff_enabled:
                point.velocities = target_vel.tolist()
            sec = int(self._traj_time)
            nanosec = int((self._traj_time - sec) * 1e9)
            point.time_from_start = DurationMsg(sec=sec, nanosec=nanosec)
            traj.points = [point]
            self._traj_pub.publish(traj)
        else:  # forward_position
            # FZI-style: just hand the latest position to the controller.
            # forward_command_controller writes whatever we send straight
            # to the position command interface on every controller tick.
            msg = Float64MultiArray()
            msg.data = target.tolist()
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
