#!/usr/bin/env python3
"""spacemouse_servo — turn a 3Dconnexion SpaceMouse into Cartesian jog targets.

The SpaceMouse driver (``spacenav``) publishes a 6-DOF *velocity* twist; the
robot-side consumers in this workspace (``ikt_pose_commander`` and the FZI
``cartesian_motion_controller``) want a *moving pose target*. This node is the
bridge: it integrates the puck twist into a streaming ``PoseStamped`` target,
starting from the live end-effector pose captured over TF.

Pipeline (fixed-rate timer)::

    spacenav/twist (Twist)  ┐
    spacenav/joy   (Joy)    ┤── dead-man gate ── deadband+scale+clamp ──┐
    TF base<-tip (capture)  ┘                                           │
                                                                        ▼
                              integrate (tool|base frame)  ──▶  PoseStamped target

Safety model:

* **Dead-man gated.** No target is published unless the dead-man button is held
  (``deadman_mode: hold``) or toggled on (``deadman_mode: toggle``). On release
  the (optional) downstream commander is disabled.
* **Idle re-capture.** While disengaged the target is continuously reset to the
  current end-effector pose, so engaging never produces a jump.
* **Input staleness.** If no twist arrives within ``input_timeout`` the puck is
  treated as centred (zero velocity) — a disconnected SpaceMouse cannot drift
  the target.
* **Conservative limits.** Per-axis scales and hard speed clamps default low;
  the downstream commander still applies its own reachability / jump / speed
  gates. This node never commands the robot directly.

The output is a generic ``PoseStamped`` (remap ``~/target_pose`` to either
``ikt_pose_commander/target_pose`` or ``<fzi_ctrl>/target_frame``), so the node
is robot-agnostic.
"""
from __future__ import annotations

import json
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from spacemouse_teleop.twist_integrator import (integrate_pose,
                                                quat_from_rotvec, shape_twist)


class SpaceMouseServo(Node):
    def __init__(self) -> None:
        super().__init__("spacemouse_servo")

        # ---- I/O ----------------------------------------------------------
        self.declare_parameter("input_topic", "spacenav/twist")
        self.declare_parameter("joy_topic", "spacenav/joy")
        self.declare_parameter("target_pose_topic", "~/target_pose")
        # Where to publish DELTA poses in output_mode 'delta' (incremental
        # transforms the commander composes onto its internal goal).
        self.declare_parameter("target_delta_topic",
                               "ikt_pose_commander/target_delta")
        # output_mode: 'absolute' = integrate the puck into an absolute
        # PoseStamped on target_pose_topic (the classic path, needs TF capture);
        # 'delta' = stream per-tick incremental poses on target_delta_topic and
        # let the commander own the goal (no TF needed; snap on engage).
        self.declare_parameter("output_mode", "absolute")
        self.declare_parameter("base_frame", "")   # frame targets are expressed in
        self.declare_parameter("tip_frame", "")    # EE link to capture / jog
        self.declare_parameter("rate_hz", 50.0)

        # ---- shaping ------------------------------------------------------
        self.declare_parameter("linear_scale", [0.05, 0.05, 0.05])   # m/s per unit twist
        self.declare_parameter("angular_scale", [0.3, 0.3, 0.3])     # rad/s per unit twist
        self.declare_parameter("deadband_lin", 0.0)   # driver already deadbands
        self.declare_parameter("deadband_ang", 0.0)
        self.declare_parameter("max_linear_speed", 0.05)    # m/s hard clamp
        self.declare_parameter("max_angular_speed", 0.3)    # rad/s hard clamp
        self.declare_parameter("jog_frame", "tool")         # tool | base
        self.declare_parameter("input_timeout", 0.2)        # s — twist staleness

        # ---- buttons ------------------------------------------------------
        # Button indices follow the spacenav/joy buttons[] mapping documented in
        # the 3dconnexion_ros2 submodule. On the SpaceMouse Pro: index 0 = the
        # "1" function key, index 1 = the "2" function key (see that README's
        # "Buttons" table). These are read individually, so the device's
        # 3+-button ghosting does not affect the dead-man / speed keys.
        self.declare_parameter("deadman_button", 0)         # Pro "1" function key
        self.declare_parameter("deadman_mode", "hold")      # hold | toggle
        self.declare_parameter("button1_index", 1)          # Pro "2" function key
        self.declare_parameter("button1_action", "speed")   # speed | position_only | none
        self.declare_parameter("speed_scales", [0.25, 1.0, 2.0])

        # ---- downstream commander (optional) ------------------------------
        self.declare_parameter("enable_commander", True)
        self.declare_parameter("commander_enable_srv", "ikt_pose_commander/enable")
        self.declare_parameter("commander_disable_srv", "ikt_pose_commander/disable")
        # Called on engage in output_mode 'delta' to seed the commander's goal
        # onto the current EE pose (so deltas accumulate from there, no jump).
        self.declare_parameter("commander_snap_srv", "ikt_pose_commander/snap_target")

        gp = self.get_parameter
        self._base_frame = str(gp("base_frame").value or "")
        self._tip_frame = str(gp("tip_frame").value or "")
        if not self._base_frame or not self._tip_frame:
            raise ValueError(
                "spacemouse_servo requires 'base_frame' and 'tip_frame' to be "
                "set (the TF frames to capture/jog the end-effector between). "
                "Got base_frame=%r tip_frame=%r" % (self._base_frame, self._tip_frame))

        self._rate = max(1.0, float(gp("rate_hz").value))
        self._dt = 1.0 / self._rate
        self._lin_scale = self._vec3(gp("linear_scale").value, "linear_scale")
        self._ang_scale = self._vec3(gp("angular_scale").value, "angular_scale")
        self._db_lin = float(gp("deadband_lin").value)
        self._db_ang = float(gp("deadband_ang").value)
        self._max_lin = float(gp("max_linear_speed").value)
        self._max_ang = float(gp("max_angular_speed").value)
        self._jog_frame = str(gp("jog_frame").value).strip().lower()
        if self._jog_frame not in ("tool", "base"):
            raise ValueError("jog_frame must be 'tool' or 'base', got %r"
                             % self._jog_frame)
        self._output_mode = str(gp("output_mode").value).strip().lower()
        if self._output_mode not in ("absolute", "delta"):
            raise ValueError("output_mode must be 'absolute' or 'delta', got %r"
                             % self._output_mode)
        self._input_timeout = max(0.0, float(gp("input_timeout").value))

        self._deadman_button = int(gp("deadman_button").value)
        self._deadman_mode = str(gp("deadman_mode").value).strip().lower()
        if self._deadman_mode not in ("hold", "toggle", "none"):
            raise ValueError(
                "deadman_mode must be 'hold', 'toggle' or 'none', got %r"
                % self._deadman_mode)
        self._button1_index = int(gp("button1_index").value)
        self._button1_action = str(gp("button1_action").value).strip().lower()
        self._speed_scales = [float(s) for s in gp("speed_scales").value] or [1.0]

        self._enable_commander = bool(gp("enable_commander").value)
        target_topic = str(gp("target_pose_topic").value)
        delta_topic = str(gp("target_delta_topic").value)

        # ---- state (guarded by _lock) ------------------------------------
        self._lock = threading.Lock()
        self._twist_lin = np.zeros(3)
        self._twist_ang = np.zeros(3)
        self._twist_stamp = 0.0
        self._buttons: list[int] = []
        self._prev_deadman = False
        self._prev_button1 = False
        self._engaged = False
        self._sender_enabled = True   # master on/off gate (dashboard set_enabled)
        self._speed_idx = min(1, len(self._speed_scales) - 1)  # start at 1.0x, not slow preset
        self._position_only = False
        self._target_pos = None     # np.ndarray(3,) or None
        self._target_quat = None    # np.ndarray(4,) or None

        # ---- TF -----------------------------------------------------------
        import tf2_ros
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- pubs / subs / services --------------------------------------
        self._target_pub = self.create_publisher(PoseStamped, target_topic, 10)
        self._delta_pub = self.create_publisher(PoseStamped, delta_topic, 10)
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(Twist, str(gp("input_topic").value),
                                 self._on_twist, 10)
        self.create_subscription(Joy, str(gp("joy_topic").value),
                                 self._on_joy, 10)
        self._enable_cli = self.create_client(
            Trigger, str(gp("commander_enable_srv").value))
        self._disable_cli = self.create_client(
            Trigger, str(gp("commander_disable_srv").value))
        self._snap_cli = self.create_client(
            Trigger, str(gp("commander_snap_srv").value))
        # Master on/off for the whole teleop sender (toggled by the dashboard).
        self.create_service(SetBool, "~/set_enabled", self._on_set_enabled)

        self.create_timer(self._dt, self._on_timer)

        dm = ("none (touch-to-move, no button)" if self._deadman_mode == "none"
              else "button%d(%s)" % (self._deadman_button, self._deadman_mode))
        sink = (delta_topic if self._output_mode == "delta" else target_topic)
        self.get_logger().info(
            "spacemouse_servo up: %s<-%s, %.0f Hz, output_mode=%s, jog_frame=%s, "
            "lin<=%.3f m/s ang<=%.3f rad/s, deadman=%s, "
            "enable_commander=%s -> %s"
            % (self._base_frame, self._tip_frame, self._rate, self._output_mode,
               self._jog_frame, self._max_lin, self._max_ang, dm,
               self._enable_commander, sink))
        if self._deadman_mode == "none":
            self.get_logger().info(
                "ENGAGED by default (deadman_mode=none): touch the puck to move "
                "the target. Centre the puck to hold position.")
        else:
            self.get_logger().info(
                "DISENGAGED. %s the dead-man button to jog the robot."
                % ("Hold" if self._deadman_mode == "hold" else "Toggle"))

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _vec3(value, name):
        arr = np.asarray([float(v) for v in value], dtype=float)
        if arr.shape == (1,):
            arr = np.repeat(arr, 3)
        if arr.shape != (3,):
            raise ValueError("%s must have 1 or 3 elements, got %s"
                             % (name, list(value)))
        return arr

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_twist(self, msg: Twist) -> None:
        with self._lock:
            self._twist_lin = np.array(
                [msg.linear.x, msg.linear.y, msg.linear.z])
            self._twist_ang = np.array(
                [msg.angular.x, msg.angular.y, msg.angular.z])
            self._twist_stamp = self._now()

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._buttons = list(msg.buttons)

    # ------------------------------------------------------------------ #
    # TF capture
    # ------------------------------------------------------------------ #
    def _capture_pose(self) -> bool:
        """Set the target to the current EE pose via TF. Returns success."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._tip_frame, RclpyTime())
        except Exception as exc:  # noqa: BLE001 — tf2 raises several types
            self.get_logger().warning(
                "TF %s<-%s unavailable: %s" % (self._base_frame,
                                               self._tip_frame, exc),
                throttle_duration_sec=2.0)
            return False
        t = tf.transform.translation
        r = tf.transform.rotation
        self._target_pos = np.array([t.x, t.y, t.z])
        self._target_quat = np.array([r.x, r.y, r.z, r.w])
        return True

    # ------------------------------------------------------------------ #
    # Engagement / buttons
    # ------------------------------------------------------------------ #
    def _read_button(self, idx: int) -> bool:
        b = self._buttons
        return bool(0 <= idx < len(b) and b[idx])

    def _handle_button1(self) -> None:
        pressed = self._read_button(self._button1_index)
        if pressed and not self._prev_button1:
            if self._button1_action == "speed":
                self._speed_idx = (self._speed_idx + 1) % len(self._speed_scales)
                self.get_logger().info(
                    "speed scale -> %.2fx" % self._speed_scales[self._speed_idx])
            elif self._button1_action == "position_only":
                self._position_only = not self._position_only
                self.get_logger().info(
                    "position_only -> %s" % self._position_only)
        self._prev_button1 = pressed

    def _update_engagement(self) -> None:
        deadman = self._read_button(self._deadman_button)
        if not self._sender_enabled:
            new_engaged = False           # master OFF (dashboard) overrides everything
        elif self._deadman_mode == "none":
            new_engaged = True            # always engaged: touch-to-move
        elif self._deadman_mode == "hold":
            new_engaged = deadman
        else:  # toggle on rising edge
            new_engaged = self._engaged
            if deadman and not self._prev_deadman:
                new_engaged = not self._engaged
        self._prev_deadman = deadman

        self._handle_button1()

        if new_engaged and not self._engaged:
            if self._output_mode == "delta":
                # Delta mode: the commander owns the goal. Enable it, then snap
                # its target onto the current EE pose so deltas accumulate from
                # there (no TF capture needed on this side).
                if self._enable_commander:
                    self._call_trigger(self._enable_cli, "enable")
                self._call_trigger(self._snap_cli, "snap")
            else:
                # Absolute mode: capture the live EE pose, then enable so the
                # first integrated target equals the current pose (no jump).
                self._capture_pose()
                if self._enable_commander:
                    self._call_trigger(self._enable_cli, "enable")
            self.get_logger().info("ENGAGED — jogging active.")
        elif not new_engaged and self._engaged:
            if self._enable_commander:
                self._call_trigger(self._disable_cli, "disable")
            self.get_logger().info("DISENGAGED — motion stopped.")
        self._engaged = new_engaged

    def _call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warning(
                "commander %s service not available (%s)"
                % (label, client.srv_name), throttle_duration_sec=5.0)
            return
        client.call_async(Trigger.Request())

    def _on_set_enabled(self, req, resp):
        """Master on/off for the teleop sender (called by the dashboard).

        OFF -> the main loop disengages: it stops publishing target_pose and
        releases the commander (~/disable). ON -> it re-engages on the next tick,
        which re-captures the live pose (jumpless) and re-enables the commander.
        The actual transition runs in _update_engagement.
        """
        with self._lock:
            self._sender_enabled = bool(req.data)
        resp.success = True
        resp.message = "spacemouse sender " + ("ON" if req.data else "OFF")
        self.get_logger().info("set_enabled: " + resp.message)
        return resp

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def _on_timer(self) -> None:
        with self._lock:
            fresh = (self._now() - self._twist_stamp) <= self._input_timeout
            lin = self._twist_lin.copy() if fresh else np.zeros(3)
            ang = self._twist_ang.copy() if fresh else np.zeros(3)

            self._update_engagement()

            # Shaped Cartesian velocity (m/s, rad/s), common to both modes.
            speed = self._speed_scales[self._speed_idx]
            v_lin, v_ang = shape_twist(
                lin, ang, self._lin_scale * speed, self._ang_scale * speed,
                deadband_lin=self._db_lin, deadband_ang=self._db_ang,
                max_lin=self._max_lin, max_ang=self._max_ang)
            if self._position_only:
                v_ang = np.zeros(3)

            if self._output_mode == "delta":
                # Delta mode: the commander owns the goal. Stream per-tick
                # incremental poses ONLY while engaged; publish nothing (and
                # need no TF) while released, so the commander simply holds its
                # last goal and another source (e.g. the dashboard) may take over.
                if self._engaged:
                    self._publish_delta(v_lin, v_ang)
                self._publish_status(fresh)
                return

            # Absolute mode (classic): integrate the puck into an absolute target.
            if not self._engaged:
                # Keep the target glued to the live pose so engaging is jumpless.
                self._capture_pose()
                self._publish_status(fresh)
                return

            if self._target_pos is None and not self._capture_pose():
                # Engaged but no pose yet (TF not ready) — cannot jog safely.
                self._publish_status(fresh)
                return

            self._target_pos, self._target_quat = integrate_pose(
                self._target_pos, self._target_quat, v_lin, v_ang,
                self._dt, frame=self._jog_frame)

            self._publish_target()
            self._publish_status(fresh)

    def _publish_delta(self, v_lin, v_ang) -> None:
        """Publish ONE incremental pose (delta) for the commander to compose.

        Position = the translation increment ``v_lin * dt``; orientation = the
        rotation increment from ``v_ang * dt`` (identity quaternion = no
        rotation). ``frame_id`` records the frame the delta is expressed in
        (``base_frame`` for a base jog, ``tip_frame`` for a tool jog) for
        traceability; the commander applies it according to its ``delta_frame``.
        A centred puck streams identity deltas, which leave the goal unchanged.
        """
        dp = v_lin * self._dt
        dq = quat_from_rotvec(v_ang * self._dt)   # (x, y, z, w)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (self._base_frame if self._jog_frame == "base"
                               else self._tip_frame)
        msg.pose.position.x = float(dp[0])
        msg.pose.position.y = float(dp[1])
        msg.pose.position.z = float(dp[2])
        msg.pose.orientation.x = float(dq[0])
        msg.pose.orientation.y = float(dq[1])
        msg.pose.orientation.z = float(dq[2])
        msg.pose.orientation.w = float(dq[3])
        self._delta_pub.publish(msg)

    def _publish_target(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.pose.position.x = float(self._target_pos[0])
        msg.pose.position.y = float(self._target_pos[1])
        msg.pose.position.z = float(self._target_pos[2])
        msg.pose.orientation.x = float(self._target_quat[0])
        msg.pose.orientation.y = float(self._target_quat[1])
        msg.pose.orientation.z = float(self._target_quat[2])
        msg.pose.orientation.w = float(self._target_quat[3])
        self._target_pub.publish(msg)

    def _publish_status(self, twist_fresh: bool) -> None:
        status = {
            "sender_enabled": self._sender_enabled,
            "engaged": self._engaged,
            "output_mode": self._output_mode,
            "jog_frame": self._jog_frame,
            "speed_scale": self._speed_scales[self._speed_idx],
            "position_only": self._position_only,
            "has_target": self._target_pos is not None,
            "twist_fresh": bool(twist_fresh),
            "base_frame": self._base_frame,
            "tip_frame": self._tip_frame,
        }
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpaceMouseServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
