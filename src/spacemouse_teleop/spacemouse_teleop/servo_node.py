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
from std_srvs.srv import Trigger

from spacemouse_teleop.twist_integrator import integrate_pose, shape_twist


class SpaceMouseServo(Node):
    def __init__(self) -> None:
        super().__init__("spacemouse_servo")

        # ---- I/O ----------------------------------------------------------
        self.declare_parameter("input_topic", "spacenav/twist")
        self.declare_parameter("joy_topic", "spacenav/joy")
        self.declare_parameter("target_pose_topic", "~/target_pose")
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
        self.declare_parameter("deadman_button", 0)
        self.declare_parameter("deadman_mode", "hold")      # hold | toggle
        self.declare_parameter("button1_index", 1)
        self.declare_parameter("button1_action", "speed")   # speed | position_only | none
        self.declare_parameter("speed_scales", [0.25, 1.0, 2.0])

        # ---- downstream commander (optional) ------------------------------
        self.declare_parameter("enable_commander", True)
        self.declare_parameter("commander_enable_srv", "ikt_pose_commander/enable")
        self.declare_parameter("commander_disable_srv", "ikt_pose_commander/disable")

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
        self._input_timeout = max(0.0, float(gp("input_timeout").value))

        self._deadman_button = int(gp("deadman_button").value)
        self._deadman_mode = str(gp("deadman_mode").value).strip().lower()
        if self._deadman_mode not in ("hold", "toggle"):
            raise ValueError("deadman_mode must be 'hold' or 'toggle', got %r"
                             % self._deadman_mode)
        self._button1_index = int(gp("button1_index").value)
        self._button1_action = str(gp("button1_action").value).strip().lower()
        self._speed_scales = [float(s) for s in gp("speed_scales").value] or [1.0]

        self._enable_commander = bool(gp("enable_commander").value)
        target_topic = str(gp("target_pose_topic").value)

        # ---- state (guarded by _lock) ------------------------------------
        self._lock = threading.Lock()
        self._twist_lin = np.zeros(3)
        self._twist_ang = np.zeros(3)
        self._twist_stamp = 0.0
        self._buttons: list[int] = []
        self._prev_deadman = False
        self._prev_button1 = False
        self._engaged = False
        self._speed_idx = 0
        self._position_only = False
        self._target_pos = None     # np.ndarray(3,) or None
        self._target_quat = None    # np.ndarray(4,) or None

        # ---- TF -----------------------------------------------------------
        import tf2_ros
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- pubs / subs / services --------------------------------------
        self._target_pub = self.create_publisher(PoseStamped, target_topic, 10)
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(Twist, str(gp("input_topic").value),
                                 self._on_twist, 10)
        self.create_subscription(Joy, str(gp("joy_topic").value),
                                 self._on_joy, 10)
        self._enable_cli = self.create_client(
            Trigger, str(gp("commander_enable_srv").value))
        self._disable_cli = self.create_client(
            Trigger, str(gp("commander_disable_srv").value))

        self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            "spacemouse_servo up: %s<-%s, %.0f Hz, jog_frame=%s, "
            "lin<=%.3f m/s ang<=%.3f rad/s, deadman=button%d(%s), "
            "enable_commander=%s -> %s"
            % (self._base_frame, self._tip_frame, self._rate, self._jog_frame,
               self._max_lin, self._max_ang, self._deadman_button,
               self._deadman_mode, self._enable_commander, target_topic))
        self.get_logger().info(
            "DISENGAGED. Hold/toggle the dead-man button to jog the robot.")

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
        if self._deadman_mode == "hold":
            new_engaged = deadman
        else:  # toggle on rising edge
            new_engaged = self._engaged
            if deadman and not self._prev_deadman:
                new_engaged = not self._engaged
        self._prev_deadman = deadman

        self._handle_button1()

        if new_engaged and not self._engaged:
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

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def _on_timer(self) -> None:
        with self._lock:
            fresh = (self._now() - self._twist_stamp) <= self._input_timeout
            lin = self._twist_lin.copy() if fresh else np.zeros(3)
            ang = self._twist_ang.copy() if fresh else np.zeros(3)

            self._update_engagement()

            if not self._engaged:
                # Keep the target glued to the live pose so engaging is jumpless.
                self._capture_pose()
                self._publish_status(fresh)
                return

            if self._target_pos is None and not self._capture_pose():
                # Engaged but no pose yet (TF not ready) — cannot jog safely.
                self._publish_status(fresh)
                return

            speed = self._speed_scales[self._speed_idx]
            v_lin, v_ang = shape_twist(
                lin, ang, self._lin_scale * speed, self._ang_scale * speed,
                deadband_lin=self._db_lin, deadband_ang=self._db_ang,
                max_lin=self._max_lin, max_ang=self._max_ang)
            if self._position_only:
                v_ang = np.zeros(3)

            self._target_pos, self._target_quat = integrate_pose(
                self._target_pos, self._target_quat, v_lin, v_ang,
                self._dt, frame=self._jog_frame)

            self._publish_target()
            self._publish_status(fresh)

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
            "engaged": self._engaged,
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
