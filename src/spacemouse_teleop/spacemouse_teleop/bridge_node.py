#!/usr/bin/env python3
"""spacemouse_teleop -- translate a SpaceMouse pose into commander targets.

The 3Dconnexion ``spacemouse`` package's ``pose_node`` publishes an ABSOLUTE,
accumulated puck pose on ``/spacemouse/curr_pose`` (``geometry_msgs/PoseStamped``)
in its own abstract origin frame. ``ikt_pose_commander`` wants an ABSOLUTE target
pose on ``~/target_pose`` expressed in a robot frame. This node is the single
adapter between the two, coupling neither side to the other:

* It captures an ANCHOR when teleop engages (by default, when the commander is
  enabled): the robot's current end-effector pose (via TF ``base_frame`` ->
  ``tip_frame``) and the SpaceMouse's current ``curr_pose``.
* For every subsequent ``curr_pose`` it composes the puck's motion SINCE the
  anchor onto the robot anchor and publishes the result as an absolute target::

      target = robot_anchor  (+)  (sm_now  -  sm_anchor)

  so the first target equals the current EE (no jump) and the arm jogs from
  there. The puck motion is treated as a WORLD/base-frame increment, matching the
  ``pose_node`` ``integration_frame: world`` (base-frame jog) setting.

Because every output is a FULL absolute pose, the commander always tracks the
LATEST one and intermediate poses may be dropped freely (no delta accumulation,
so transport delay / dropped messages never corrupt the goal).
"""
import json
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from ikt_interfaces.msg import PoseCommand
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

import tf2_ros


def _quat_normalize(q):
    """Return ``q`` (w, x, y, z) as a unit quaternion (identity if ~zero)."""
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    return np.array([1.0, 0.0, 0.0, 0.0]) if n < 1e-12 else q / n


def _quat_mul(a, b):
    """Hamilton product ``a (x) b`` for (w, x, y, z) quaternions."""
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _quat_conj(q):
    """Conjugate (inverse for a unit quaternion) of (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q)
    return np.array([w, -x, -y, -z])


class SpaceMousePoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("spacemouse_teleop")
        self._lock = threading.Lock()

        # ---- parameters -------------------------------------------------
        self.declare_parameter("input_pose_topic", "/spacemouse/curr_pose")
        self.declare_parameter("output_command_topic",
                               "/ikt_pose_commander/pose_command")
        # set_pose_topic: pose_node resets its accumulated curr_pose to whatever
        # we publish here. We push the current EE on engage / link change so the
        # next curr_pose == EE -> jog starts jump-free (the pose_node does the
        # anchoring; the bridge stays a thin translator).
        self.declare_parameter("set_pose_topic", "/spacemouse/set_pose")
        self.declare_parameter("commander_status_topic",
                               "/ikt_pose_commander/status")
        # base_frame: the robot root frame. Used BOTH as the TF anchor base and
        # as the output pose header.frame_id, so the commander resolves it as
        # identity into its solver frame (no robot frames leak to the SpaceMouse).
        self.declare_parameter("base_frame", "base_link")
        # tip_frame: the controlled end-effector link. Empty = take it live from
        # the commander status (``controlled_frame``), so it always matches what
        # the commander drives.
        self.declare_parameter("tip_frame", "")
        # follow_commander_enable: anchor on the commander's enable rising edge
        # and only feed targets while it is enabled. False = anchor once on the
        # first pose and feed continuously (standalone, no commander status).
        self.declare_parameter("follow_commander_enable", True)

        gp = self.get_parameter
        self._in_topic = str(gp("input_pose_topic").value)
        self._cmd_topic = str(gp("output_command_topic").value)
        self._set_topic = str(gp("set_pose_topic").value)
        self._status_topic = str(gp("commander_status_topic").value)
        self._base_frame = str(gp("base_frame").value)
        self._tip_param = str(gp("tip_frame").value or "")
        self._follow_enable = bool(gp("follow_commander_enable").value)

        # ---- state ------------------------------------------------------
        self._enabled = False
        self._tip_from_status = ""
        self._last_tip = ""            # control link last set_pose'd at

        # ---- TF ---------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- interfaces -------------------------------------------------
        self._pub = self.create_publisher(PoseCommand, self._cmd_topic, 10)
        self._set_pub = self.create_publisher(PoseStamped, self._set_topic, 10)
        self.create_subscription(
            PoseStamped, self._in_topic, self._on_curr_pose, 10)
        self.create_subscription(
            String, self._status_topic, self._on_status, 10)
        # Re-anchor: set pose_node's curr_pose to the current EE (no jump).
        self.create_service(Trigger, "~/reanchor", self._srv_reanchor)

        self.get_logger().info(
            "spacemouse_teleop up: %s -> %s (set=%s base=%s tip=%s "
            "follow_commander_enable=%s)"
            % (self._in_topic, self._cmd_topic, self._set_topic,
               self._base_frame, self._tip_param or "<from commander status>",
               self._follow_enable))

    # ------------------------------------------------------------------ #
    def _tip_frame(self) -> str:
        return self._tip_param or self._tip_from_status

    def _robot_ee_now(self):
        """(pos, quat_wxyz) of the controlled frame in base_frame, or None."""
        tip = self._tip_frame()
        if not tip:
            return None
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, tip, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "TF %s<-%s unavailable: %r" % (self._base_frame, tip, exc),
                throttle_duration_sec=2.0)
            return None
        tr = t.transform.translation
        ro = t.transform.rotation
        return (np.array([tr.x, tr.y, tr.z]),
                _quat_normalize([ro.w, ro.x, ro.y, ro.z]))

    def _reanchor(self):
        """Set pose_node's curr_pose to the current EE so jogging is jump-free.

        Pushes the controlled frame's current pose (TF base<-tip) to the
        pose_node ``set_pose`` topic; the next curr_pose then equals the EE, so
        the translated target starts where the arm is. pose_node owns the
        accumulation -- the bridge just triggers the reset.
        """
        ee = self._robot_ee_now()
        if ee is None:
            return False, "robot EE pose unavailable (TF / tip_frame not ready)"
        pos, quat = ee
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._base_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            float(pos[0]), float(pos[1]), float(pos[2]))
        m.pose.orientation.w = float(quat[0])
        m.pose.orientation.x = float(quat[1])
        m.pose.orientation.y = float(quat[2])
        m.pose.orientation.z = float(quat[3])
        self._set_pub.publish(m)
        self.get_logger().info(
            "set_pose: EE [%.3f %.3f %.3f] tip=%s -- jogging from here"
            % (pos[0], pos[1], pos[2], self._tip_frame()))
        return True, "set_pose to current EE"

    def _srv_reanchor(self, request, response):
        ok, msg = self._reanchor()
        response.success = ok
        response.message = msg
        return response

    # ------------------------------------------------------------------ #
    def _on_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        enabled = bool(d.get("enabled", False))
        tip = str(d.get("controlled_frame", "") or "")
        with self._lock:
            self._tip_from_status = tip
            was = self._enabled
            self._enabled = enabled
        cur_tip = self._tip_frame()
        # set_pose on the enable rising edge OR when the control link changed,
        # so the next translated pose == EE (no jump on engage or link switch).
        if self._follow_enable and enabled and (not was or cur_tip != self._last_tip):
            if self._reanchor()[0]:
                self._last_tip = cur_tip

    def _on_curr_pose(self, msg: PoseStamped) -> None:
        if self._follow_enable and not self._enabled:
            return
        # Forward pose ONLY; leave control_link/frame_link empty so the commander
        # reuses whatever owns the link (config / dashboard) -- the bridge never
        # fights the dashboard over the control link. Link changes re-anchor via
        # set_pose (on enable / status link change) instead.
        c = PoseCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.has_pose = True
        c.pose = msg.pose
        self._pub.publish(c)


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMousePoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
