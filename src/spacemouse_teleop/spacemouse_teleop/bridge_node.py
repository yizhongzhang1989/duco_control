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
        self.declare_parameter("output_pose_topic",
                               "/ikt_pose_commander/target_pose")
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
        self._out_topic = str(gp("output_pose_topic").value)
        self._status_topic = str(gp("commander_status_topic").value)
        self._base_frame = str(gp("base_frame").value)
        self._tip_param = str(gp("tip_frame").value or "")
        self._follow_enable = bool(gp("follow_commander_enable").value)

        # ---- state ------------------------------------------------------
        self._sm_latest = None        # (pos[3], quat[4]) latest curr_pose
        self._robot_anchor = None     # (pos, quat) robot EE captured at engage
        self._sm_anchor = None        # (pos, quat) curr_pose captured at engage
        self._enabled = False
        self._tip_from_status = ""

        # ---- TF ---------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- interfaces -------------------------------------------------
        self._pub = self.create_publisher(PoseStamped, self._out_topic, 10)
        self.create_subscription(
            PoseStamped, self._in_topic, self._on_curr_pose, 10)
        self.create_subscription(
            String, self._status_topic, self._on_status, 10)
        # Re-capture the anchor onto the current EE without disabling (e.g. to
        # recentre after jogging to the edge of comfortable reach).
        self.create_service(Trigger, "~/reanchor", self._srv_reanchor)

        self.get_logger().info(
            "spacemouse_teleop up: %s -> %s (base=%s tip=%s "
            "follow_commander_enable=%s)"
            % (self._in_topic, self._out_topic, self._base_frame,
               self._tip_param or "<from commander status>",
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
        """Capture the robot EE + the current curr_pose as the jog anchor."""
        with self._lock:
            sm = self._sm_latest
        if sm is None:
            return False, "no SpaceMouse curr_pose received yet"
        ee = self._robot_ee_now()
        if ee is None:
            return False, "robot EE pose unavailable (TF / tip_frame not ready)"
        with self._lock:
            self._robot_anchor = ee
            self._sm_anchor = (np.array(sm[0], dtype=float),
                               np.array(sm[1], dtype=float))
        self.get_logger().info(
            "anchored: EE [%.3f %.3f %.3f] tip=%s -- jogging from here"
            % (ee[0][0], ee[0][1], ee[0][2], self._tip_frame()))
        return True, "anchored to current EE"

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
        # On the enable rising edge, anchor at the current EE so jogging starts
        # jump-free; on disable, drop the anchor so we stop feeding targets.
        if self._follow_enable:
            if enabled and not was:
                self._reanchor()
            elif was and not enabled:
                with self._lock:
                    self._robot_anchor = None
                    self._sm_anchor = None

    def _on_curr_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        sm = (np.array([p.x, p.y, p.z], dtype=float),
              _quat_normalize([o.w, o.x, o.y, o.z]))
        with self._lock:
            self._sm_latest = sm
            enabled = self._enabled
            ra = self._robot_anchor
            sa = self._sm_anchor
        if self._follow_enable and not enabled:
            return
        if ra is None or sa is None:
            # Not anchored yet: a TF lag at the enable edge, or standalone mode
            # (follow_commander_enable=False) on the first pose. Retry now.
            ok, _msg = self._reanchor()
            if not ok:
                return
            with self._lock:
                ra = self._robot_anchor
                sa = self._sm_anchor

        # World/base-frame composition: the puck's motion SINCE the anchor,
        # applied onto the robot anchor. translation adds directly; rotation is a
        # world-frame (left-multiplied) increment.
        dp = sm[0] - sa[0]
        dq = _quat_mul(sm[1], _quat_conj(sa[1]))
        tgt_pos = ra[0] + dp
        tgt_quat = _quat_normalize(_quat_mul(dq, ra[1]))

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._base_frame
        out.pose.position.x = float(tgt_pos[0])
        out.pose.position.y = float(tgt_pos[1])
        out.pose.position.z = float(tgt_pos[2])
        out.pose.orientation.w = float(tgt_quat[0])
        out.pose.orientation.x = float(tgt_quat[1])
        out.pose.orientation.y = float(tgt_quat[2])
        out.pose.orientation.z = float(tgt_quat[3])
        self._pub.publish(out)


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
