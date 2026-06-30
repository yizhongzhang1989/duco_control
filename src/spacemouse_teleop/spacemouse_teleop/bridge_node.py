#!/usr/bin/env python3
"""spacemouse_teleop -- translate a SpaceMouse pose into commander targets.

The 3Dconnexion ``spacemouse`` package's ``pose_node`` publishes an ABSOLUTE,
accumulated puck pose on ``/spacemouse/curr_pose`` (``geometry_msgs/PoseStamped``).
``ikt_pose_commander`` wants an ABSOLUTE target pose on ``~/target_pose`` expressed
in a robot frame. This node is the single adapter between the two:

* It forwards EVERY ``curr_pose`` 1:1 as a target on ``~/target_pose`` (frame_id =
  ``base_frame``) -- nothing is dropped, so slow/gentle jogs reach the robot too.
* It RESETS the puck -- pushes the robot's current EE (via TF ``base_frame`` ->
  ``tip_frame``) to the pose_node's ``set_pose`` so ``curr_pose`` snaps to the EE,
  dropping samples until it lands -- whenever it must NOT forward the raw pose: on
  enable, on a control-link switch, AND on a curr_pose DISCONTINUITY (a big jump
  between samples, e.g. the pose_node restarting at the origin). That last guard is
  critical: forwarding a stale/identity pose would teleport the robot, and the
  commander does NOT jump-reject in FPC rate mode (-> self-collision). These resets
  are the ONLY case the bridge ever drops a sample.
* It can HAND OFF control: ``~/set_forwarding`` (SetBool) gates forwarding so an
  operator switches between SpaceMouse control (ON) and dashboard-gizmo control
  (OFF -> bridge silent, dashboard owns target_pose). The latched ``~/forwarding``
  (Bool) topic mirrors the state.

Because every output is a FULL absolute pose, the commander always tracks the
LATEST one (no delta accumulation, so transport delay never corrupts the goal).
"""
import json
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

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
        self.declare_parameter("target_pose_topic",
                               "/ikt_pose_commander/target_pose")
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
        # start_forwarding: whether the bridge forwards curr_pose -> target_pose
        # at startup. Toggled at runtime via ``~/set_forwarding`` (SetBool) so an
        # operator can hand control between the SpaceMouse and the dashboard
        # gizmo (both drive target_pose); state mirrored on ``~/forwarding``.
        self.declare_parameter("start_forwarding", True)

        gp = self.get_parameter
        self._in_topic = str(gp("input_pose_topic").value)
        self._cmd_topic = str(gp("target_pose_topic").value)
        self._set_topic = str(gp("set_pose_topic").value)
        self._status_topic = str(gp("commander_status_topic").value)
        self._base_frame = str(gp("base_frame").value)
        self._tip_param = str(gp("tip_frame").value or "")
        self._follow_enable = bool(gp("follow_commander_enable").value)
        self._forward_enabled = bool(gp("start_forwarding").value)

        # ---- state ------------------------------------------------------
        self._enabled = False
        self._tip_from_status = ""
        self._last_tip = ""            # control link last set_pose'd at
        # The bridge forwards EVERY curr_pose 1:1, EXCEPT while RESETTING: on
        # enable / control-link switch / a curr_pose DISCONTINUITY (e.g. the
        # pose_node restarting at the origin) it drops samples and re-anchors
        # curr_pose to the current EE, resuming only once curr_pose == EE. This
        # is what stops a stale/identity pose from teleporting the robot (the
        # commander does NOT jump-reject in FPC rate mode -> self-collision).
        self._resetting = self._follow_enable   # anchor before forwarding
        self._last_curr = None         # last curr_pose seen (jump detection)
        self._last_reanchor = 0.0      # throttle set_pose re-issue while resetting
        self._jump_thresh = 0.10       # m: bigger curr_pose step = teleport, drop
        self._anchor_tol = 0.01        # m: "curr_pose anchored to EE" tolerance

        # ---- TF ---------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- interfaces -------------------------------------------------
        self._pub = self.create_publisher(PoseStamped, self._cmd_topic, 10)
        self._set_pub = self.create_publisher(PoseStamped, self._set_topic, 10)
        self.create_subscription(
            PoseStamped, self._in_topic, self._on_curr_pose, 10)
        self.create_subscription(
            String, self._status_topic, self._on_status, 10)
        # Re-anchor: set pose_node's curr_pose to the current EE (no jump).
        self.create_service(Trigger, "~/reanchor", self._srv_reanchor)
        # Forwarding gate: hand control between the SpaceMouse and the dashboard.
        # ``~/forwarding`` (latched Bool) mirrors the state; ``~/set_forwarding``
        # (SetBool) toggles it. OFF = bridge stays silent so the dashboard owns
        # target_pose; ON re-anchors to the current EE then resumes forwarding.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._fwd_pub = self.create_publisher(Bool, "~/forwarding", latched)
        self.create_service(SetBool, "~/set_forwarding",
                            self._srv_set_forwarding)
        self._publish_forwarding()

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
            % (pos[0], pos[1], pos[2], self._tip_frame()),
            throttle_duration_sec=1.0)
        return True, "set_pose to current EE"

    def _srv_reanchor(self, request, response):
        self._resetting = True   # drop + re-anchor curr_pose to the EE
        ok, msg = self._reanchor()
        response.success = ok
        response.message = msg
        return response

    def _publish_forwarding(self) -> None:
        m = Bool()
        m.data = bool(self._forward_enabled)
        self._fwd_pub.publish(m)

    def _srv_set_forwarding(self, request, response):
        enable = bool(request.data)
        was = self._forward_enabled
        self._forward_enabled = enable
        if enable and not was:
            self._resetting = True    # re-anchor to the EE before resuming
        self._publish_forwarding()
        response.success = True
        response.message = (
            "SpaceMouse forwarding ENABLED (SpaceMouse drives target_pose)"
            if enable else
            "SpaceMouse forwarding DISABLED (dashboard drives target_pose)")
        self.get_logger().info(response.message)
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
        # Re-anchor on the enable rising edge OR a control-link switch: the
        # _on_curr_pose reset loop drops + set_poses curr_pose to the (new) EE
        # before forwarding again, so jogging starts jump-free.
        if self._follow_enable and enabled and (not was or cur_tip != self._last_tip):
            self._resetting = True
            self._last_tip = cur_tip

    def _on_curr_pose(self, msg: PoseStamped) -> None:
        if not self._forward_enabled:
            return                      # dashboard control: bridge stays silent
        if self._follow_enable and not self._enabled:
            return
        p = msg.pose.position
        pos = np.array([p.x, p.y, p.z])
        # DISCONTINUITY guard: a large step between consecutive curr_pose samples
        # is NOT puck motion (e.g. the pose_node restarted at the origin).
        # Forwarding it would teleport the robot -- the commander does NOT
        # jump-reject in FPC rate mode -> self-collision. Trigger a reset.
        if (self._last_curr is not None and not self._resetting
                and float(np.linalg.norm(pos - self._last_curr))
                > self._jump_thresh):
            self.get_logger().warn(
                "curr_pose jumped %.2f m (pose_node reset?) -- holding & "
                "re-anchoring to the EE, NOT forwarding"
                % float(np.linalg.norm(pos - self._last_curr)))
            self._resetting = True
        self._last_curr = pos
        # RESETTING (enable / link switch / jump): drop curr_pose and re-anchor
        # it to the current EE; resume forwarding only once it has landed.
        if self._resetting:
            ee = self._robot_ee_now()
            if ee is None:
                return                                  # TF not ready -> drop
            if float(np.linalg.norm(pos - ee[0])) <= self._anchor_tol:
                self._resetting = False                 # anchored -> forward now
            else:
                now = time.monotonic()
                if now - self._last_reanchor > 0.15:    # throttle set_pose
                    self._last_reanchor = now
                    self._reanchor()
                return
        # Forward EVERY continuous curr_pose 1:1 as the absolute target.
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._base_frame  # pose is in the robot base frame
        out.pose = msg.pose
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
