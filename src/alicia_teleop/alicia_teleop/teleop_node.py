"""Teleoperate the Duco GCR5_910 from Alicia-D leader arm joint states.

Subscribes to /arm_joint_state (ArmJointState) from the Alicia driver,
maps the 6 leader joint angles to the follower robot's joints, and
publishes them as a JointTrajectory on the JTC's command topic.

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
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
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
DEFAULT_LEADER_TOPIC = "/arm_joint_state"


class AliciaTeleop(Node):
    def __init__(self):
        super().__init__("alicia_teleop")

        # Parameters
        self.declare_parameter("rate", 100.0)  # Hz — command publish rate
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("trajectory_topic", DEFAULT_TRAJECTORY_TOPIC)
        self.declare_parameter("leader_topic", DEFAULT_LEADER_TOPIC)
        self.declare_parameter("joint_scale", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("trajectory_time", 0.05)  # seconds — time_from_start per point
        # Velocity feedforward: estimate leader joint velocity from
        # successive samples and send it with each trajectory point so
        # JTC interpolates a smooth velocity profile instead of
        # stop-and-go ramps. Cap protects against finite-difference
        # noise / large timestamp gaps.
        self.declare_parameter("velocity_feedforward", True)
        self.declare_parameter("max_velocity", 3.0)  # rad/s, per joint

        rate = self.get_parameter("rate").value
        self._joint_names = list(self.get_parameter("joint_names").value)
        trajectory_topic = self.get_parameter("trajectory_topic").value
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

        # Publisher for joint trajectory commands
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            trajectory_topic,
            10,
        )

        # Timer to send commands at fixed rate
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info(
            f"Alicia teleop started at {rate:.0f} Hz, "
            f"traj_time={self._traj_time:.3f}s, "
            f"vff={'on' if self._vff_enabled else 'off'} "
            f"vmax={self._vmax:.2f} rad/s, "
            f"leader_topic={leader_topic}, "
            f"trajectory_topic={trajectory_topic}, "
            f"joint_names={self._joint_names}"
        )
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
        """Send joint trajectory command from leader arm data."""
        if self._leader_joints is None or not self._active:
            return

        # Map leader joints to follower joints. Velocity is mapped by
        # the same scale (sign flips track) and offset is irrelevant
        # since it's a constant.
        target = self._leader_joints * self._joint_scale + self._joint_offset
        target_vel = self._leader_velocity * self._joint_scale

        # Build trajectory message with single point
        traj = JointTrajectory()
        traj.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        if self._vff_enabled:
            point.velocities = target_vel.tolist()
        sec = int(self._traj_time)
        nanosec = int((self._traj_time - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points = [point]

        self._traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = AliciaTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
