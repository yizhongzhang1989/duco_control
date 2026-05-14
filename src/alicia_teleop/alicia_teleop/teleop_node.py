"""Teleoperate the Duco GCR5_910 from Alicia-D leader arm joint states.

Subscribes to /arm_joint_state (ArmJointState) from the Alicia driver,
maps the 6 leader joint angles to the follower robot's joints, and
publishes them as a JointTrajectory on the JTC's command topic.

The leader-side calibration (direction flip / zero offset / continuous
unwrap per joint) is handled inside the Alicia driver itself via its
``joint_config`` YAML; by the time the angles reach this node they are
already in the follower robot's joint frame. ``joint_scale`` /
``joint_offset`` here are an optional second-stage tweak.

Default joint mapping (Alicia → Duco GCR5_910):
  joint1 → arm_1_joint_1
  joint2 → arm_1_joint_2
  joint3 → arm_1_joint_3
  joint4 → arm_1_joint_4
  joint5 → arm_1_joint_5
  joint6 → arm_1_joint_6
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
        self.declare_parameter("rate", 50.0)  # Hz — command publish rate
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("trajectory_topic", DEFAULT_TRAJECTORY_TOPIC)
        self.declare_parameter("leader_topic", DEFAULT_LEADER_TOPIC)
        self.declare_parameter("joint_scale", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("trajectory_time", 0.1)  # seconds — time_from_start per point

        rate = self.get_parameter("rate").value
        self._joint_names = list(self.get_parameter("joint_names").value)
        trajectory_topic = self.get_parameter("trajectory_topic").value
        leader_topic = self.get_parameter("leader_topic").value
        self._joint_scale = np.array(self.get_parameter("joint_scale").value, dtype=np.float64)
        self._joint_offset = np.array(self.get_parameter("joint_offset").value, dtype=np.float64)
        self._traj_time = self.get_parameter("trajectory_time").value

        if len(self._joint_names) != 6:
            raise ValueError(
                f"alicia_teleop expects 6 joint_names, got {len(self._joint_names)}: "
                f"{self._joint_names}")
        if self._joint_scale.shape != (6,) or self._joint_offset.shape != (6,):
            raise ValueError(
                "joint_scale and joint_offset must each have 6 elements")

        # State
        self._leader_joints = None  # latest leader arm joint angles (6,)
        self._active = False

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
            f"leader_topic={leader_topic}, "
            f"trajectory_topic={trajectory_topic}, "
            f"joint_names={self._joint_names}"
        )

    def _leader_cb(self, msg: ArmJointState):
        """Store latest leader arm joints."""
        self._leader_joints = np.array([
            msg.joint1, msg.joint2, msg.joint3,
            msg.joint4, msg.joint5, msg.joint6,
        ], dtype=np.float64)

        # Use button to toggle active state
        # but1: 0x10 = sync mode active
        if msg.but1 == 0x10:
            if not self._active:
                self._active = True
                self.get_logger().info("Teleop ACTIVE (sync button pressed)")
        else:
            if self._active:
                self._active = False
                self.get_logger().info("Teleop PAUSED (sync button released)")

    def _timer_cb(self):
        """Send joint trajectory command from leader arm data."""
        if self._leader_joints is None or not self._active:
            return

        # Map leader joints to follower joints
        target = self._leader_joints * self._joint_scale + self._joint_offset

        # Build trajectory message with single point
        traj = JointTrajectory()
        traj.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = target.tolist()
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
