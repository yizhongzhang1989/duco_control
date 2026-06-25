#!/usr/bin/env python3
"""Deterministic on-graph integration test for spacemouse_servo.

This is a **manual** test (it needs a running ROS graph + TF), not an offline
pytest. It simulates the SpaceMouse puck and asserts the servo's gating /
capture / integrate / stop behaviour end-to-end.

Setup (three terminals, each ``source install/setup.bash`` first):

1. Publish a static EE transform::

       ros2 run tf2_ros static_transform_publisher \
           --x 0.5 --y 0.1 --z 0.3 --frame-id test_base --child-frame-id test_tip

2. Start the bridge against those frames (commander disabled = pure target test)::

       ros2 run spacemouse_teleop servo_node --ros-args \
           -p base_frame:=test_base -p tip_frame:=test_tip \
           -p target_pose_topic:=/sm_target -p enable_commander:=false \
           -p jog_frame:=base -p linear_scale:="[0.1,0.1,0.1]" -p max_linear_speed:=0.1

3. Run this checker::

       python3 test/manual_servo_integration.py

Phases simulated:
  WARMUP : disengaged, zero twist  (servo captures the TF pose)
  A      : disengaged + twist      -> expect ZERO targets (dead-man gating)
  B      : engaged   + twist (+x)  -> expect target streaming, +x motion
  C      : released  + twist       -> expect target stream STOPS

Prints PASS/FAIL per phase and exits non-zero on any failure.
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

WARMUP_END = 0.8
A_END = 2.0
B_END = 3.6
C_CHECK = 4.4


class Tester(Node):
    def __init__(self):
        super().__init__("sm_integration_test")
        self._twist_pub = self.create_publisher(Twist, "spacenav/twist", 10)
        self._joy_pub = self.create_publisher(Joy, "spacenav/joy", 10)
        self.create_subscription(PoseStamped, "/sm_target", self._on_target, 50)
        self._t0 = self._now()
        self.poses = []   # (t, x, y, z)
        self.done = False
        self.create_timer(0.02, self._tick)  # 50 Hz

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_target(self, msg: PoseStamped):
        t = self._now() - self._t0
        self.poses.append((t, msg.pose.position.x,
                           msg.pose.position.y, msg.pose.position.z))

    def _pub(self, buttons, lin_x):
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = [0.0] * 6
        joy.buttons = buttons
        self._joy_pub.publish(joy)
        tw = Twist()
        tw.linear.x = lin_x
        self._twist_pub.publish(tw)

    def _tick(self):
        if self.done:
            return
        t = self._now() - self._t0
        if t < WARMUP_END:
            self._pub([0, 0], 0.0)
        elif t < A_END:
            self._pub([0, 0], 1.0)          # disengaged + twist
        elif t < B_END:
            self._pub([1, 0], 1.0)          # engaged + twist
        elif t < C_CHECK:
            self._pub([0, 0], 1.0)          # released + twist
        else:
            self.done = True
            self._evaluate()

    def _evaluate(self):
        a = [p for p in self.poses if WARMUP_END <= p[0] < A_END]
        b = [p for p in self.poses if A_END <= p[0] < B_END]
        c = [p for p in self.poses if B_END + 0.3 <= p[0] < C_CHECK]

        # Effective speed = linear_scale(0.1) * speed_scales[0](0.25) = 0.025 m/s
        # over the ~1.6 s engaged window ~= 0.04 m; threshold well below that and
        # well above the disengaged ~0.
        pass_a = len(a) == 0
        pass_b = len(b) >= 5 and (b[-1][1] - b[0][1]) > 0.02
        pass_c = len(c) <= 2

        print("\n==== spacemouse_servo integration test ====")
        print("Phase A (disengaged+twist): %d targets  -> %s "
              "(expect 0 — dead-man gating)"
              % (len(a), "PASS" if pass_a else "FAIL"))
        if b:
            print("Phase B (engaged+twist):    %d targets, dx=%.4f m -> %s "
                  "(expect motion +x)"
                  % (len(b), b[-1][1] - b[0][1], "PASS" if pass_b else "FAIL"))
            print("    start x=%.4f  end x=%.4f" % (b[0][1], b[-1][1]))
        else:
            print("Phase B: NO targets received -> FAIL")
        print("Phase C (released+twist):   %d targets -> %s "
              "(expect stop)" % (len(c), "PASS" if pass_c else "FAIL"))
        ok = pass_a and pass_b and pass_c
        print("RESULT: %s\n" % ("ALL PASS" if ok else "FAILURE"))
        self._rc = 0 if ok else 1
        raise SystemExit


def main():
    rclpy.init()
    node = Tester()
    rc = 0
    try:
        rclpy.spin(node)
    except SystemExit:
        rc = getattr(node, "_rc", 1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
