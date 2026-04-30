"""ROS 2 node that streams the Duco 6-DoF F/T sensor.

Topics
------
publishes:
    ~/wrench_raw  geometry_msgs/WrenchStamped   raw 6-axis wrench, frame_id=<frame_id>
  ~/diagnostics  diagnostic_msgs/...           (not used; kept simple)

subscribes:
  ~/command  std_msgs/String                   one of: "start", "stop", "tare", "zero"

Services
--------
  ~/start    std_srvs/Trigger                  begin streaming (no tare)
  ~/stop     std_srvs/Trigger                  stop streaming
  ~/tare     std_srvs/Trigger                  zero the sensor and stream

Parameters
----------
  port           string  default "/dev/ttyUSB0"
  baud           int     default 460800
    topic          string  default "~/wrench_raw"
  frame_id       string  default "ft_sensor_link"
  publish_rate   double  default 0.0  (0 = publish every frame, ~960 Hz)
  autostart      bool    default true (start streaming on launch)
  tare_on_start  bool    default false
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

import serial

from .driver import FTSensor, Wrench


class FTSensorNode(Node):
    def __init__(self) -> None:
        super().__init__("duco_ft_sensor")

        # --- parameters ---------------------------------------------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 460800)
        self.declare_parameter("topic", "~/wrench_raw")
        self.declare_parameter("frame_id", "ft_sensor_link")
        self.declare_parameter("publish_rate", 0.0)
        self.declare_parameter("autostart", True)
        self.declare_parameter("tare_on_start", False)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        autostart = self.get_parameter("autostart").get_parameter_value().bool_value
        tare_on_start = self.get_parameter("tare_on_start").get_parameter_value().bool_value

        # --- driver -------------------------------------------------------
        try:
            self._sensor = FTSensor(port=port, baud=baud)
        except serial.SerialException as exc:
            self.get_logger().fatal(f"could not open {port}: {exc}")
            raise

        # --- ROS interface ------------------------------------------------
        # High-rate streaming sensor: BEST_EFFORT + KEEP_LAST with a depth
        # comfortably larger than any plausible USB-serial batch (~16 frames
        # at the kernel default 16 ms latency timer). depth=200 covers >100 ms
        # of buffering even at 1 kHz so a slow subscriber tick never causes
        # the publisher's history to drop frames.
        wrench_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self._pub = self.create_publisher(WrenchStamped, topic, wrench_qos)
        self._cmd_sub = self.create_subscription(
            String, "~/command", self._on_command, 10)
        self._srv_start = self.create_service(Trigger, "~/start", self._srv_start_cb)
        self._srv_stop = self.create_service(Trigger, "~/stop", self._srv_stop_cb)
        self._srv_tare = self.create_service(Trigger, "~/tare", self._srv_tare_cb)
        self._srv_reset_tare = self.create_service(
            Trigger, "~/reset_tare", self._srv_reset_tare_cb)

        # Software tare offsets (Fx, Fy, Fz, Mx, My, Mz). Subtracted from
        # every published wrench. We use software tare because the sensor's
        # 0x47 hardware-tare command is unreliable on the verified firmware
        # (it neither zeroes the readings nor restarts the data stream).
        self._offsets: List[float] = [0.0] * 6
        # Set by tare request; cleared by the reader loop after capturing
        # the next fresh sample as the new offset baseline.
        self._tare_pending: bool = False

        # --- background reader thread ------------------------------------
        self._streaming = False
        self._stop_thread = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        if autostart:
            self.get_logger().info(
                f"starting stream on {port} @ {baud} -> {topic} "
                f"(tare={tare_on_start})")
            self._sensor.start_stream(tare=tare_on_start)
            self._streaming = True
        else:
            self.get_logger().info(
                f"opened {port} @ {baud}; publishing to {topic}; "
                "waiting for start command")

    # --- ROS callbacks ----------------------------------------------------
    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == "start":
            self._do_start(tare=False)
        elif cmd == "stop":
            self._do_stop()
        elif cmd in ("tare", "zero"):
            self._do_tare()
        elif cmd in ("reset_tare", "untare", "clear_tare"):
            self._do_reset_tare()
        else:
            self.get_logger().warn(
                f"ignoring unknown command '{msg.data}' "
                "(expected: start | stop | tare | zero | reset_tare)")

    def _srv_start_cb(self, _req, resp: Trigger.Response):
        self._do_start(tare=False)
        resp.success = True
        resp.message = "streaming"
        return resp

    def _srv_stop_cb(self, _req, resp: Trigger.Response):
        self._do_stop()
        resp.success = True
        resp.message = "stopped"
        return resp

    def _srv_tare_cb(self, _req, resp: Trigger.Response):
        self._do_tare()
        resp.success = True
        resp.message = "tare requested (offset captured from next sample)"
        return resp

    def _srv_reset_tare_cb(self, _req, resp: Trigger.Response):
        self._do_reset_tare()
        resp.success = True
        resp.message = "tare cleared"
        return resp

    # --- helpers ----------------------------------------------------------
    def _do_start(self, tare: bool) -> None:
        # Park the reader thread first so it isn't holding the buffer or
        # mid-`serial.read` while we issue commands. The reader picks back
        # up immediately when we re-flip the flag.
        was_streaming = self._streaming
        self._streaming = False
        if was_streaming:
            time.sleep(0.15)  # > one serial-read timeout (0.1 s)
        try:
            self._sensor.start_stream(tare=tare)
        finally:
            self._streaming = True
        self.get_logger().info("stream started" + (" (tared)" if tare else ""))

    def _do_stop(self) -> None:
        self._streaming = False
        time.sleep(0.05)
        self._sensor.stop_stream()
        self.get_logger().info("stream stopped")

    def _do_tare(self) -> None:
        """Capture the next received sample as the software-tare baseline.

        Use software tare because the hardware 0x47 command is a no-op on
        the verified firmware. If the stream isn't running, start it first
        so that a fresh sample is actually available.
        """
        if not self._streaming:
            self._do_start(tare=False)
        self._tare_pending = True
        self.get_logger().info("tare requested (offset = next sample)")

    def _do_reset_tare(self) -> None:
        self._offsets = [0.0] * 6
        self._tare_pending = False
        self.get_logger().info("tare cleared (offsets = 0)")

    def _publish(self, w: Wrench) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        ox, oy, oz, otx, oty, otz = self._offsets
        msg.wrench.force.x = w.fx - ox
        msg.wrench.force.y = w.fy - oy
        msg.wrench.force.z = w.fz - oz
        msg.wrench.torque.x = w.tx - otx
        msg.wrench.torque.y = w.ty - oty
        msg.wrench.torque.z = w.tz - otz
        self._pub.publish(msg)

    # --- reader loop ------------------------------------------------------
    def _reader_loop(self) -> None:
        min_period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.0
        last_pub = 0.0
        while not self._stop_thread.is_set():
            if not self._streaming:
                time.sleep(0.01)
                continue
            try:
                wrench = self._sensor.read_wrench()
            except serial.SerialException as exc:
                self.get_logger().error(f"serial read failed: {exc}")
                break
            if wrench is None:
                continue
            # Capture next-sample tare baseline if requested.
            if self._tare_pending:
                self._offsets = [wrench.fx, wrench.fy, wrench.fz,
                                 wrench.tx, wrench.ty, wrench.tz]
                self._tare_pending = False
                self.get_logger().info(
                    f"tare baseline set: Fx={wrench.fx:+.3f} Fy={wrench.fy:+.3f} "
                    f"Fz={wrench.fz:+.3f} N  Mx={wrench.tx:+.3f} My={wrench.ty:+.3f} "
                    f"Mz={wrench.tz:+.3f} N*m")
            now = time.monotonic()
            if min_period > 0.0 and (now - last_pub) < min_period:
                continue
            last_pub = now
            if self._stop_thread.is_set() or not rclpy.ok():
                break
            try:
                self._publish(wrench)
            except Exception:  # noqa: BLE001
                # context likely shutting down
                break

    # --- shutdown ---------------------------------------------------------
    def destroy_node(self) -> bool:
        self._stop_thread.set()
        self._thread.join(timeout=1.0)
        try:
            self._sensor.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[FTSensorNode] = None
    try:
        node = FTSensorNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger("duco_ft_sensor").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
