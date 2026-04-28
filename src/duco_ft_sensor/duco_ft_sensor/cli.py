"""Standalone (non-ROS) CLI for reading the Duco F/T sensor.

Mainly useful for bench testing the wiring and the sensor itself without
spinning up a full ROS graph.

Run with:
    ros2 run duco_ft_sensor read_ft_sensor
or directly:
    python3 -m duco_ft_sensor.cli
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from .driver import FTSensor


def main() -> int:
    ap = argparse.ArgumentParser(description="Duco 6-DoF F/T sensor reader (standalone)")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial device (default: /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=460800, help="baud rate (default: 460800)")
    ap.add_argument("--tare", action="store_true", help="zero the sensor before streaming")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="max console refresh rate in Hz (default 30; sensor still streams at 960 Hz)")
    args = ap.parse_args()

    try:
        sensor = FTSensor(port=args.port, baud=args.baud)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: cannot open {args.port}: {exc}", file=sys.stderr)
        return 1

    sensor.start_stream(tare=args.tare)

    def _shutdown(*_a):
        sensor.close()
        sys.stdout.write("\n")
        sys.stdout.flush()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    min_period = 1.0 / args.rate if args.rate > 0 else 0.0
    last = 0.0
    n = 0
    t0 = time.monotonic()
    print(f"Reading {args.port} @ {args.baud} 8N1.  Streaming at 960 Hz; printing up to {args.rate:.0f} Hz.")
    print("Press Ctrl-C to stop.\n")

    try:
        while True:
            w = sensor.read_wrench()
            if w is None:
                continue
            n += 1
            now = time.monotonic()
            if now - last < min_period:
                continue
            last = now
            hz = n / (now - t0) if now > t0 else 0.0
            sys.stdout.write(
                f"\rFx={w.fx:+8.3f} Fy={w.fy:+8.3f} Fz={w.fz:+8.3f} N    |  "
                f"Mx={w.tx:+8.4f} My={w.ty:+8.4f} Mz={w.tz:+8.4f} N*m  "
                f"[{hz:6.1f} Hz, drops={sensor.dropped_bytes}]"
            )
            sys.stdout.flush()
    except KeyboardInterrupt:
        _shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
