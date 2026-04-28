#!/usr/bin/env python3
"""
Read 6-DoF force/torque sensor over RS-422 (USB serial adapter, /dev/ttyUSB0).

Wire protocol actually used by this unit (matches the manufacturer's C# sample
in manual.md, not the 12-byte int format described in section 4.3):
  Serial:   460800 baud, 8N1, no flow control
  Frame:    28 bytes
              [0]    = 0x48           header byte 1
              [1]    = 0xAA           header byte 2
              [2:26] = 6 x float32 LE (Fx, Fy, Fz, Mx, My, Mz)
              [26]   = 0x0D           tail
              [27]   = 0x0A           tail
            Final values are float * 10  ->  forces in N, torques in N*m.

Host -> sensor commands (all 4 bytes):
  0x43 0xAA 0x0D 0x0A   stop streaming
  0x47 0xAA 0x0D 0x0A   tare (zero) then stream @ 960 Hz
  0x48 0xAA 0x0D 0x0A   stream @ 960 Hz (no tare)
  0x49 0xAA 0x0D 0x0A   one shot

CLI:
  python3 read_ft_sensor.py                    # stream raw values
  python3 read_ft_sensor.py --tare             # zero the sensor before streaming
  python3 read_ft_sensor.py --port /dev/ttyUSB0 --baud 460800
  python3 read_ft_sensor.py --rate 20          # cap console refresh to ~20 Hz
  python3 read_ft_sensor.py --raw              # also print unscaled float32 values

Press Ctrl-C to stop. Sensor is commanded to stop streaming on exit.

Note: /dev/ttyUSB0 belongs to group 'dialout'. If you see PermissionError run once:
        sudo usermod -aG dialout $USER && newgrp dialout
      (or temporarily: sudo chmod 666 /dev/ttyUSB0)
"""

from __future__ import annotations

import argparse
import signal
import struct
import sys
import time

import serial

# --- protocol constants ----------------------------------------------------
FRAME_LEN = 28
CMD_STOP = b"\x43\xAA\x0D\x0A"
CMD_TARE_STREAM = b"\x47\xAA\x0D\x0A"
CMD_STREAM = b"\x48\xAA\x0D\x0A"

# Manufacturer's C# sample multiplies each float by 10. Empirically this yields
# Newtons for Fx/Fy/Fz and N*m for Mx/My/Mz.
SCALE = 10.0

_FLOATS = struct.Struct("<6f")


def parse_frame(frame: bytes) -> tuple[float, float, float, float, float, float]:
    """Decode a 28-byte frame into (Fx, Fy, Fz, Mx, My, Mz) scaled values."""
    vals = _FLOATS.unpack_from(frame, 2)
    return tuple(v * SCALE for v in vals)  # type: ignore[return-value]


def open_serial(port: str, baud: int) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
        write_timeout=0.5,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="6-DoF F/T sensor reader (28-byte float protocol)")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial device (default: /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=460800, help="baud rate (default: 460800)")
    ap.add_argument("--tare", action="store_true",
                    help="zero the sensor before streaming (sends 0x47 instead of 0x48)")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="max console refresh rate in Hz (default 30; sensor still streams at 960 Hz)")
    ap.add_argument("--raw", action="store_true",
                    help="also print the unscaled float32 values from the frame")
    args = ap.parse_args()

    try:
        ser = open_serial(args.port, args.baud)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {args.port}: {e}", file=sys.stderr)
        if "Permission denied" in str(e):
            print("Hint: add yourself to the 'dialout' group:\n"
                  "        sudo usermod -aG dialout $USER && newgrp dialout\n"
                  "      or temporarily: sudo chmod 666 " + args.port, file=sys.stderr)
        return 1

    # Stop any prior stream, flush, then start fresh.
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(CMD_STOP)
    ser.flush()
    time.sleep(0.05)
    ser.reset_input_buffer()
    ser.write(CMD_TARE_STREAM if args.tare else CMD_STREAM)
    ser.flush()

    def _shutdown(*_a):
        try:
            ser.write(CMD_STOP)
            ser.flush()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        sys.stdout.write("\n")
        sys.stdout.flush()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    buf = bytearray()
    min_period = 1.0 / args.rate if args.rate > 0 else 0.0
    last_print = 0.0
    frame_count = 0
    bad_bytes = 0
    t0 = time.monotonic()

    print(f"Reading {args.port} @ {args.baud} 8N1.  Streaming at 960 Hz; printing up to {args.rate:.0f} Hz.")
    print("Press Ctrl-C to stop.\n")

    try:
        while True:
            chunk = ser.read(1024)
            if chunk:
                buf.extend(chunk)

            # Frame sync: walk buffer for 0x48 0xAA ... 0x0D 0x0A (length 28).
            latest = None
            while len(buf) >= FRAME_LEN:
                if (buf[0] == 0x48 and buf[1] == 0xAA
                        and buf[26] == 0x0D and buf[27] == 0x0A):
                    latest = bytes(buf[:FRAME_LEN])
                    del buf[:FRAME_LEN]
                    frame_count += 1
                else:
                    del buf[0]
                    bad_bytes += 1

            if latest is None:
                continue

            now = time.monotonic()
            if now - last_print < min_period:
                continue
            last_print = now

            fx, fy, fz, mx, my, mz = parse_frame(latest)
            elapsed = now - t0
            hz = frame_count / elapsed if elapsed > 0 else 0.0

            line = (f"\rFx={fx:+8.3f} Fy={fy:+8.3f} Fz={fz:+8.3f} N    |  "
                    f"Mx={mx:+8.4f} My={my:+8.4f} Mz={mz:+8.4f} N*m  "
                    f"[{hz:6.1f} Hz, drops={bad_bytes}]")
            if args.raw:
                raw_vals = _FLOATS.unpack_from(latest, 2)
                line += "  raw=" + ",".join(f"{v:+.4f}" for v in raw_vals)
            sys.stdout.write(line)
            sys.stdout.flush()

    except serial.SerialException as e:
        print(f"\nSerial error: {e}", file=sys.stderr)
        _shutdown()
        return 1


if __name__ == "__main__":
    sys.exit(main())
