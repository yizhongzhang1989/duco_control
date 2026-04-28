#!/usr/bin/env python3
"""Standalone self-test for the Duco 6-DoF F/T sensor.

This script intentionally has *no* ROS dependency and *no* dependency on the
duco_ft_sensor package -- everything it needs is inlined below. Drop it on
any Linux machine with Python 3.8+ and pyserial installed and run it.

Install pyserial if needed:
    sudo apt install python3-serial          # Debian / Ubuntu
    # or
    pip install pyserial                     # any OS

Run it:
    python3 test_ft_sensor.py
    python3 test_ft_sensor.py --port /dev/ttyUSB0 --duration 5
    python3 test_ft_sensor.py --check-tare        # also verify TARE works
    python3 test_ft_sensor.py --check-oneshot     # also verify ONE-SHOT works

The script runs a sequence of checks against the physical sensor and
prints PASS/FAIL for each. Exit status is 0 if every selected check
passed, 1 otherwise -- suitable for use in CI / install scripts.

Make sure no other process holds the serial port (e.g. stop the ROS node
first), and that the user can read+write the device (group "dialout"
membership is the usual fix on Linux).

Sensor protocol (verified on the physical hardware):

    Serial   : 460800 baud, 8N1, no flow control
    Frame    : 28 bytes
                 [0]    = 0x48           header byte 1
                 [1]    = 0xAA           header byte 2
                 [2:26] = 6 x float32 LE (Fx, Fy, Fz, Mx, My, Mz)
                 [26]   = 0x0D
                 [27]   = 0x0A
               Final values are float * 10  ->  N (forces) / N*m (torques).
    Native rate: ~960 Hz.

    Host -> sensor commands (each is exactly 4 bytes):
      0x43 0xAA 0x0D 0x0A   stop streaming
      0x47 0xAA 0x0D 0x0A   tare (zero) then stream
      0x48 0xAA 0x0D 0x0A   stream
      0x49 0xAA 0x0D 0x0A   one-shot
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from typing import Callable, List, Optional, Tuple

try:
    import serial
except ImportError:
    sys.stderr.write(
        "ERROR: pyserial is not installed. Install it with one of:\n"
        "  sudo apt install python3-serial   # Debian / Ubuntu\n"
        "  pip install pyserial              # any OS\n")
    sys.exit(2)


# ---------------------------------------------------------------------------
# protocol constants (inlined; this script depends on nothing else)
# ---------------------------------------------------------------------------
DEFAULT_BAUD = 460800
FRAME_LEN = 28
HEADER0 = 0x48
HEADER1 = 0xAA
TAIL0 = 0x0D
TAIL1 = 0x0A

CMD_STOP = b"\x43\xAA\x0D\x0A"
CMD_TARE_STREAM = b"\x47\xAA\x0D\x0A"
CMD_STREAM = b"\x48\xAA\x0D\x0A"
CMD_ONESHOT = b"\x49\xAA\x0D\x0A"

SCALE = 10.0
_FLOATS = struct.Struct("<6f")


def parse_frame(frame: bytes) -> Tuple[float, float, float, float, float, float]:
    """Decode a 28-byte frame -> (Fx, Fy, Fz, Mx, My, Mz) in N / N*m."""
    if len(frame) != FRAME_LEN:
        raise ValueError(f"frame must be {FRAME_LEN} bytes, got {len(frame)}")
    if (frame[0] != HEADER0 or frame[1] != HEADER1
            or frame[26] != TAIL0 or frame[27] != TAIL1):
        raise ValueError("frame does not match 0x48 0xAA ... 0x0D 0x0A")
    fx, fy, fz, tx, ty, tz = _FLOATS.unpack_from(frame, 2)
    return (fx * SCALE, fy * SCALE, fz * SCALE,
            tx * SCALE, ty * SCALE, tz * SCALE)


# ---------------------------------------------------------------------------
# tiny test harness
# ---------------------------------------------------------------------------
class TestRunner:
    def __init__(self) -> None:
        self.results: List[Tuple[str, bool, str]] = []

    def run(self, name: str, fn: Callable[[], Tuple[bool, str]]) -> bool:
        sys.stdout.write(f"  {name:<42} ... ")
        sys.stdout.flush()
        try:
            ok, detail = fn()
        except Exception as exc:  # noqa: BLE001
            ok, detail = False, f"exception: {type(exc).__name__}: {exc}"
        sys.stdout.write(("PASS" if ok else "FAIL")
                         + (f"  ({detail})" if detail else "") + "\n")
        self.results.append((name, ok, detail))
        return ok

    def summary(self) -> int:
        passed = sum(1 for _, ok, _ in self.results if ok)
        total = len(self.results)
        print()
        print(f"{passed}/{total} checks passed.")
        return 0 if passed == total else 1


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _open(port: str, baud: int) -> serial.Serial:
    return serial.Serial(
        port=port, baudrate=baud,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=0.1, write_timeout=0.5)


def _read_for(s: serial.Serial, seconds: float) -> bytes:
    end = time.monotonic() + seconds
    buf = bytearray()
    while time.monotonic() < end:
        chunk = s.read(4096)
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def _is_finite(x: float) -> bool:
    return x == x and abs(x) < float("inf")


def _drain_one_frame(s: serial.Serial, seconds: float
                     ) -> Optional[Tuple[float, float, float,
                                         float, float, float]]:
    """Read for at most `seconds` and return the most recent decoded wrench
    (or None if no aligned frame was seen)."""
    raw = _read_for(s, seconds)
    last = None
    i = 0
    while i + FRAME_LEN <= len(raw):
        if (raw[i] == HEADER0 and raw[i+1] == HEADER1
                and raw[i+26] == TAIL0 and raw[i+27] == TAIL1):
            try:
                last = parse_frame(raw[i:i+FRAME_LEN])
            except Exception:
                pass
            i += FRAME_LEN
        else:
            i += 1
    return last


# ---------------------------------------------------------------------------
# checks
# ---------------------------------------------------------------------------
def check_device_present(port: str) -> Tuple[bool, str]:
    if not os.path.exists(port):
        return False, f"{port} does not exist"
    if not os.access(port, os.R_OK | os.W_OK):
        return False, (f"{port} not readable/writable "
                       "(add user to 'dialout' group?)")
    return True, port


def check_open_port(port: str, baud: int) -> Tuple[bool, str]:
    s = _open(port, baud)
    s.close()
    return True, f"opened at {baud} 8N1"


def check_stop_command(port: str, baud: int) -> Tuple[bool, str]:
    """Sending CMD_STOP should silence the stream."""
    with _open(port, baud) as s:
        s.reset_input_buffer()
        s.write(CMD_STOP); s.flush()
        time.sleep(0.10)
        s.reset_input_buffer()
        bytes_after = len(_read_for(s, 0.30))
    if bytes_after > 256:
        return False, (f"got {bytes_after} bytes in 0.3 s after STOP "
                       "(sensor still streaming?)")
    return True, f"{bytes_after} bytes after STOP"


def check_streaming(port: str, baud: int) -> Tuple[bool, str]:
    """CMD_STREAM should produce ~1000 frames/s of well-formed data."""
    with _open(port, baud) as s:
        s.reset_input_buffer()
        s.write(CMD_STOP); s.flush(); time.sleep(0.05); s.reset_input_buffer()
        s.write(CMD_STREAM); s.flush()
        raw = _read_for(s, 1.0)
        s.write(CMD_STOP); s.flush()
    if len(raw) < FRAME_LEN:
        return False, f"got only {len(raw)} bytes in 1 s"
    return True, (f"{len(raw)} bytes in 1.0 s "
                  f"(~{len(raw)/FRAME_LEN:.0f} frames)")


def check_frame_alignment(port: str, baud: int) -> Tuple[bool, str]:
    """Verify frames lock to the expected header/tail and decode to finite floats."""
    with _open(port, baud) as s:
        s.reset_input_buffer()
        s.write(CMD_STOP); s.flush(); time.sleep(0.05); s.reset_input_buffer()
        s.write(CMD_STREAM); s.flush()
        raw = _read_for(s, 0.5)
        s.write(CMD_STOP); s.flush()

    good = bad = 0
    last = None
    i = 0
    while i + FRAME_LEN <= len(raw):
        if (raw[i] == HEADER0 and raw[i+1] == HEADER1
                and raw[i+26] == TAIL0 and raw[i+27] == TAIL1):
            try:
                last = parse_frame(raw[i:i+FRAME_LEN])
                good += 1
            except Exception:
                bad += 1
            i += FRAME_LEN
        else:
            bad += 1
            i += 1

    if good == 0:
        return False, f"no aligned frames in {len(raw)} bytes"
    if last is None or any(not _is_finite(v) for v in last):
        return False, "decoded a non-finite value"
    fx, fy, fz, _, _, _ = last
    return True, (f"{good} good frames, {bad} stray bytes; "
                  f"last Fx={fx:+.2f} Fy={fy:+.2f} Fz={fz:+.2f} N")


def check_sample_rate(port: str, baud: int, duration: float,
                      lo: float = 800.0, hi: float = 1100.0
                      ) -> Tuple[bool, str]:
    """Count frames received per wall-clock second over `duration`."""
    buf = bytearray()
    n_frames = 0
    drops = 0
    with _open(port, baud) as s:
        s.reset_input_buffer()
        s.write(CMD_STOP); s.flush(); time.sleep(0.05); s.reset_input_buffer()
        s.write(CMD_STREAM); s.flush()
        t0 = time.monotonic()
        end = t0 + duration
        while time.monotonic() < end:
            chunk = s.read(1024)
            if chunk:
                buf.extend(chunk)
            while len(buf) >= FRAME_LEN:
                if (buf[0] == HEADER0 and buf[1] == HEADER1
                        and buf[26] == TAIL0 and buf[27] == TAIL1):
                    n_frames += 1
                    del buf[:FRAME_LEN]
                else:
                    drops += 1
                    del buf[0]
        elapsed = time.monotonic() - t0
        s.write(CMD_STOP); s.flush()

    rate = n_frames / elapsed if elapsed > 0 else 0.0
    if not (lo <= rate <= hi):
        return False, (f"got {rate:.1f} Hz over {elapsed:.1f}s "
                       f"(expected {lo:g}-{hi:g})")
    return True, (f"{n_frames} frames in {elapsed:.1f}s "
                  f"-> {rate:.1f} Hz (drops={drops})")


def check_tare(port: str, baud: int) -> Tuple[bool, str]:
    """Check whether the firmware HARDWARE tare actually zeroes the axes.

    On the verified unit this is a NO-OP -- the firmware does not honour
    0x47 (it neither zeroes nor restarts streaming). The check sends
    STOP -> 0x47 -> 0x48 and verifies all six axes are within +/- 5 of
    zero. Failure here is normal on this hardware; the ROS node works
    around it with software tare.
    """
    with _open(port, baud) as s:
        s.reset_input_buffer()
        s.write(CMD_STOP); s.flush(); time.sleep(0.05); s.reset_input_buffer()
        s.write(CMD_TARE_STREAM); s.flush(); time.sleep(0.10)
        s.write(CMD_STREAM); s.flush()
        last = _drain_one_frame(s, 0.50)
        s.write(CMD_STOP); s.flush()
    if last is None:
        return False, "no aligned frame received after tare"
    if any(abs(v) > 5.0 for v in last):
        return False, f"post-tare values too large: {tuple(round(v,3) for v in last)}"
    fx, fy, fz, mx, my, mz = last
    return True, (f"Fx={fx:+.3f} Fy={fy:+.3f} Fz={fz:+.3f} N  "
                  f"Mx={mx:+.3f} My={my:+.3f} Mz={mz:+.3f} N*m")


def check_oneshot(port: str, baud: int) -> Tuple[bool, str]:
    """0x49 should produce at least one well-formed frame within ~1 s."""
    with _open(port, baud) as s:
        s.reset_input_buffer()
        s.write(CMD_STOP); s.flush(); time.sleep(0.10); s.reset_input_buffer()
        s.write(CMD_ONESHOT); s.flush()
        raw = _read_for(s, 1.00)
    for i in range(len(raw) - FRAME_LEN + 1):
        if (raw[i] == HEADER0 and raw[i+1] == HEADER1
                and raw[i+26] == TAIL0 and raw[i+27] == TAIL1):
            return True, f"got aligned frame in {len(raw)} bytes"
    return False, (f"no aligned frame in {len(raw)} bytes after one-shot "
                   "(this firmware may not support 0x49; "
                   "--skip-oneshot to ignore)")


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Standalone self-test for the Duco 6-DoF F/T sensor "
                    "(no ROS, no extra deps beyond pyserial).")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--duration", type=float, default=2.0,
                    help="rate-test duration in seconds (default 2.0)")
    ap.add_argument("--check-tare", action="store_true",
                    help="also verify the 0x47 (tare) command zeros all axes; "
                         "only meaningful with no load on the sensor and on "
                         "firmware revisions that support tare")
    ap.add_argument("--check-oneshot", action="store_true",
                    help="also verify the 0x49 (one-shot) command produces a "
                         "frame; not all firmware revisions support it")
    args = ap.parse_args()

    print(f"Duco F/T sensor self-test  port={args.port}  baud={args.baud}")
    print()
    r = TestRunner()
    if not r.run("device file present + permissions",
                 lambda: check_device_present(args.port)):
        return r.summary()
    if not r.run("open serial port",
                 lambda: check_open_port(args.port, args.baud)):
        return r.summary()
    r.run("STOP command silences stream",
          lambda: check_stop_command(args.port, args.baud))
    r.run("STREAM command produces data",
          lambda: check_streaming(args.port, args.baud))
    r.run("frames align (header/tail) & decode",
          lambda: check_frame_alignment(args.port, args.baud))
    r.run(f"sample rate over {args.duration:g}s (~960 Hz expected)",
          lambda: check_sample_rate(args.port, args.baud, args.duration))
    if args.check_tare:
        r.run("TARE zeros all 6 axes (~+/-5)  [optional]",
              lambda: check_tare(args.port, args.baud))
    if args.check_oneshot:
        r.run("ONE-SHOT produces a frame      [optional]",
              lambda: check_oneshot(args.port, args.baud))
    return r.summary()


if __name__ == "__main__":
    sys.exit(main())
