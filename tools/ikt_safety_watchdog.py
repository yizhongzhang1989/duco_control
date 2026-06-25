#!/usr/bin/env python3
"""Measured-TCP safety watchdog — independent backstop for the 30 cm envelope.

Polls the REAL Duco TCP pose (via ``tools/read_tcp_pose <ip> --csv``) at >=5 Hz
and, if the TCP leaves a sphere of ``--radius`` metres around ``--center-xyz``,
prints a loud warning and calls the commander's ``~/disable`` service (which
switches back to a JTC hold and cancels in-flight goals). It **fails safe**: any
read/parse error also triggers a disable.

Run during EVERY real-robot motion test (Phase 6), in its own terminal::

    ./tools/ikt_safety_watchdog.py --ip 192.168.1.10 \
        --center-xyz <x> <y> <z> --radius 0.30 --ns /ikt_pose_commander

It is deliberately independent of the commander's in-process Cartesian gate: a
separate process reading ground-truth TCP, so a commander bug cannot defeat it.
The hardware e-stop on the pendant remains the final backstop.

Offline check (no robot)::

    ./tools/ikt_safety_watchdog.py --self-test
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

DEFAULT_IP = "192.168.1.10"
DEFAULT_RADIUS = 0.30
DEFAULT_NS = "/ikt_pose_commander"
DEFAULT_RATE = 10.0  # Hz; plan requires >= 5 Hz


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def read_tcp_xyz(ip: str, reader_path: Optional[str] = None,
                 timeout: float = 1.0) -> Tuple[float, float, float]:
    """Return (x, y, z) in metres of the real TCP via ``read_tcp_pose --csv``.

    CSV columns: ``q1..q6,x,y,z,rx,ry,rz`` (joints rad, position m, rotation
    rad), so x/y/z are columns 6/7/8.
    """
    exe = reader_path or str(_repo_root() / "tools" / "read_tcp_pose")
    out = subprocess.run([exe, ip, "--csv"], capture_output=True, text=True,
                         timeout=timeout)
    if out.returncode != 0:
        raise RuntimeError("read_tcp_pose failed (rc=%d): %s"
                           % (out.returncode, (out.stderr or "").strip()))
    parts = out.stdout.strip().split(",")
    if len(parts) < 9:
        raise ValueError("unexpected CSV (%d cols): %r" % (len(parts), out.stdout))
    return float(parts[6]), float(parts[7]), float(parts[8])


def call_disable(ns: str, timeout: float = 5.0) -> bool:
    """Call ``<ns>/disable`` (std_srvs/srv/Trigger) via ``ros2 service call``."""
    svc = ns.rstrip("/") + "/disable"
    try:
        out = subprocess.run(
            ["ros2", "service", "call", svc, "std_srvs/srv/Trigger"],
            capture_output=True, text=True, timeout=timeout)
        return out.returncode == 0
    except Exception as exc:  # noqa: BLE001
        print("  !! disable call error: %r" % exc, file=sys.stderr)
        return False


def switch_to_hold(deactivate: str, activate: str, timeout: float = 5.0) -> bool:
    """Abort by switching controllers back to a JTC hold.

    For a raw FZI cartesian controller (no commander ``disable`` service) the
    fail-safe is to deactivate it and activate the trajectory controller, which
    holds the current measured pose. Uses ``ros2 control switch_controllers``.
    """
    try:
        out = subprocess.run(
            ["ros2", "control", "switch_controllers",
             "--deactivate", deactivate, "--activate", activate],
            capture_output=True, text=True, timeout=timeout)
        return out.returncode == 0
    except Exception as exc:  # noqa: BLE001
        print("  !! switch_controllers error: %r" % exc, file=sys.stderr)
        return False


class SafetyWatchdog:
    """Breach detector with injectable read/disable functions (testable core)."""

    def __init__(self, center_xyz: Sequence[float], radius: float,
                 read_fn: Callable[[], Tuple[float, float, float]],
                 disable_fn: Callable[[], bool], rate_hz: float = DEFAULT_RATE):
        self.center = tuple(float(v) for v in center_xyz)
        self.radius = float(radius)
        self.read_fn = read_fn
        self.disable_fn = disable_fn
        self.rate_hz = max(5.0, float(rate_hz))  # plan: >= 5 Hz
        self.tripped = False
        self.trip_reason = ""

    def distance(self, xyz: Sequence[float]) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(xyz, self.center)))

    def check_once(self) -> bool:
        """Read once; return True if a breach was handled (disable invoked)."""
        try:
            xyz = self.read_fn()
        except Exception as exc:  # noqa: BLE001  -> fail safe
            return self._trip("TCP read failed: %r (FAIL-SAFE disable)" % exc)
        d = self.distance(xyz)
        if d > self.radius:
            return self._trip(
                "TCP left the %.2f m sphere: d=%.3f m at [%.3f %.3f %.3f]"
                % (self.radius, d, xyz[0], xyz[1], xyz[2]))
        return False

    def _trip(self, reason: str) -> bool:
        self.tripped = True
        self.trip_reason = reason
        bar = "!" * 64
        print("\n%s\nWATCHDOG BREACH: %s\ncalling %s ...\n%s"
              % (bar, reason, "disable", bar), file=sys.stderr)
        ok = self.disable_fn()
        print("  disable %s" % ("OK" if ok else "FAILED -> USE E-STOP"),
              file=sys.stderr)
        return True

    def run(self) -> int:
        period = 1.0 / self.rate_hz
        print("watchdog: centre=[%.3f %.3f %.3f] radius=%.2f m rate=%.0f Hz "
              "(Ctrl-C to stop)"
              % (self.center[0], self.center[1], self.center[2],
                 self.radius, self.rate_hz))
        try:
            while not self.tripped:
                t0 = time.monotonic()
                if self.check_once():
                    return 2
                dt = period - (time.monotonic() - t0)
                if dt > 0:
                    time.sleep(dt)
        except KeyboardInterrupt:
            print("\nwatchdog: stopped by user (no breach)")
            return 0
        return 2


def _self_test() -> int:
    """Offline validation of the breach logic, timing and fail-safe path."""
    calls = {"disable": 0}

    def disabled() -> bool:
        calls["disable"] += 1
        return True

    # inside the sphere -> no trip
    wd = SafetyWatchdog([0, 0, 0], 0.30, lambda: (0.10, 0.0, 0.0), disabled)
    assert wd.check_once() is False and not wd.tripped

    # outside -> trip + disable, fast
    wd2 = SafetyWatchdog([0, 0, 0], 0.30, lambda: (0.50, 0.0, 0.0), disabled)
    t0 = time.monotonic()
    assert wd2.check_once() is True and wd2.tripped
    assert (time.monotonic() - t0) < 0.5
    assert calls["disable"] == 1

    # read error -> fail-safe trip + disable
    def boom() -> Tuple[float, float, float]:
        raise RuntimeError("no robot")

    wd3 = SafetyWatchdog([0, 0, 0], 0.30, boom, disabled)
    assert wd3.check_once() is True and wd3.tripped
    assert calls["disable"] == 2

    # boundary: exactly on radius is allowed; just beyond trips
    wd4 = SafetyWatchdog([0, 0, 0], 0.30, lambda: (0.30, 0.0, 0.0), disabled)
    assert wd4.check_once() is False
    wd5 = SafetyWatchdog([0, 0, 0], 0.30, lambda: (0.3001, 0.0, 0.0), disabled)
    assert wd5.check_once() is True

    print("SELF-TEST PASS (disable calls=%d)" % calls["disable"])
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Measured-TCP safety watchdog")
    ap.add_argument("--ip", default=DEFAULT_IP, help="Duco controller IP")
    ap.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                    help="allowed sphere radius (m), default 0.30")
    ap.add_argument("--center-xyz", type=float, nargs=3,
                    metavar=("X", "Y", "Z"),
                    help="sphere centre (m); default: current TCP")
    ap.add_argument("--ns", default=DEFAULT_NS, help="commander namespace")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE,
                    help="poll rate Hz (>=5)")
    ap.add_argument("--reader", default=None,
                    help="path to read_tcp_pose (default tools/read_tcp_pose)")
    ap.add_argument("--abort-mode", choices=("disable", "switch"),
                    default="disable",
                    help="breach action: call <ns>/disable, or switch "
                         "controllers back to a JTC hold")
    ap.add_argument("--abort-deactivate", default="cartesian_motion_controller",
                    help="(switch mode) controller to deactivate on breach")
    ap.add_argument("--abort-activate", default="arm_1_controller",
                    help="(switch mode) JTC hold controller to activate on breach")
    ap.add_argument("--self-test", action="store_true",
                    help="run offline self-test and exit")
    args = ap.parse_args(argv)

    if args.self_test:
        return _self_test()

    if args.center_xyz is None:
        try:
            args.center_xyz = list(read_tcp_xyz(args.ip, args.reader))
            print("auto-centred on current TCP [%.3f %.3f %.3f]"
                  % tuple(args.center_xyz))
        except Exception as exc:  # noqa: BLE001
            print("ERROR: cannot read TCP to auto-centre: %r" % exc,
                  file=sys.stderr)
            return 1

    if args.abort_mode == "switch":
        abort_fn = lambda: switch_to_hold(args.abort_deactivate, args.abort_activate)  # noqa: E731
    else:
        abort_fn = lambda: call_disable(args.ns)  # noqa: E731

    wd = SafetyWatchdog(
        args.center_xyz, args.radius,
        read_fn=lambda: read_tcp_xyz(args.ip, args.reader),
        disable_fn=abort_fn,
        rate_hz=args.rate)
    return wd.run()


if __name__ == "__main__":
    raise SystemExit(main())
