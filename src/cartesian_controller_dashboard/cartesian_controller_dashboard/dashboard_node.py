"""Web dashboard node for the FZI cartesian-controller stack.

This node is a **pure UI / monitoring** layer.  It does **not** launch
any controllers, run a wrench relay, or perform safety supervision --
those responsibilities live in :package:`duco_cartesian_control`.

Responsibilities
----------------

* subscribe to ``/duco_cartesian_control/state`` (JSON status from the
  orchestrator) so the dashboard reflects engaged/tripped state and
  the orchestrator's safety thresholds;
* subscribe to the live wrench topic and to ``/joint_states`` so the
  dashboard can show fresh data and freshness pills even before the
  orchestrator publishes its first state;
* call ``/duco_cartesian_control/engage`` and
  ``/duco_cartesian_control/disengage`` (``std_srvs/srv/Trigger``) when
  the operator clicks a button;
* read and write a curated list of parameters on the FZI Cartesian
  controller (e.g. ``cartesian_force_controller``) using the standard
  ``rcl_interfaces`` parameter services.

The HTTP server runs in a daemon thread; rclpy callbacks are dispatched
on a :class:`MultiThreadedExecutor` with a :class:`ReentrantCallbackGroup`
so synchronous service calls issued from the HTTP handler thread do
not deadlock with the ROS spin thread.
"""

from __future__ import annotations

import json
import math
import mimetypes
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import (PoseStamped, WrenchStamped)
from rcl_interfaces.msg import (Parameter, ParameterType, ParameterValue)
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# `common.config_manager` exposes the project's robot_config.yaml.  The
# dashboard's "Tool frames" editor reads `duco_robot_bringup.aux_frames`
# from the resolved config and writes back via the line-based
# `save_aux_frames` helper (which preserves comments).  Imported with a
# soft fallback so the dashboard still starts in environments where
# `common` is not on the PYTHONPATH (the API simply returns a clear
# error in that case).
try:
    from common.config_manager import (  # type: ignore
        get_config as _get_config,
        read_aux_frames as _read_aux_frames,
        save_aux_frames as _save_aux_frames,
    )
    _COMMON_IMPORT_ERROR: Optional[str] = None
except Exception as _exc:  # noqa: BLE001
    _get_config = None  # type: ignore
    _read_aux_frames = None  # type: ignore
    _save_aux_frames = None  # type: ignore
    _COMMON_IMPORT_ERROR = f"{type(_exc).__name__}: {_exc}"

# ``duco_robot_bringup.urdf_loader.update_aux_frames`` rewrites the
# ``<origin>`` of existing aux-frame joints in a URDF string, returning
# a new URDF that can be pushed to ``robot_state_publisher`` via
# SetParameters for live tool-frame updates.  Soft-imported for the
# same reason as ``common.config_manager`` above.
try:
    from duco_robot_bringup.urdf_loader import (  # type: ignore
        update_aux_frames as _update_aux_frames,
    )
    _URDF_LOADER_IMPORT_ERROR: Optional[str] = None
except Exception as _exc:  # noqa: BLE001
    _update_aux_frames = None  # type: ignore
    _URDF_LOADER_IMPORT_ERROR = f"{type(_exc).__name__}: {_exc}"


# ---------------------------------------------------------------------------
# Time-windowed publish-rate estimator.
#
# Each subscriber callback (or TF sampler timer) calls ``tick()`` once per
# message; ``hz()`` returns the rate computed over the last
# ``window_s`` seconds of arrivals.  Using a *time* window (rather than
# a fixed sample count) means the computed Hz itself is stable at
# whatever the publisher's rate happens to be -- a 250 Hz wrench
# integrates over ~250 samples, a 5 Hz orchestrator over ~5, but both
# produce a once-per-second number that varies by at most a couple of
# counts.
#
# ``hz()`` returns ``None`` when fewer than two samples are inside the
# current window *or* the most recent arrival is older than
# ``stale_after`` seconds (so the displayed Hz drops to "no data" as
# soon as a publisher stops, instead of decaying through a long false
# history).
#
# A short cache (``update_period``, default 1 s) keeps the value
# returned to the HTTP poller constant within each integration window
# even when /api/live is polled at 5 Hz: each poll reuses the last
# computation instead of re-counting the deque.  The stale check runs
# on every call regardless, so a stopped publisher still shows
# "no data" within ``stale_after`` seconds.
#
# Reads are intentionally lock-free: the underlying ``deque`` is only
# mutated on the rclpy spin thread, and ``hz()`` snapshots ``len`` +
# endpoints atomically enough for a UI readout.
# ---------------------------------------------------------------------------
class _RateTracker:
    __slots__ = ("_buf", "_window_s", "_stale_after", "_update_period",
                 "_cached_hz", "_cached_mono")

    def __init__(self, window_s: float = 1.0,
                 stale_after: float = 1.0,
                 update_period: float = 1.0) -> None:
        self._buf: Deque[float] = deque()
        self._window_s = float(window_s)
        self._stale_after = float(stale_after)
        self._update_period = float(update_period)
        self._cached_hz: Optional[float] = None
        self._cached_mono: Optional[float] = None

    def tick(self, t: Optional[float] = None) -> None:
        ts = time.monotonic() if t is None else t
        buf = self._buf
        buf.append(ts)
        # Evict samples older than the integration window so the
        # deque size stays bounded by the publish rate, not by
        # uptime.
        cutoff = ts - self._window_s
        while buf and buf[0] < cutoff:
            buf.popleft()

    def hz(self) -> Optional[float]:
        now = time.monotonic()
        buf = self._buf
        # Re-evict here too so a stopped publisher's old samples
        # don't keep the rate artificially high until the next tick
        # (and so the stale check below sees the most recent state).
        cutoff = now - self._window_s
        while buf and buf[0] < cutoff:
            buf.popleft()
        n = len(buf)
        if n < 2:
            return None
        last = buf[-1]
        if now - last > self._stale_after:
            # Publisher stopped: invalidate the cache so the next
            # ``tick()`` doesn't briefly resurrect an old value.
            self._cached_hz = None
            self._cached_mono = None
            return None
        # Return the cached value if recent enough.  This is what
        # gives the sticky-bar pills a stable once-per-second
        # readout -- the value changes only when a new integration
        # is performed, not on every HTTP poll.
        if (self._cached_hz is not None
                and self._cached_mono is not None
                and now - self._cached_mono < self._update_period):
            return self._cached_hz
        first = buf[0]
        span = last - first
        if span <= 0.0:
            # Single instant: keep the previous reading rather than
            # report a spurious infinity.
            return self._cached_hz
        val = (n - 1) / span
        self._cached_hz = val
        self._cached_mono = now
        return val


# ---------------------------------------------------------------------------
# tunable parameter catalogue (curated, keyed by controller "kind").
#
# Each entry is ``(name, kind)`` where ``kind`` is "double" or "integer"
# so we can build a correctly-typed ``ParameterValue``.  The dashboard
# discovers which controller is currently *active* from the orchestrator's
# state topic and exposes the matching list as the editable controls.
#
# All FZI controllers share the base ``pd_gains.*`` and ``solver.*``
# inner-loop knobs; the compliance controller adds an apparent-stiffness
# matrix, which is the main parameter operators want to play with.
# ---------------------------------------------------------------------------
_BASE_TUNABLES: List[Tuple[str, str]] = [
    ("pd_gains.trans_x.p", "double"),
    ("pd_gains.trans_y.p", "double"),
    ("pd_gains.trans_z.p", "double"),
    ("pd_gains.rot_x.p",   "double"),
    ("pd_gains.rot_y.p",   "double"),
    ("pd_gains.rot_z.p",   "double"),
    ("solver.error_scale", "double"),
    ("solver.iterations",  "integer"),
]
_COMPLIANCE_EXTRAS: List[Tuple[str, str]] = [
    ("stiffness.trans_x", "double"),
    ("stiffness.trans_y", "double"),
    ("stiffness.trans_z", "double"),
    ("stiffness.rot_x",   "double"),
    ("stiffness.rot_y",   "double"),
    ("stiffness.rot_z",   "double"),
]
_TUNABLES_BY_KIND: Dict[str, List[Tuple[str, str]]] = {
    "force":      list(_BASE_TUNABLES),
    "motion":     list(_BASE_TUNABLES),
    "compliance": list(_BASE_TUNABLES) + list(_COMPLIANCE_EXTRAS),
}


# ---------------------------------------------------------------------------
# orchestrator safety thresholds editable from the dashboard.
#
# These parameters live on the ``/duco_cartesian_control`` node (see
# :class:`duco_cartesian_control.control_node.CartesianControlNode`) and
# already accept live parameter updates -- the orchestrator validates
# they are non-negative and refuses changes that would interrupt the
# safety supervisor's invariants.  The dashboard simply forwards a
# typed SetParameters request and reports per-parameter results.
#
# Order is the display order in the UI.
_SAFETY_THRESHOLDS: List[Tuple[str, str, str]] = [
    # (param_name, kind, unit_label_for_ui)
    ("max_wrench_force",          "double", "N"),
    ("max_wrench_torque",         "double", "Nm"),
    ("engage_max_joint_velocity", "double", "rad/s"),
    ("ft_stale_after",            "double", "s"),
    ("joint_states_stale_after",  "double", "s"),
]
_SAFETY_THRESHOLD_NAMES = {n for (n, _k, _u) in _SAFETY_THRESHOLDS}


# ---------------------------------------------------------------------------
# wrench dead-zone editable from the dashboard.
#
# These parameters live on the ``ft_sensor_gravity_compensation`` node
# (see :module:`ft_sensor_gravity_compensation.compensation_node`) and
# are applied to the *compensated* wrench (after gravity + bias
# subtraction, before publishing).  Per-axis soft (continuous) shrinkage
# so the downstream cartesian-force / compliance controller does not
# see a step change at the dead-zone boundary.
#
# Each entry is ``(param_name, ui_label, unit_label)``.  All four are
# 3-element ``double[]`` arrays of N (force) or Nm (torque); validation
# (length 3, finite, >= 0) is done both here and in the gravity-comp
# node's on-set-parameters callback.
_WRENCH_DEADBAND_PARAMS: List[Tuple[str, str, str]] = [
    # (param_name, ui_label, unit_label)
    ("force_deadband",  "force",  "N"),
    ("torque_deadband", "torque", "Nm"),
]
_WRENCH_DEADBAND_NAMES = {n for (n, _l, _u) in _WRENCH_DEADBAND_PARAMS}


# ---------------------------------------------------------------------------
# static web assets
# ---------------------------------------------------------------------------
# The dashboard's HTML, CSS, and JS live as separate files in the
# package's ``static/`` directory (next to this module).  They're
# served by the HTTP handler at ``/static/<filename>``; the root URL
# ``/`` serves ``static/index.html``.
#
# Files are read from disk on every request through an mtime-keyed
# cache, so ``colcon build --symlink-install`` makes the static
# assets live-editable -- save the file, reload the browser, no
# node restart required.
_STATIC_DIR = Path(__file__).resolve().parent / "static"

_static_cache: Dict[str, Tuple[float, bytes]] = {}
_static_cache_lock = threading.Lock()


def _load_static(rel_path: str) -> Tuple[bytes, str]:
    """Load ``static/<rel_path>``, returning ``(bytes, mime)``.

    The MIME type is guessed from the filename and forced to UTF-8 for
    text payloads.  Raises :class:`FileNotFoundError` for missing
    files or paths that escape the static dir (e.g. ``..`` traversal).
    """
    # Resolve and reject any path that escapes the static dir.
    target = (_STATIC_DIR / rel_path).resolve()
    try:
        target.relative_to(_STATIC_DIR)
    except ValueError as exc:
        raise FileNotFoundError(rel_path) from exc
    if not target.is_file():
        raise FileNotFoundError(rel_path)
    mtime = target.stat().st_mtime
    key = str(target)
    with _static_cache_lock:
        cached = _static_cache.get(key)
        if cached is None or cached[0] != mtime:
            data = target.read_bytes()
            _static_cache[key] = (mtime, data)
        else:
            data = cached[1]
    mime, _enc = mimetypes.guess_type(target.name)
    if mime is None:
        mime = "application/octet-stream"
    if mime.startswith("text/") or mime in (
            "application/javascript", "application/json"):
        mime += "; charset=utf-8"
    return data, mime


# ---------------------------------------------------------------------------
# request handler
# ---------------------------------------------------------------------------
class _DashboardHandler(BaseHTTPRequestHandler):
    """Routes JSON API + serves the static HTML for the dashboard."""

    def __init__(self, *args, dashboard=None, **kwargs):
        self._dashboard = dashboard  # type: DashboardNode
        super().__init__(*args, **kwargs)

    def log_message(self, _format, *args):  # noqa: N802
        return

    # ---- response helpers -------------------------------------------------
    def _send_json(self, status: int, payload: Any) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_static(self, body: bytes, mime: str,
                     status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        # Disable caching so live edits with --symlink-install show up
        # immediately on the next page load.  The static payload is
        # only a few tens of KB so re-fetching costs nothing.
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self) -> Any:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return None
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"invalid JSON body: {exc}") from None

    # ---- Server-Sent Events: live URDF transforms -----------------------
    # ``/api/urdf_tf_stream`` opens a long-lived SSE connection that
    # pushes the latest tf snapshot at ~30 Hz.  This is much smoother
    # than per-frame polling (no HTTP round-trip per update) and a
    # single connection per browser tab is plenty -- the
    # ThreadingHTTPServer spawns one thread per open connection, so
    # the ``time.sleep`` cadence here doesn't block anything else.
    _URDF_STREAM_HZ = 30.0

    def _serve_urdf_tf_stream(self) -> None:
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache, no-transform")
            self.send_header("Connection", "keep-alive")
            # Some reverse-proxies buffer responses by default; this
            # header disables that for nginx/etc. while being a no-op
            # for direct browser connections.
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return
        period = 1.0 / self._URDF_STREAM_HZ
        next_t = time.monotonic()
        while True:
            try:
                payload = self._dashboard.api_urdf_tf()
            except Exception as exc:  # noqa: BLE001
                payload = {
                    "ok": False,
                    "message": f"{type(exc).__name__}: {exc}",
                }
            body = (b"data: "
                    + json.dumps(payload, separators=(",", ":")).encode("utf-8")
                    + b"\n\n")
            try:
                self.wfile.write(body)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # We're behind schedule (e.g. system load).  Reset the
                # deadline so we don't busy-loop trying to catch up.
                next_t = time.monotonic()

    # ---- routes -----------------------------------------------------------
    def do_GET(self):  # noqa: N802
        path = urlparse(self.path).path
        try:
            if path in ("/", "/index.html"):
                body, mime = _load_static("index.html")
                self._send_static(body, mime)
                return
            if path.startswith("/static/"):
                rel = path[len("/static/"):]
                try:
                    body, mime = _load_static(rel)
                except FileNotFoundError:
                    self.send_response(404)
                    self.end_headers()
                    return
                self._send_static(body, mime)
                return
            if path == "/api/state":
                self._send_json(200, self._dashboard.api_state_full())
                return
            if path == "/api/live":
                self._send_json(200, self._dashboard.api_state_live())
                return
            if path == "/api/params":
                self._send_json(200, self._dashboard.api_params())
                return
            if path == "/api/urdf_model":
                self._send_json(200, self._dashboard.api_urdf_model())
                return
            if path == "/api/urdf_tf":
                self._send_json(200, self._dashboard.api_urdf_tf())
                return
            if path == "/api/urdf_tf_stream":
                # Long-lived SSE connection -- handled inline and does
                # not return until the client drops.
                self._serve_urdf_tf_stream()
                return
            if path == "/api/aux_frames":
                self._send_json(200, self._dashboard.api_get_aux_frames())
                return
            if path == "/api/safety_thresholds":
                self._send_json(
                    200, self._dashboard.api_get_safety_thresholds())
                return
            if path == "/api/wrench_deadband":
                self._send_json(
                    200, self._dashboard.api_get_wrench_deadband())
                return
            self.send_response(404)
            self.end_headers()
        except Exception as exc:  # noqa: BLE001
            self._send_json(500, {"error": f"{type(exc).__name__}: {exc}"})

    def do_POST(self):  # noqa: N802
        path = urlparse(self.path).path
        try:
            if path == "/api/engage":
                self._send_json(200, self._dashboard.api_engage())
                return
            if path == "/api/disengage":
                self._send_json(200, self._dashboard.api_disengage())
                return
            if path == "/api/param":
                body = self._read_json_body() or {}
                self._send_json(200, self._dashboard.api_set_param(body))
                return
            if path == "/api/active_controller":
                body = self._read_json_body() or {}
                self._send_json(
                    200, self._dashboard.api_set_active_controller(body))
                return
            if path == "/api/jog":
                body = self._read_json_body() or {}
                self._send_json(200, self._dashboard.api_jog(body))
                return
            if path == "/api/snap_target":
                body = self._read_json_body() or {}
                self._send_json(200, self._dashboard.api_snap_target(body))
                return
            if path == "/api/aux_frames":
                body = self._read_json_body() or {}
                self._send_json(200, self._dashboard.api_set_aux_frames(body))
                return
            if path == "/api/safety_thresholds":
                body = self._read_json_body() or {}
                self._send_json(
                    200,
                    self._dashboard.api_set_safety_thresholds(body))
                return
            if path == "/api/wrench_deadband":
                body = self._read_json_body() or {}
                self._send_json(
                    200,
                    self._dashboard.api_set_wrench_deadband(body))
                return
            self.send_response(404)
            self.end_headers()
        except RuntimeError as exc:
            # Operational rejection (e.g. service unavailable, engage
            # refused) -> 409 so the JS can surface the message.
            self._send_json(409, {"error": str(exc)})
        except Exception as exc:  # noqa: BLE001
            self._send_json(500, {"error": f"{type(exc).__name__}: {exc}"})


# ---------------------------------------------------------------------------
# dashboard node
# ---------------------------------------------------------------------------
class DashboardNode(Node):
    """ROS node + HTTP server for the cartesian-controller dashboard."""

    _PARAM_DECLARATIONS: List[Tuple[str, object]] = [
        # connectivity ---------------------------------------------------
        ("orchestrator_ns",  "/duco_cartesian_control"),
        # Catalogue of FZI controller node names this dashboard knows
        # how to talk to.  Must match the orchestrator's
        # ``available_controllers`` list (this is the *fallback* for
        # bootstrap before the orchestrator's state topic arrives).
        ("available_controllers", [
            "cartesian_force_controller",
            "cartesian_motion_controller",
            "cartesian_compliance_controller",
        ]),
        ("default_controller_name",  "cartesian_force_controller"),
        ("wrench_topic",     "/duco_ft_sensor/wrench_compensated"),
        ("joint_states_topic", "/joint_states"),
        # Fully-qualified name (relative to root, with leading slash) of
        # the gravity-compensation node whose ``force_deadband`` /
        # ``torque_deadband`` parameters the dashboard's "Wrench
        # deadband" editor targets.  Default matches the node name used
        # by ``ft_sensor_gravity_compensation.compensation_node``.
        ("gravity_compensation_node", "/ft_sensor_gravity_compensation"),
        # Frames used for jog / snap-target publishes on motion +
        # compliance controllers.  Must match the URDF that
        # controller_manager sees and the FZI YAML's
        # ``robot_base_link`` / ``end_effector_link``.
        ("base_frame", "base_link"),
        ("tool_frame", "link_6"),
        ("service_timeout_sec", 2.0),
        # http -----------------------------------------------------------
        ("host", "0.0.0.0"),
        ("port", 8120),
    ]

    def __init__(self) -> None:
        super().__init__("cartesian_controller_dashboard")
        for name, default in self._PARAM_DECLARATIONS:
            self.declare_parameter(name, default)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self._orchestrator_ns = str(gp("orchestrator_ns")).rstrip("/")
        # Catalogue of controller names we'll create per-controller param
        # clients for.  Order is not significant.
        avail = list(gp("available_controllers") or [])
        self._available_controllers: List[str] = [
            str(n).strip("/") for n in avail if str(n).strip("/")]
        if not self._available_controllers:
            self._available_controllers = ["cartesian_force_controller"]
        # Bootstrap default until orchestrator state arrives.  The cached
        # state's ``active_controller`` field overrides this at runtime.
        self._default_controller_name = str(
            gp("default_controller_name")).strip("/")
        if self._default_controller_name not in self._available_controllers:
            self._default_controller_name = self._available_controllers[0]
        self._wrench_topic = str(gp("wrench_topic"))
        self._joint_states_topic = str(gp("joint_states_topic"))
        self._gravcomp_ns = str(gp("gravity_compensation_node")).rstrip("/")
        if not self._gravcomp_ns.startswith("/"):
            self._gravcomp_ns = "/" + self._gravcomp_ns
        self._base_frame = str(gp("base_frame")).strip("/")
        self._tool_frame = str(gp("tool_frame")).strip("/")
        self._service_timeout = float(gp("service_timeout_sec"))
        self._host = str(gp("host"))
        self._port = int(gp("port"))

        # ---- live state cache (guarded by ``_lock``) -------------------
        self._lock = threading.RLock()
        self._control_state: Optional[Dict[str, Any]] = None
        self._control_state_mono: Optional[float] = None
        self._wrench: Optional[Dict[str, float]] = None
        self._wrench_mono: Optional[float] = None
        self._js_mono: Optional[float] = None

        # Per-source publish-rate trackers.  All four integrate over a
        # 1 s window and recompute (and refresh the cached readout)
        # at most once per second, so the sticky-bar Hz numbers are
        # stable to within ±1 count regardless of underlying publish
        # rate.  ``stale_after`` is per-source: the orchestrator is
        # slow (~5 Hz), so a 5 s silence is still considered alive;
        # wrench / joint_states / tf are fast (~250 Hz) and we treat
        # >1 s gaps as a stopped publisher.  Reads are lock-free (the
        # deques are only mutated on the rclpy spin thread, and
        # ``hz()`` only inspects head + tail snapshots).
        self._state_rate  = _RateTracker(window_s=1.0, stale_after=5.0)
        self._wrench_rate = _RateTracker(window_s=1.0, stale_after=1.0)
        self._js_rate     = _RateTracker(window_s=1.0, stale_after=1.0)
        self._tf_rate     = _RateTracker(window_s=1.0, stale_after=1.0)
        # Cached TF stamp (nanoseconds) used by ``_tick_tf_rate`` to
        # detect *new* TF samples and bump ``_tf_rate``.  Polled at
        # 50 Hz, so the displayed TF rate is capped at ~50 Hz -- enough
        # to confirm "data flowing", not a precise measurement of the
        # underlying publish rate.
        self._last_tf_stamp_ns: Optional[int] = None
        # Cached URDF skeleton parsed from /robot_description.  The
        # 3D-model panel polls /api/urdf_model once on load (and on
        # request) to seed its renderer; live joint motion comes from
        # /api/urdf_tf which uses the existing tf2 buffer.
        self._urdf_xml: Optional[str] = None        # raw XML payload
        self._urdf_model: Optional[Dict[str, Any]] = None  # parsed skeleton
        self._urdf_mono: Optional[float] = None     # when URDF arrived

        # ---- callback group (reentrant for HTTP-thread service calls) --
        self._cbgroup = ReentrantCallbackGroup()

        # ---- subscriptions --------------------------------------------
        wrench_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=20)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._sub_state = self.create_subscription(
            String, f"{self._orchestrator_ns}/state",
            self._on_state, state_qos)
        self._sub_wrench = self.create_subscription(
            WrenchStamped, self._wrench_topic, self._on_wrench, wrench_qos)
        self._sub_js = self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states, 50)

        # ``/robot_description`` is published by ``robot_state_publisher``
        # exactly once with TRANSIENT_LOCAL durability -- new subscribers
        # get the latched value on connection.  We re-parse it each time
        # in case ``robot_state_publisher`` ever republishes (e.g. after
        # a controller reload that swaps the URDF).
        urdf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._sub_urdf = self.create_subscription(
            String, "/robot_description", self._on_urdf, urdf_qos)

        # ---- service clients ------------------------------------------
        self._cli_engage = self.create_client(
            Trigger, f"{self._orchestrator_ns}/engage",
            callback_group=self._cbgroup)
        self._cli_disengage = self.create_client(
            Trigger, f"{self._orchestrator_ns}/disengage",
            callback_group=self._cbgroup)

        # Parameter clients on every FZI controller node we know about.
        # We pre-create them so switching the *active* controller does
        # not require any client churn.  Each entry maps controller_name
        # -> {"list": ..., "get": ..., "set": ...}.
        self._param_clients: Dict[str, Dict[str, Any]] = {}
        for ctrl in self._available_controllers:
            self._param_clients[ctrl] = {
                "list": self.create_client(
                    ListParameters, f"/{ctrl}/list_parameters",
                    callback_group=self._cbgroup),
                "get": self.create_client(
                    GetParameters, f"/{ctrl}/get_parameters",
                    callback_group=self._cbgroup),
                "set": self.create_client(
                    SetParameters, f"/{ctrl}/set_parameters",
                    callback_group=self._cbgroup),
            }
        # Service client for the orchestrator's *own* parameters --
        # used to flip ``active_controller_name`` at runtime.
        self._cli_orchestrator_set_params = self.create_client(
            SetParameters,
            f"{self._orchestrator_ns}/set_parameters",
            callback_group=self._cbgroup)

        # Service clients for the gravity-compensation node's parameters.
        # Used by the "Wrench deadband" editor to read the current
        # per-axis force / torque dead-zone (GetParameters) and to push
        # operator edits (SetParameters).  If the node is not running
        # the dashboard surfaces a clear timeout on both paths.
        self._cli_gravcomp_get_params = self.create_client(
            GetParameters,
            f"{self._gravcomp_ns}/get_parameters",
            callback_group=self._cbgroup)
        self._cli_gravcomp_set_params = self.create_client(
            SetParameters,
            f"{self._gravcomp_ns}/set_parameters",
            callback_group=self._cbgroup)

        # Service client for ``robot_state_publisher``'s parameters.
        # Used by the "Tool frames" editor to push an updated
        # ``robot_description`` URDF when the operator saves aux-frame
        # offsets, so the static TF tree refreshes live without
        # restarting ``duco_robot_bringup`` (rsp re-parses the URDF and
        # re-publishes ``/tf_static`` with the new offsets).  The name
        # ``/robot_state_publisher`` is the standard one for
        # ROS 2 Humble; if a remap is ever introduced this client will
        # simply time out and the API surfaces a clear error.
        self._cli_rsp_set_params = self.create_client(
            SetParameters,
            "/robot_state_publisher/set_parameters",
            callback_group=self._cbgroup)

        # The FZI cartesian controllers fork has been patched to react
        # to ``robot_description`` parameter updates on their own node
        # (not on /controller_manager): each controller rebuilds the
        # KDL chain at runtime and swaps it in atomically from the RT
        # update() thread, so saving new aux-frame offsets takes effect
        # inside the engaged controller without an unload/load.  We
        # already have ``self._param_clients[ctrl]['set']`` for each
        # available controller; the Tool-frames editor reuses those
        # clients.  See cartesian_controller_base.cpp::
        # ``onParameterUpdate`` for the receiving side.

        # ---- target_frame publishers (one per controller) -------------
        # The motion + compliance controllers consume PoseStamped on
        # ``/<name>/target_frame``; the force controller does not, but
        # publishing into it is harmless (no subscribers).  We pre-create
        # one publisher per available controller so the jog / snap UI
        # can route to whichever is active without churn.
        self._target_frame_pubs: Dict[str, Any] = {
            ctrl: self.create_publisher(
                PoseStamped, f"/{ctrl}/target_frame", 10)
            for ctrl in self._available_controllers
        }

        # ---- tf2 listener (current EE pose source for jog / snap) -----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Periodic TF-rate sampler.  tf2's C++ listener doesn't surface
        # a per-message Python callback, so we instead poll the buffer
        # at 50 Hz and count *new* stamps -- enough to confirm the TF
        # tree is updating at controller rate.
        self._tf_rate_timer = self.create_timer(
            0.02, self._tick_tf_rate, callback_group=self._cbgroup)

        # ---- HTTP server ----------------------------------------------
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None
        self._start_http()

    # ------------------------------------------------------------------
    # subscription callbacks
    # ------------------------------------------------------------------
    def _on_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._control_state = payload
            self._control_state_mono = time.monotonic()
        self._state_rate.tick()

    def _on_wrench(self, msg: WrenchStamped) -> None:
        with self._lock:
            self._wrench = {
                "fx": float(msg.wrench.force.x),
                "fy": float(msg.wrench.force.y),
                "fz": float(msg.wrench.force.z),
                "tx": float(msg.wrench.torque.x),
                "ty": float(msg.wrench.torque.y),
                "tz": float(msg.wrench.torque.z),
            }
            self._wrench_mono = time.monotonic()
        self._wrench_rate.tick()

    def _on_joint_states(self, msg: JointState) -> None:  # noqa: ARG002
        with self._lock:
            self._js_mono = time.monotonic()
        self._js_rate.tick()

    def _tick_tf_rate(self) -> None:
        """50 Hz sampler that bumps ``_tf_rate`` on each new TF stamp.

        ``tf2`` doesn't expose a per-message Python callback, so we
        peek at the latest available ``base_frame -> tool_frame`` and
        record a tick only when the stamp differs from the one seen
        last cycle.  This caps the *displayed* TF rate at the timer's
        50 Hz, which is plenty for a freshness indicator.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._tool_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0))
        except TransformException:
            return
        stamp_ns = int(t.header.stamp.sec) * 1_000_000_000 \
                 + int(t.header.stamp.nanosec)
        if stamp_ns <= 0:
            return  # static / unstamped TF -- no rate to compute
        if self._last_tf_stamp_ns is None \
           or stamp_ns != self._last_tf_stamp_ns:
            self._last_tf_stamp_ns = stamp_ns
            self._tf_rate.tick()

    def _on_urdf(self, msg: String) -> None:
        """Cache and parse the latched ``/robot_description`` payload.

        Re-parsed on every message in case ``robot_state_publisher``
        ever republishes (e.g. a controller switch that swaps the
        URDF).  Parsing failures are logged but don't crash the node.
        """
        xml_text = msg.data or ""
        model = _parse_urdf(xml_text)
        with self._lock:
            self._urdf_xml = xml_text
            self._urdf_model = model
            self._urdf_mono = time.monotonic()
        if model is None:
            self.get_logger().warn(
                "received /robot_description but could not parse URDF "
                f"({len(xml_text)} bytes)")
        else:
            self.get_logger().info(
                f"URDF received: {len(model.get('links', []))} links, "
                f"{len(model.get('joints', []))} joints, "
                f"root={model.get('root_link')!r}")

    # ------------------------------------------------------------------
    # HTTP server lifecycle
    # ------------------------------------------------------------------
    def _start_http(self) -> None:
        try:
            handler = partial(_DashboardHandler, dashboard=self)
            self._httpd = ThreadingHTTPServer((self._host, self._port), handler)
            self._http_thread = threading.Thread(
                target=self._httpd.serve_forever, daemon=True,
                name=f"cartesian-dashboard-{self._port}")
            self._http_thread.start()
            shown = (_local_ip_hint() if self._host == "0.0.0.0"
                     else self._host)
            self.get_logger().info(
                f"web dashboard: http://{shown}:{self._port}/  "
                f"(orchestrator={self._orchestrator_ns!r}, "
                f"controllers={self._available_controllers})")
        except OSError as exc:
            self.get_logger().error(
                f"web dashboard: failed to bind {self._host}:{self._port}: "
                f"{exc}")

    def destroy_node(self) -> bool:
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()

    # ------------------------------------------------------------------
    # ROS service / param helpers
    # ------------------------------------------------------------------
    def _service_call_sync(self, client, request, timeout_s: float):
        """Issue ``call_async`` and block until done or timeout.

        Mirrors the orchestrator's helper -- see
        :class:`duco_cartesian_control.control_node.CartesianControlNode`.
        """
        if not client.wait_for_service(timeout_sec=min(0.5, timeout_s)):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception:  # noqa: BLE001
            return None

    def _trigger(self, client, kind: str) -> Dict[str, Any]:
        if not client.wait_for_service(timeout_sec=0.2):
            raise RuntimeError(
                f"{kind} service unavailable -- is duco_cartesian_control "
                "running?")
        resp = self._service_call_sync(
            client, Trigger.Request(), self._service_timeout)
        if resp is None:
            raise RuntimeError(
                f"{kind}: no response within {self._service_timeout:.1f} s")
        if not resp.success:
            raise RuntimeError(resp.message or f"{kind} failed")
        return {"ok": True, "message": resp.message or kind}

    def _get_param_values(self, controller: str, names: List[str]
                          ) -> Dict[str, Optional[Any]]:
        """Read parameter values from a specific controller node.

        Returns a dict ``{name: value_or_None}``.  ``None`` means the
        parameter does not exist (or the controller is not running).
        """
        result: Dict[str, Optional[Any]] = {n: None for n in names}
        if not names:
            return result
        clients = self._param_clients.get(controller)
        if clients is None:
            return result
        get_cli = clients["get"]
        if not get_cli.wait_for_service(timeout_sec=0.2):
            return result
        req = GetParameters.Request()
        req.names = list(names)
        resp = self._service_call_sync(
            get_cli, req, self._service_timeout)
        if resp is None:
            return result
        for name, pv in zip(names, resp.values):
            result[name] = _decode_param_value(pv)
        return result

    def _set_param(self, controller: str, name: str, kind: str,
                   raw_value: Any) -> Tuple[bool, str]:
        clients = self._param_clients.get(controller)
        if clients is None:
            return False, f"unknown controller {controller!r}"
        set_cli = clients["set"]
        if not set_cli.wait_for_service(timeout_sec=0.2):
            return False, (f"set_parameters service unavailable -- is the "
                           f"{controller!r} controller running?")
        param = Parameter()
        param.name = name
        pv = ParameterValue()
        try:
            if kind == "integer":
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = int(raw_value)
            elif kind == "double":
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(raw_value)
            else:
                return False, f"unsupported kind: {kind!r}"
        except (TypeError, ValueError) as exc:
            return False, f"value {raw_value!r}: {exc}"
        param.value = pv
        req = SetParameters.Request()
        req.parameters = [param]
        resp = self._service_call_sync(
            set_cli, req, self._service_timeout)
        if resp is None:
            return False, "set_parameters: no response"
        if not resp.results or not resp.results[0].successful:
            why = (resp.results[0].reason
                   if resp.results else "unknown error")
            return False, why or "set_parameters failed"
        return True, "ok"

    def _set_orchestrator_active_controller(self, name: str
                                            ) -> Tuple[bool, str]:
        """Flip the orchestrator's ``active_controller_name`` parameter.

        The orchestrator validates the value against its own catalogue
        and refuses the change while engaged -- both surface here as
        ``(False, reason)`` from the SetParameters response.
        """
        if not self._cli_orchestrator_set_params.wait_for_service(
                timeout_sec=0.2):
            return False, (f"orchestrator set_parameters service "
                           f"unavailable at {self._orchestrator_ns!r}")
        param = Parameter()
        param.name = "active_controller_name"
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(name)
        param.value = pv
        req = SetParameters.Request()
        req.parameters = [param]
        resp = self._service_call_sync(
            self._cli_orchestrator_set_params, req, self._service_timeout)
        if resp is None:
            return False, "orchestrator set_parameters: no response"
        if not resp.results or not resp.results[0].successful:
            why = (resp.results[0].reason
                   if resp.results else "unknown error")
            return False, why or "set_parameters failed"
        return True, "ok"

    def _set_orchestrator_doubles(self, updates: Dict[str, float]
                                  ) -> Tuple[bool, List[Dict[str, Any]]]:
        """Push a batch of ``double`` parameters to the orchestrator.

        Returns ``(ok, per_param_results)`` where ``ok`` is True iff
        the service responded AND every parameter was accepted, and
        ``per_param_results`` is a list of dicts
        ``{"name": str, "successful": bool, "reason": str}`` matching
        the order of ``updates``.  Used by the safety-threshold editor.
        """
        if not updates:
            return True, []
        if not self._cli_orchestrator_set_params.wait_for_service(
                timeout_sec=0.2):
            return False, [{
                "name": n, "successful": False,
                "reason": (f"orchestrator set_parameters service "
                           f"unavailable at {self._orchestrator_ns!r}"),
            } for n in updates]
        params: List[Parameter] = []
        ordered_names: List[str] = []
        for name, value in updates.items():
            p = Parameter()
            p.name = name
            pv = ParameterValue()
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
            p.value = pv
            params.append(p)
            ordered_names.append(name)
        req = SetParameters.Request()
        req.parameters = params
        resp = self._service_call_sync(
            self._cli_orchestrator_set_params, req, self._service_timeout)
        if resp is None:
            return False, [{
                "name": n, "successful": False,
                "reason": (f"no response within "
                           f"{self._service_timeout:.1f}s"),
            } for n in ordered_names]
        results: List[Dict[str, Any]] = []
        for name, r in zip(ordered_names, resp.results):
            results.append({
                "name":       name,
                "successful": bool(r.successful),
                "reason":     str(r.reason or ""),
            })
        all_ok = all(r["successful"] for r in results)
        return all_ok, results

    # ------------------------------------------------------------------
    # active-controller helpers (cached from orchestrator state topic)
    # ------------------------------------------------------------------
    def _active_controller(self) -> str:
        """Return whichever controller the orchestrator currently
        considers *active* (settable param).  Falls back to the
        bootstrap default until the first state message arrives.
        """
        with self._lock:
            ctl = self._control_state
        if ctl:
            v = ctl.get("active_controller")
            if isinstance(v, str) and v:
                return v
        return self._default_controller_name

    def _active_kind(self) -> str:
        """Return the kind ("force"/"motion"/"compliance") of the
        active controller, looked up from the orchestrator's published
        catalogue.  Defaults to "force" if unknown.
        """
        active = self._active_controller()
        with self._lock:
            ctl = self._control_state or {}
        for entry in ctl.get("available_controllers", []) or []:
            if (isinstance(entry, dict)
                    and entry.get("name") == active
                    and isinstance(entry.get("kind"), str)):
                return entry["kind"]
        return "force"

    def _tunables_for_active(self) -> List[Tuple[str, str]]:
        return list(_TUNABLES_BY_KIND.get(self._active_kind(),
                                          _BASE_TUNABLES))

    # ------------------------------------------------------------------
    # snapshots used by the HTTP handler
    # ------------------------------------------------------------------
    def api_state_full(self) -> Dict[str, Any]:
        active = self._active_controller()
        tunables = self._tunables_for_active()
        names = [n for (n, _) in tunables]
        values = self._get_param_values(active, names)
        params = [
            {"name": n, "kind": k, "value": values.get(n)}
            for (n, k) in tunables
        ]
        live = self._snapshot_live_locked()
        live["params"] = params
        live["controller_name"] = active
        live["controller_kind"] = self._active_kind()
        live["orchestrator_ns"] = self._orchestrator_ns
        return live

    def api_state_live(self) -> Dict[str, Any]:
        return self._snapshot_live_locked()

    def api_params(self) -> Dict[str, Any]:
        active = self._active_controller()
        tunables = self._tunables_for_active()
        names = [n for (n, _) in tunables]
        values = self._get_param_values(active, names)
        return {
            "controller_name": active,
            "controller_kind": self._active_kind(),
            "params": [
                {"name": n, "kind": k, "value": values.get(n)}
                for (n, k) in tunables
            ],
        }

    def api_engage(self) -> Dict[str, Any]:
        return self._trigger(self._cli_engage, "engage")

    def api_disengage(self) -> Dict[str, Any]:
        return self._trigger(self._cli_disengage, "disengage")

    def api_set_param(self, body: Dict[str, Any]) -> Dict[str, Any]:
        name = body.get("name")
        kind = body.get("kind")
        value = body.get("value")
        if not isinstance(name, str) or not name:
            raise RuntimeError("missing 'name'")
        if kind not in ("double", "integer"):
            raise RuntimeError(
                f"unsupported 'kind' {kind!r}; expected 'double' or 'integer'")
        # Whitelist for safety: only allow names the dashboard advertises
        # for the *currently active* controller's kind.
        allowed = {n for (n, _) in self._tunables_for_active()}
        if name not in allowed:
            raise RuntimeError(
                f"parameter {name!r} is not in the dashboard's tunable "
                f"list for kind {self._active_kind()!r}")
        active = self._active_controller()
        ok, why = self._set_param(active, name, kind, value)
        if not ok:
            raise RuntimeError(why)
        return {"ok": True, "name": name, "value": value, "message": why,
                "controller_name": active}

    def api_set_active_controller(self, body: Dict[str, Any]
                                  ) -> Dict[str, Any]:
        """Switch which FZI controller engage will activate.

        Forwarded to the orchestrator's ``active_controller_name``
        parameter via SetParameters; the orchestrator validates and
        refuses the change while engaged.
        """
        name = body.get("name")
        if not isinstance(name, str) or not name:
            raise RuntimeError("missing 'name'")
        if name not in self._available_controllers:
            raise RuntimeError(
                f"{name!r} is not in available_controllers "
                f"({self._available_controllers})")
        ok, why = self._set_orchestrator_active_controller(name)
        if not ok:
            raise RuntimeError(why)
        return {"ok": True, "name": name, "message": why}

    # ------------------------------------------------------------------
    # safety thresholds editor
    # ------------------------------------------------------------------
    # The orchestrator owns the safety supervisor's limits and exposes
    # them as live-tunable parameters on its own node (see
    # ``CartesianControlNode._SAFETY_KNOBS``).  The dashboard provides a
    # typed editor for them so operators don't need to remember the
    # ``ros2 param set`` syntax.  The orchestrator itself validates
    # non-negativity and unknown names; we add a thin whitelist here so
    # an obvious typo from the browser does not blindly become a
    # SetParameters call on an unrelated parameter.
    def api_get_safety_thresholds(self) -> Dict[str, Any]:
        """Return the orchestrator's currently-known safety thresholds.

        We read directly from the cached ``control_state`` (which the
        orchestrator publishes ~5 Hz) rather than calling GetParameters
        again; this keeps the GET fast and avoids serialising every
        save round-trip through an extra RPC.
        """
        with self._lock:
            ctl = dict(self._control_state) if self._control_state else {}
            ctl_age = (time.monotonic() - self._control_state_mono
                       if self._control_state_mono is not None else None)
        limits = (ctl.get("limits") or {}) if isinstance(ctl, dict) else {}
        items: List[Dict[str, Any]] = []
        for name, kind, unit in _SAFETY_THRESHOLDS:
            value = limits.get(name)
            try:
                value = float(value) if value is not None else None
            except (TypeError, ValueError):
                value = None
            items.append({
                "name":  name,
                "kind":  kind,
                "unit":  unit,
                "value": value,
            })
        return {
            "ok":              True,
            "orchestrator_ns": self._orchestrator_ns,
            "orchestrator_live": (ctl_age is not None and ctl_age <= 5.0),
            "control_state_age": ctl_age,
            "thresholds":      items,
            "message": ("orchestrator-owned safety limits "
                        "(/duco_cartesian_control); live-tunable; "
                        "trips disengage the FZI controller"),
        }

    def api_set_safety_thresholds(self, body: Dict[str, Any]
                                  ) -> Dict[str, Any]:
        """Push edits to the orchestrator's safety thresholds.

        Body shape::

            {"thresholds": [{"name": str, "value": float}, ...]}

        Unknown names are rejected (the dashboard restricts the editor
        to ``_SAFETY_THRESHOLDS``); non-finite or negative values are
        rejected here before the round-trip.  The orchestrator does
        its own validation too, which we surface verbatim.
        """
        if not isinstance(body, dict):
            raise RuntimeError("body must be a JSON object")
        items = body.get("thresholds")
        if not isinstance(items, list):
            raise RuntimeError("body.thresholds must be a list")

        updates: Dict[str, float] = {}
        for idx, entry in enumerate(items):
            if not isinstance(entry, dict):
                raise RuntimeError(
                    f"thresholds[{idx}] must be an object")
            name = entry.get("name")
            if not isinstance(name, str) or not name:
                raise RuntimeError(
                    f"thresholds[{idx}].name must be a non-empty string")
            if name not in _SAFETY_THRESHOLD_NAMES:
                raise RuntimeError(
                    f"thresholds[{idx}].name={name!r} is not editable "
                    f"({sorted(_SAFETY_THRESHOLD_NAMES)})")
            raw = entry.get("value")
            try:
                value = float(raw)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    f"thresholds[{idx}].value not numeric: {exc}"
                ) from exc
            if not math.isfinite(value):
                raise RuntimeError(
                    f"thresholds[{idx}].value must be finite "
                    f"(got {raw!r})")
            if value < 0.0:
                raise RuntimeError(
                    f"thresholds[{idx}].name={name!r} must be >= 0 "
                    f"(got {value})")
            updates[name] = value

        if not updates:
            return {"ok": True, "updated": 0, "results": [],
                    "message": "no thresholds supplied"}

        ok, results = self._set_orchestrator_doubles(updates)
        successful = sum(1 for r in results if r["successful"])
        return {
            "ok":      ok,
            "updated": successful,
            "results": results,
            "message": (
                f"applied {successful}/{len(results)} threshold(s) "
                "to /duco_cartesian_control" if ok else
                f"orchestrator rejected {len(results) - successful}/"
                f"{len(results)} threshold(s)"
            ),
        }

    # ------------------------------------------------------------------
    # wrench dead-zone editor (gravity-compensation node parameters)
    # ------------------------------------------------------------------
    # The dead-zone is per-axis soft (continuous) shrinkage applied to
    # the compensated wrench just before it is published.  It lives on
    # the gravity-compensation node so that every downstream consumer
    # (the force / compliance controllers, the dashboard's live wrench
    # display, rosbag recorders) sees the same clean signal.  See
    # ``ft_sensor_gravity_compensation.compensation_node`` for the
    # validation rules: 3 doubles per parameter, non-negative, finite.
    def _get_gravcomp_double_arrays(self, names: List[str]
                                    ) -> Dict[str, Optional[List[float]]]:
        """Read ``double[]`` parameters from the gravity-comp node.

        Returns ``{name: list_or_None}``.  ``None`` means the parameter
        is not declared on the remote node OR the service is not
        reachable inside ``_service_timeout``.  Callers are responsible
        for distinguishing those two cases via ``service_available``
        in the caller's response payload if needed.
        """
        result: Dict[str, Optional[List[float]]] = {n: None for n in names}
        if not names:
            return result
        if not self._cli_gravcomp_get_params.wait_for_service(
                timeout_sec=0.2):
            return result
        req = GetParameters.Request()
        req.names = list(names)
        resp = self._service_call_sync(
            self._cli_gravcomp_get_params, req, self._service_timeout)
        if resp is None:
            return result
        for name, pv in zip(names, resp.values):
            decoded = _decode_param_value(pv)
            # We only accept double-array values for these parameters
            # (anything else means the node has a name collision with a
            # differently-typed parameter -- treat as "not available").
            if isinstance(decoded, list) and all(
                    isinstance(v, (int, float)) for v in decoded):
                result[name] = [float(v) for v in decoded]
            else:
                result[name] = None
        return result

    def _set_gravcomp_double_arrays(self, updates: Dict[str, List[float]]
                                    ) -> Tuple[bool, List[Dict[str, Any]]]:
        """Push a batch of ``double[]`` parameters to the gravity-comp node.

        Returns ``(ok, per_param_results)`` matching
        ``_set_orchestrator_doubles`` in shape.  Used by the
        wrench-deadband editor.
        """
        if not updates:
            return True, []
        if not self._cli_gravcomp_set_params.wait_for_service(
                timeout_sec=0.2):
            return False, [{
                "name": n, "successful": False,
                "reason": (f"gravity-compensation set_parameters "
                           f"service unavailable at {self._gravcomp_ns!r}"),
            } for n in updates]
        params: List[Parameter] = []
        ordered_names: List[str] = []
        for name, vec in updates.items():
            p = Parameter()
            p.name = name
            pv = ParameterValue()
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(v) for v in vec]
            p.value = pv
            params.append(p)
            ordered_names.append(name)
        req = SetParameters.Request()
        req.parameters = params
        resp = self._service_call_sync(
            self._cli_gravcomp_set_params, req, self._service_timeout)
        if resp is None:
            return False, [{
                "name": n, "successful": False,
                "reason": (f"no response within "
                           f"{self._service_timeout:.1f}s"),
            } for n in ordered_names]
        results: List[Dict[str, Any]] = []
        for name, r in zip(ordered_names, resp.results):
            results.append({
                "name":       name,
                "successful": bool(r.successful),
                "reason":     str(r.reason or ""),
            })
        all_ok = all(r["successful"] for r in results)
        return all_ok, results

    def api_get_wrench_deadband(self) -> Dict[str, Any]:
        """Return the current per-axis force / torque dead-zone vectors.

        Each entry's ``value`` is a 3-element list (xyz) or ``None`` if
        the gravity-compensation node is not reachable.
        """
        names = [n for (n, _l, _u) in _WRENCH_DEADBAND_PARAMS]
        values = self._get_gravcomp_double_arrays(names)
        items: List[Dict[str, Any]] = []
        any_missing = False
        for name, label, unit in _WRENCH_DEADBAND_PARAMS:
            v = values.get(name)
            if v is None:
                any_missing = True
            items.append({
                "name":  name,
                "label": label,
                "unit":  unit,
                "value": v,
            })
        return {
            "ok":                True,
            "node":              self._gravcomp_ns,
            "available":         not any_missing,
            "deadbands":         items,
            "message": (
                "per-axis soft deadband applied to the compensated "
                "wrench just before publishing; live-tunable on "
                f"{self._gravcomp_ns}"
            ),
        }

    def api_set_wrench_deadband(self, body: Dict[str, Any]
                                ) -> Dict[str, Any]:
        """Push edits to the gravity-comp node's dead-zone parameters.

        Body shape::

            {"deadbands": [{"name": str, "value": [x, y, z]}, ...]}

        Unknown names are rejected (the dashboard restricts the editor
        to ``_WRENCH_DEADBAND_PARAMS``); non-finite, non-3-vector, and
        negative values are rejected here before the round-trip.  The
        gravity-compensation node does its own validation too, which
        we surface verbatim.
        """
        if not isinstance(body, dict):
            raise RuntimeError("body must be a JSON object")
        items = body.get("deadbands")
        if not isinstance(items, list):
            raise RuntimeError("body.deadbands must be a list")

        updates: Dict[str, List[float]] = {}
        for idx, entry in enumerate(items):
            if not isinstance(entry, dict):
                raise RuntimeError(
                    f"deadbands[{idx}] must be an object")
            name = entry.get("name")
            if not isinstance(name, str) or not name:
                raise RuntimeError(
                    f"deadbands[{idx}].name must be a non-empty string")
            if name not in _WRENCH_DEADBAND_NAMES:
                raise RuntimeError(
                    f"deadbands[{idx}].name={name!r} is not editable "
                    f"({sorted(_WRENCH_DEADBAND_NAMES)})")
            raw = entry.get("value")
            if not isinstance(raw, list) or len(raw) != 3:
                raise RuntimeError(
                    f"deadbands[{idx}].value must be a 3-element list "
                    f"(got {raw!r})")
            vec: List[float] = []
            for j, v in enumerate(raw):
                try:
                    fv = float(v)
                except (TypeError, ValueError) as exc:
                    raise RuntimeError(
                        f"deadbands[{idx}].value[{j}] not numeric: {exc}"
                    ) from exc
                if not math.isfinite(fv):
                    raise RuntimeError(
                        f"deadbands[{idx}].value[{j}] must be finite "
                        f"(got {v!r})")
                if fv < 0.0:
                    raise RuntimeError(
                        f"deadbands[{idx}].value[{j}] must be >= 0 "
                        f"(got {fv})")
                vec.append(fv)
            updates[name] = vec

        if not updates:
            return {"ok": True, "updated": 0, "results": [],
                    "message": "no deadbands supplied"}

        ok, results = self._set_gravcomp_double_arrays(updates)
        successful = sum(1 for r in results if r["successful"])
        return {
            "ok":      ok,
            "updated": successful,
            "results": results,
            "message": (
                f"applied {successful}/{len(results)} deadband(s) "
                f"to {self._gravcomp_ns}" if ok else
                f"gravity-compensation rejected "
                f"{len(results) - successful}/{len(results)} deadband(s)"
            ),
        }

    # ------------------------------------------------------------------
    # jog / snap-target -- per-controller motion-style command surface
    # ------------------------------------------------------------------
    # Both ``cartesian_motion_controller`` and
    # ``cartesian_compliance_controller`` consume PoseStamped on
    # ``/<name>/target_frame`` and snapshot the current EE pose on
    # activate.  The dashboard's "control" panel for those kinds lets
    # the operator nudge that target -- each press computes
    # ``new_target = current_ee_pose + delta`` (in the base frame) and
    # publishes it.  The force controller has no such input, so its
    # control panel is purely informational.
    def _lookup_current_pose(self) -> PoseStamped:
        """Look up ``base_frame -> tool_frame`` from /tf and return
        it as a :class:`PoseStamped`.  Raises RuntimeError on failure
        so the HTTP layer can surface a clean 409.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._tool_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except TransformException as exc:
            raise RuntimeError(
                f"tf lookup {self._base_frame!r} -> "
                f"{self._tool_frame!r} failed: {exc}")
        ps = PoseStamped()
        ps.header.frame_id = self._base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(t.transform.translation.x)
        ps.pose.position.y = float(t.transform.translation.y)
        ps.pose.position.z = float(t.transform.translation.z)
        ps.pose.orientation = t.transform.rotation
        return ps

    @staticmethod
    def _quat_mul(q1: Tuple[float, float, float, float],
                  q2: Tuple[float, float, float, float]
                  ) -> Tuple[float, float, float, float]:
        """Hamilton product on (x, y, z, w) tuples."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @classmethod
    def _axis_angle_quat(cls, drx: float, dry: float, drz: float
                         ) -> Tuple[float, float, float, float]:
        """Compose a quaternion that rotates by drz about Z, then dry
        about Y, then drx about X (extrinsic, base-frame axes).
        Returns identity when all three angles are zero.
        """
        def axq(axis: int, ang: float) -> Tuple[float, float, float, float]:
            s = math.sin(ang / 2.0)
            c = math.cos(ang / 2.0)
            if axis == 0:
                return (s, 0.0, 0.0, c)
            if axis == 1:
                return (0.0, s, 0.0, c)
            return (0.0, 0.0, s, c)
        q = (0.0, 0.0, 0.0, 1.0)
        q = cls._quat_mul(axq(0, drx), q)
        q = cls._quat_mul(axq(1, dry), q)
        q = cls._quat_mul(axq(2, drz), q)
        return q

    def _publish_target_pose(self, controller: str, pose: PoseStamped
                             ) -> None:
        pub = self._target_frame_pubs.get(controller)
        if pub is None:
            raise RuntimeError(
                f"no target_frame publisher for controller {controller!r}")
        pub.publish(pose)

    def api_jog(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """Nudge the active motion / compliance controller's target.

        Body fields (all optional, default 0): ``dx``, ``dy``, ``dz``
        (metres), ``drx``, ``dry``, ``drz`` (radians).  The new target
        is ``current_ee_pose * delta`` published in ``base_frame``.
        Refused unless the active controller's kind is
        ``motion`` or ``compliance``.
        """
        active = self._active_controller()
        kind = self._active_kind()
        if kind not in ("motion", "compliance"):
            raise RuntimeError(
                f"jog is only supported for motion / compliance controllers; "
                f"active is {active!r} (kind={kind!r})")

        def _f(key: str) -> float:
            v = body.get(key, 0.0)
            try:
                return float(v)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"{key!r}: {exc}")
        dx = _f("dx"); dy = _f("dy"); dz = _f("dz")
        drx = _f("drx"); dry = _f("dry"); drz = _f("drz")

        # Bounded translation step to avoid catastrophic typos.  10 cm
        # is plenty for incremental jog use-cases; rotation is capped
        # at ~30 deg per press for similar reasons.
        if max(abs(dx), abs(dy), abs(dz)) > 0.10:
            raise RuntimeError("translation step too large (>10 cm)")
        if max(abs(drx), abs(dry), abs(drz)) > math.radians(30.0):
            raise RuntimeError("rotation step too large (>30 deg)")

        pose = self._lookup_current_pose()
        pose.pose.position.x += dx
        pose.pose.position.y += dy
        pose.pose.position.z += dz

        if drx or dry or drz:
            qd = self._axis_angle_quat(drx, dry, drz)
            qc = (pose.pose.orientation.x, pose.pose.orientation.y,
                  pose.pose.orientation.z, pose.pose.orientation.w)
            qn = self._quat_mul(qd, qc)
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = qn

        self._publish_target_pose(active, pose)
        return {
            "ok": True,
            "controller_name": active,
            "kind": kind,
            "target": {
                "frame_id": pose.header.frame_id,
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z,
                },
                "orientation": {
                    "x": pose.pose.orientation.x,
                    "y": pose.pose.orientation.y,
                    "z": pose.pose.orientation.z,
                    "w": pose.pose.orientation.w,
                },
            },
            "delta": {"dx": dx, "dy": dy, "dz": dz,
                      "drx": drx, "dry": dry, "drz": drz},
        }

    def api_snap_target(self, _body: Dict[str, Any]) -> Dict[str, Any]:
        """Publish the current EE pose as the target.

        Useful as a "reset" after the controller has been chasing an
        old target -- after snapping, the arm will hold its current
        pose.  Only valid for motion / compliance.
        """
        return self.api_jog({})  # zero delta == snap to current pose

    # ------------------------------------------------------------------
    # tool-frame offsets (aux_frames in robot_config.yaml)
    # ------------------------------------------------------------------
    # The dashboard's "Tool frames" panel lets the operator tweak the
    # xyz / rpy of each aux_frame (e.g. ft_sensor_link, compliance_link)
    # without editing YAML by hand.  Changes are persisted to
    # ``config/robot_config.yaml`` via the line-targeted
    # ``common.config_manager.save_aux_frames`` helper which preserves
    # comments and unrelated keys; they take effect on the next
    # ``duco_robot_bringup`` launch.
    #
    # Adding, renaming, or removing aux_frames is intentionally NOT
    # exposed -- those edits ripple into ``fzi_zero_gravity.yaml``
    # (which names a specific end-effector link in the KDL chain), so
    # operators do them by hand.
    def api_get_aux_frames(self) -> Dict[str, Any]:
        """Return the current aux_frames list as on disk."""
        if _get_config is None or _read_aux_frames is None:
            raise RuntimeError(
                "common.config_manager not importable: "
                f"{_COMMON_IMPORT_ERROR}")
        cfg = _get_config()
        config_path = cfg.config_path
        if not config_path:
            raise RuntimeError("config_manager has not resolved a path")
        frames = _read_aux_frames(config_path)
        return {
            "ok":            True,
            "config_path":   config_path,
            "frames":        frames,
            "editable_keys": ["xyz", "rpy"],
            "message": ("changes apply live: "
                        "/robot_state_publisher refreshes TF / RViz / "
                        "the dashboard viewer immediately, and the FZI "
                        "cartesian controllers (forked) rebuild their "
                        "KDL chain at runtime via an on-set-parameters "
                        "callback, so engaged controllers pick up the "
                        "new tool TCP without unload/load"),
        }

    def api_set_aux_frames(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """Persist xyz / rpy edits to robot_config.yaml.

        Body shape:
          ``{"frames": [{"name": str, "xyz": [3 floats], "rpy": [3 floats]}, ...]}``

        Only entries whose ``name`` matches an existing aux_frame on
        disk are updated; unknown names are ignored (and the response
        reports the actual count).  Adding / renaming / removing is
        not supported via this endpoint.
        """
        if _save_aux_frames is None or _get_config is None:
            raise RuntimeError(
                "common.config_manager not importable: "
                f"{_COMMON_IMPORT_ERROR}")
        if not isinstance(body, dict):
            raise RuntimeError("body must be a JSON object")
        frames_in = body.get("frames")
        if not isinstance(frames_in, list):
            raise RuntimeError("body.frames must be a list")

        # Validate and normalise -> { name: { xyz: [...], rpy: [...] } }
        updates: Dict[str, Dict[str, List[float]]] = {}
        for idx, entry in enumerate(frames_in):
            if not isinstance(entry, dict):
                raise RuntimeError(
                    f"frames[{idx}] must be an object")
            name = entry.get("name")
            if not isinstance(name, str) or not name:
                raise RuntimeError(
                    f"frames[{idx}].name must be a non-empty string")
            triples: Dict[str, List[float]] = {}
            for key in ("xyz", "rpy"):
                if key not in entry:
                    continue
                vec = entry[key]
                if (not isinstance(vec, (list, tuple))
                        or len(vec) != 3):
                    raise RuntimeError(
                        f"frames[{idx}].{key} must be a length-3 list")
                try:
                    triples[key] = [float(v) for v in vec]
                except (TypeError, ValueError) as exc:
                    raise RuntimeError(
                        f"frames[{idx}].{key} not numeric: {exc}"
                    ) from exc
            if not triples:
                continue  # nothing to update for this entry
            updates[name] = triples

        if not updates:
            return {"ok": True, "updated": 0,
                    "message": "no editable fields supplied"}

        cfg = _get_config()
        config_path = cfg.config_path
        if not config_path:
            raise RuntimeError("config_manager has not resolved a path")

        try:
            updated = _save_aux_frames(config_path, updates)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"failed to save aux_frames: {exc}") from exc

        # ---- Live update: push the rebuilt URDF to rsp so /tf_static
        # refreshes without restarting duco_robot_bringup.  Best-effort:
        # any failure here is reported in the response but does not
        # roll back the yaml save (the file edit is the source of truth
        # for the next launch).
        live = self._apply_aux_frames_live()

        if live["ok"]:
            message = ("saved; static TF updated live via "
                       "robot_state_publisher")
            if live.get("missing"):
                message += (f" (URDF missing: "
                            f"{', '.join(live['missing'])} -- restart "
                            f"bringup to materialise them)")
            warnings = live.get("controller_warnings") or []
            if warnings:
                message += (". FZI controllers: "
                            + "; ".join(warnings))
            else:
                message += (". FZI cartesian controllers picked up "
                            "the new chain at runtime (no reload "
                            "needed).")
        else:
            message = (f"saved to yaml, but live URDF push failed: "
                       f"{live['error']}. Restart duco_robot_bringup "
                       f"to apply the change.")

        return {
            "ok":          True,
            "config_path": config_path,
            "updated":     updated,
            "live":        live,
            "message":     message,
        }

    # ------------------------------------------------------------------
    # Live URDF push to robot_state_publisher
    # ------------------------------------------------------------------
    def _apply_aux_frames_live(self) -> Dict[str, Any]:
        """Rebuild ``robot_description`` from the on-disk aux_frames and
        push it to ``/robot_state_publisher`` via SetParameters.

        Returns a dict ``{"ok": bool, "error": Optional[str], ...}``
        suitable to embed in the api_set_aux_frames response.  All
        failure modes (missing imports, no cached URDF, service
        unavailable, rsp rejection) come back as ``ok=False`` with a
        human-readable ``error`` so the operator can decide whether to
        fall back to a launch restart.
        """
        if _update_aux_frames is None or _read_aux_frames is None:
            return {
                "ok":    False,
                "error": ("duco_robot_bringup.urdf_loader not "
                          f"importable: {_URDF_LOADER_IMPORT_ERROR}"),
            }

        # Snapshot the current URDF the rsp is publishing.  We rebuild
        # from THIS string (rather than from xacro) because we do not
        # know the original xacro args (robot_ip, robot_port,
        # use_fake_hardware) at dashboard launch time and re-running
        # xacro with the wrong args would silently drift.
        with self._lock:
            urdf_xml = self._urdf_xml
        if not urdf_xml:
            return {
                "ok":    False,
                "error": ("no cached /robot_description yet -- is "
                          "robot_state_publisher running?"),
            }

        # Read the full aux_frames list back from disk (post-save) so
        # every entry's xyz/rpy is up-to-date.
        cfg = _get_config()
        config_path = cfg.config_path if cfg else None
        if not config_path:
            return {"ok": False, "error": "config path not resolved"}
        try:
            frames = _read_aux_frames(config_path)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False,
                    "error": f"could not re-read aux_frames: {exc}"}

        try:
            result = _update_aux_frames(urdf_xml, frames)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False,
                    "error": f"URDF rewrite failed: {exc}"}

        # Push the rewritten URDF to robot_state_publisher.
        if not self._cli_rsp_set_params.wait_for_service(timeout_sec=0.5):
            return {
                "ok":      False,
                "updated": result.updated,
                "missing": result.missing,
                "error":   ("/robot_state_publisher/set_parameters "
                            "service unavailable -- is rsp running and "
                            "reachable on this DDS domain?"),
            }
        param = Parameter()
        param.name = "robot_description"
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = result.urdf_xml
        param.value = pv
        req = SetParameters.Request()
        req.parameters = [param]
        resp = self._service_call_sync(
            self._cli_rsp_set_params, req, self._service_timeout)
        if resp is None:
            return {
                "ok":      False,
                "updated": result.updated,
                "missing": result.missing,
                "error":   ("no response from rsp set_parameters within "
                            f"{self._service_timeout:.1f}s"),
            }
        if not resp.results or not resp.results[0].successful:
            why = (resp.results[0].reason
                   if resp.results else "unknown error")
            return {
                "ok":      False,
                "updated": result.updated,
                "missing": result.missing,
                "error":   f"rsp rejected the new URDF: {why}",
            }

        # Also push to each FZI cartesian controller so the per-
        # controller on-set-parameters callback can rebuild the KDL
        # chain at runtime (see cartesian_controller_base.cpp::
        # onParameterUpdate in the forked submodule).  Each controller
        # owns its own node and its own ``robot_description`` parameter;
        # pushing to /controller_manager only updates the manager's
        # copy, which the running controllers do not observe.  Treated
        # as best-effort per controller: if a controller isn't loaded
        # the Save still succeeds (rsp got the new URDF) and we report
        # a per-controller warning so the operator can decide whether
        # to reload manually.
        cm_warnings: List[str] = []
        for ctrl in self._available_controllers:
            cli = self._param_clients.get(ctrl, {}).get("set")
            if cli is None:
                cm_warnings.append(
                    f"{ctrl}: no set_parameters client (not in "
                    "available_controllers?)")
                continue
            if not cli.wait_for_service(timeout_sec=0.3):
                cm_warnings.append(
                    f"{ctrl}: /{ctrl}/set_parameters unavailable -- "
                    "controller may not be loaded; reload it manually "
                    "to pick up the new chain")
                continue
            ctrl_resp = self._service_call_sync(
                cli, req, self._service_timeout)
            if ctrl_resp is None:
                cm_warnings.append(
                    f"{ctrl}: no response from set_parameters within "
                    f"{self._service_timeout:.1f}s -- old chain may "
                    "persist; reload if needed")
            elif (not ctrl_resp.results
                  or not ctrl_resp.results[0].successful):
                why = (ctrl_resp.results[0].reason
                       if ctrl_resp.results else "unknown error")
                cm_warnings.append(
                    f"{ctrl}: rejected the URDF push: {why} -- "
                    "controller will keep its old chain")

        out: Dict[str, Any] = {
            "ok":      True,
            "updated": result.updated,
            "missing": result.missing,
        }
        if cm_warnings:
            out["controller_warnings"] = cm_warnings
        return out

    # ------------------------------------------------------------------
    # 3D-model viewer support
    # ------------------------------------------------------------------
    # The dashboard's "3D model" panel renders a kinematic skeleton in
    # the browser.  It needs two things from the node:
    #
    # * ``/api/urdf_model`` -- the parsed URDF tree (links + joints
    #   with their static origins).  Fetched once on page load and
    #   whenever the user hits "Refresh".
    #
    # * ``/api/urdf_tf`` -- live world-frame poses of each link
    #   relative to ``root_link``.  Polled at a few Hz so the skeleton
    #   tracks live joint motion.  Returns ``null`` for any link whose
    #   transform isn't in /tf yet; the renderer falls back to the
    #   URDF static pose for those.
    def api_urdf_model(self) -> Dict[str, Any]:
        with self._lock:
            model = self._urdf_model
            age = (time.monotonic() - self._urdf_mono
                   if self._urdf_mono is not None else None)
            xml_len = len(self._urdf_xml) if self._urdf_xml else 0
        if model is None:
            return {
                "ok":      False,
                "message": ("waiting for /robot_description -- is "
                            "robot_state_publisher running?"),
                "age":     age,
            }
        return {
            "ok":        True,
            "age":       age,
            "xml_bytes": xml_len,
            "root_link": model["root_link"],
            "links":     list(model["links"]),
            "joints":    [dict(j) for j in model["joints"]],
        }

    def api_urdf_tf(self) -> Dict[str, Any]:
        """Look up every URDF link's transform relative to ``root_link``.

        Uses the existing tf2 buffer.  Links whose lookup fails are
        emitted as ``null`` so the browser can decide what to do (the
        renderer falls back to URDF static origins for missing
        transforms).
        """
        with self._lock:
            model = self._urdf_model
        if model is None:
            return {
                "ok":      False,
                "message": "no URDF cached yet",
            }
        root = model["root_link"]
        out: Dict[str, Any] = {}
        zero_time = rclpy.time.Time()
        timeout = rclpy.duration.Duration(seconds=0.05)
        for link in model["links"]:
            if link == root:
                out[link] = {"t": [0.0, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]}
                continue
            try:
                tf = self._tf_buffer.lookup_transform(
                    root, link, zero_time, timeout=timeout)
            except TransformException:
                out[link] = None
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            out[link] = {
                "t": [float(t.x), float(t.y), float(t.z)],
                "q": [float(q.x), float(q.y), float(q.z), float(q.w)],
            }
        return {
            "ok":         True,
            "root_link":  root,
            "transforms": out,
        }

    # ------------------------------------------------------------------
    def _try_lookup_tcp_pose(self) -> Optional[Dict[str, Any]]:
        """Best-effort TF lookup of ``base_frame -> tool_frame`` for the
        sticky-bar TCP-pose readout.  Returns ``None`` on any failure
        (frame missing, no TF yet) instead of raising -- this is a
        polled read, not a user action.

        The returned dict contains the cartesian xyz, the quaternion,
        roll/pitch/yaw expressed in degrees (ZYX-extrinsic convention,
        same as ``tf2`` ``getRPY``), and an estimated age of the TF
        sample (seconds since the TF stamp; may be ``None`` if the
        stamp is zero / unavailable).
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._tool_frame,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.0))
        except TransformException:
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        # RPY from quaternion (roll about X, pitch about Y, yaw about Z,
        # extrinsic).  Pitch is clamped at +/- pi/2 in the singular case.
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Age of the TF sample: now - stamp.  Stamp of zero means
        # "static" (or no stamp); report ``None`` then.
        age: Optional[float] = None
        try:
            stamp_sec = (int(t.header.stamp.sec)
                         + int(t.header.stamp.nanosec) * 1e-9)
            if stamp_sec > 0.0:
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                age = max(0.0, now_sec - stamp_sec)
        except Exception:  # noqa: BLE001
            age = None
        return {
            "frame_id":       t.header.frame_id,
            "child_frame_id": t.child_frame_id,
            "x":              float(tr.x),
            "y":              float(tr.y),
            "z":              float(tr.z),
            "qx":             float(q.x),
            "qy":             float(q.y),
            "qz":             float(q.z),
            "qw":             float(q.w),
            "roll_deg":       math.degrees(roll),
            "pitch_deg":      math.degrees(pitch),
            "yaw_deg":        math.degrees(yaw),
            "age":            age,
        }

    # ------------------------------------------------------------------
    def _snapshot_live_locked(self) -> Dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            ctl = dict(self._control_state) if self._control_state else {}
            ctl_age = (now - self._control_state_mono
                       if self._control_state_mono is not None else None)
            wrench: Optional[Dict[str, Any]] = None
            if self._wrench is not None and self._wrench_mono is not None:
                wrench = dict(self._wrench)
                wrench["age"] = now - self._wrench_mono
            js_age = (now - self._js_mono
                      if self._js_mono is not None else None)

        # Freshness flags: prefer the orchestrator's verdict when its
        # state is fresh; otherwise use our own thresholds (1.0 s).
        if ctl and ctl_age is not None and ctl_age <= 5.0:
            ft_ok = bool(ctl.get("ft_ok", False))
            joint_states_ok = bool(ctl.get("joint_states_ok", False))
        else:
            ft_ok = (wrench is not None
                     and wrench.get("age", 99.0) <= 1.0)
            joint_states_ok = (js_age is not None and js_age <= 1.0)

        # TF lookup is independent of the local cache lock -- tf2's
        # Buffer is thread-safe and this is a cheap latest-time read.
        tcp = self._try_lookup_tcp_pose()
        if tcp is not None:
            tcp["hz"] = self._tf_rate.hz()
        if wrench is not None:
            wrench["hz"] = self._wrench_rate.hz()

        return {
            "control":            ctl,
            "control_state_age":  ctl_age,
            "control_state_hz":   self._state_rate.hz(),
            "wrench":             wrench,
            "joint_states_age":   js_age,
            "joint_states_hz":    self._js_rate.hz(),
            "ft_ok":              ft_ok,
            "joint_states_ok":    joint_states_ok,
            "tcp":                tcp,
            "base_frame":         self._base_frame,
            "tool_frame":         self._tool_frame,
        }


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _parse_urdf(xml_text: str) -> Optional[Dict[str, Any]]:
    """Parse a URDF XML string into a minimal skeleton description.

    Returns a dict::

        {
          "root_link": "base_link",
          "links": ["base_link", "link_1", ...],
          "joints": [
            {"name": "joint_1", "type": "revolute",
             "parent": "base_link", "child": "link_1",
             "origin_xyz": [0, 0, 0.1], "origin_rpy": [0, 0, 0],
             "axis_xyz": [0, 0, 1]},
            ...
          ],
        }

    Returns ``None`` if the XML is empty or unparseable.  Mesh
    information is intentionally discarded -- the dashboard renders
    only the kinematic skeleton.
    """
    if not xml_text or not xml_text.strip():
        return None
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError:
        return None
    if root.tag != "robot":
        # Some URDFs are wrapped; try to find a <robot> element.
        robot = root.find(".//robot")
        if robot is None:
            return None
        root = robot

    links: List[str] = []
    parents: Dict[str, str] = {}  # child_link -> parent_link
    joints: List[Dict[str, Any]] = []

    for link in root.findall("link"):
        name = link.get("name")
        if name:
            links.append(name)

    def _floats(s: Optional[str], default: List[float]) -> List[float]:
        if not s:
            return list(default)
        try:
            parts = [float(x) for x in s.split()]
        except ValueError:
            return list(default)
        if len(parts) != len(default):
            return list(default)
        return parts

    for joint in root.findall("joint"):
        jname = joint.get("name") or ""
        jtype = joint.get("type") or "fixed"
        p_el = joint.find("parent")
        c_el = joint.find("child")
        if p_el is None or c_el is None:
            continue
        parent = p_el.get("link") or ""
        child = c_el.get("link") or ""
        if not parent or not child:
            continue
        origin = joint.find("origin")
        origin_xyz = _floats(origin.get("xyz") if origin is not None else None,
                             [0.0, 0.0, 0.0])
        origin_rpy = _floats(origin.get("rpy") if origin is not None else None,
                             [0.0, 0.0, 0.0])
        axis = joint.find("axis")
        axis_xyz = _floats(axis.get("xyz") if axis is not None else None,
                           [1.0, 0.0, 0.0])
        joints.append({
            "name":       jname,
            "type":       jtype,
            "parent":     parent,
            "child":      child,
            "origin_xyz": origin_xyz,
            "origin_rpy": origin_rpy,
            "axis_xyz":   axis_xyz,
        })
        parents[child] = parent

    # Root link is the only one that never appears as a child.
    root_link = ""
    for ln in links:
        if ln not in parents:
            root_link = ln
            break
    if not root_link and links:
        root_link = links[0]

    return {
        "root_link": root_link,
        "links":     links,
        "joints":    joints,
    }


def _decode_param_value(pv: ParameterValue) -> Optional[Any]:
    """Best-effort scalar decode of a :class:`ParameterValue`.

    Returns ``None`` when the parameter is not set on the remote node
    (``PARAMETER_NOT_SET``).
    """
    t = pv.type
    if t == ParameterType.PARAMETER_NOT_SET:
        return None
    if t == ParameterType.PARAMETER_BOOL:
        return bool(pv.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(pv.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(pv.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(pv.string_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        # rclpy hands us an ``array.array('d', ...)``; coerce to a plain
        # Python list so the value is JSON-serialisable for the HTTP layer.
        return [float(v) for v in pv.double_array_value]
    return None  # other array types not used by the dashboard


def _local_ip_hint() -> str:
    """Best-effort guess of the host's primary IPv4 for log output."""
    import socket as _s
    try:
        s = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        return "127.0.0.1"


def main(args=None) -> int:
    rclpy.init(args=args)
    node = DashboardNode()
    # MultiThreadedExecutor + ReentrantCallbackGroup so the synchronous
    # service calls issued from the HTTP handler thread (engage, set
    # parameters, etc.) can be dispatched on a separate executor
    # thread; a single-threaded executor would deadlock.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # rclpy's default SIGINT handler already shuts the context down
        # before the KeyboardInterrupt reaches us; calling shutdown()
        # again would raise 'rcl_shutdown already called'.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
