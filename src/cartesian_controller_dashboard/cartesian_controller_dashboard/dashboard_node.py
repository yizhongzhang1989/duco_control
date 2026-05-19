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
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
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

    def _on_joint_states(self, msg: JointState) -> None:  # noqa: ARG002
        with self._lock:
            self._js_mono = time.monotonic()

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

        return {
            "control":            ctl,
            "control_state_age":  ctl_age,
            "wrench":             wrench,
            "joint_states_age":   js_age,
            "ft_ok":              ft_ok,
            "joint_states_ok":    joint_states_ok,
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
    return None  # arrays not supported by the dashboard


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
