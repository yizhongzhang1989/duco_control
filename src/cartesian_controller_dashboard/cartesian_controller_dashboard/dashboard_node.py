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
import sys
import threading
import time
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import WrenchStamped
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
# embedded HTML / JS / CSS
# ---------------------------------------------------------------------------
_HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Cartesian controller dashboard</title>
<style>
  :root { color-scheme: light dark; }
  body { font-family: ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
         margin: 0; padding: 14px; background: #0f1115; color: #e6e6e6; }
  h1 { margin: 0 0 4px; font-size: 1.05rem; font-weight: 600; }
  h2 { margin: 14px 0 8px; font-size: 0.95rem; font-weight: 600; color: #cfd3dc; }
  .sub { color: #8a93a6; font-size: 0.82rem; margin-bottom: 12px; }
  code { background: #11141a; padding: 1px 5px; border-radius: 3px; }
  .row { display: flex; gap: 16px; flex-wrap: wrap; }
  .col { flex: 1 1 380px; min-width: 320px; }
  .panel { background: #181b22; border: 1px solid #262a33; border-radius: 8px;
           padding: 10px 12px 12px; margin-bottom: 12px; }
  table { width: 100%; border-collapse: collapse; font-size: 0.82rem; }
  th, td { text-align: left; padding: 4px 6px; border-bottom: 1px solid #232730;
           font-variant-numeric: tabular-nums; }
  th { color: #8a93a6; font-weight: 500; }
  button { background: #2563eb; color: white; border: 0; border-radius: 4px;
           padding: 4px 10px; font: inherit; cursor: pointer; font-size: 0.82rem; }
  button:hover { background: #1d4ed8; }
  button.danger  { background: #b91c1c; }
  button.danger:hover  { background: #991b1b; }
  button.success { background: #15803d; }
  button.success:hover { background: #166534; }
  button.big { padding: 8px 14px; font-size: 0.9rem; }
  button.ghost { background: #2b3140; }
  button.ghost:hover { background: #3a4150; }
  button:disabled { opacity: 0.5; cursor: not-allowed; }
  input[type="number"] { background: #0f1218; color: #e6e6e6;
           border: 1px solid #2b3140; border-radius: 4px; padding: 3px 6px;
           font: inherit; font-size: 0.82rem; width: 7em;
           font-variant-numeric: tabular-nums; }
  select { background: #0f1218; color: #e6e6e6;
           border: 1px solid #2b3140; border-radius: 4px; padding: 3px 6px;
           font: inherit; font-size: 0.82rem; }
  select:disabled { opacity: 0.5; cursor: not-allowed; }
  .status { display: inline-block; padding: 2px 7px; border-radius: 999px;
            background: #1f2937; color: #cfd3dc; font-variant-numeric: tabular-nums; }
  .status.ok  { background: #14532d; color: #c6f7d6; }
  .status.bad { background: #5a1d1d; color: #ffb4b4; }
  .status.warn { background: #553e1a; color: #ffe6a8; }
  .pill { display: inline-block; padding: 1px 6px; border-radius: 999px;
          background: #11141a; border: 1px solid #2b3140; font-size: 0.75rem;
          color: #cfd3dc; margin-right: 4px; }
  .pill.active { background: #14532d; color: #c6f7d6; border-color: #14532d; }
  .small { font-size: 0.78rem; color: #8a93a6; }
  .num { font-variant-numeric: tabular-nums; }
  .engage-bar { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
  .toast { position: fixed; right: 14px; bottom: 14px; max-width: 400px;
           background: #181b22; border: 1px solid #262a33; border-radius: 8px;
           padding: 8px 12px; font-size: 0.82rem; color: #cfd3dc;
           opacity: 0; transition: opacity .15s; pointer-events: none; }
  .toast.show { opacity: 1; }
  .toast.bad { border-color: #b91c1c; color: #ffb4b4; }
  .toast.ok  { border-color: #15803d; color: #c6f7d6; }
</style>
</head>
<body>
  <h1>Cartesian controller dashboard</h1>
  <div class="sub">
    active controller <code id="controller-name">--</code>
    &nbsp; orchestrator state <span id="orch-status" class="status">--</span>
    &nbsp; <span id="conn" class="status">connecting...</span>
  </div>

  <div class="panel">
    <div class="engage-bar">
      <span id="engaged" class="status">--</span>
      <span class="small">controller:</span>
      <select id="sel-controller" title="select which FZI controller engage will activate"></select>
      <span class="pill" id="kind-pill">kind: --</span>
      <span class="small">trip:</span>
      <span id="trip" class="status">--</span>
      <button id="btn-engage"    type="button" class="big success">Engage</button>
      <button id="btn-disengage" type="button" class="big danger">Disengage</button>
    </div>
    <div class="small" style="margin-top:6px;">
      Use the dropdown to select a controller; click Engage to activate
      it (only allowed while idle).  Force = pure free-drive
      (target_wrench=0); Motion = pose-hold (snapshots current pose);
      Compliance = pose-hold + yields to wrench.
    </div>
  </div>

  <div class="row">
    <div class="col">
      <div class="panel">
        <h2>Live wrench (sensor frame)</h2>
        <table>
          <thead><tr><th></th><th>force [N]</th><th>torque [Nm]</th></tr></thead>
          <tbody>
            <tr><td>x</td><td class="num" id="wfx">--</td><td class="num" id="wmx">--</td></tr>
            <tr><td>y</td><td class="num" id="wfy">--</td><td class="num" id="wmy">--</td></tr>
            <tr><td>z</td><td class="num" id="wfz">--</td><td class="num" id="wmz">--</td></tr>
          </tbody>
        </table>
        <div class="small" style="margin-top:8px;">
          |F| <span id="fmag" class="status">-- N</span>
          &nbsp; |T| <span id="tmag" class="status">-- Nm</span>
          &nbsp; ft <span id="ft-status" class="status">--</span>
          &nbsp; q <span id="q-status" class="status">--</span>
        </div>
      </div>

      <div class="panel">
        <h2>Safety thresholds (orchestrator)</h2>
        <table>
          <tbody>
            <tr><td>max |F|</td><td class="num" id="lim-f">--</td><td>N</td></tr>
            <tr><td>max |T|</td><td class="num" id="lim-t">--</td><td>Nm</td></tr>
            <tr><td>engage max |qdot|</td><td class="num" id="lim-qe">--</td><td>rad/s</td></tr>
            <tr><td>FT stale after</td><td class="num" id="lim-ftstale">--</td><td>s</td></tr>
            <tr><td>joint_states stale after</td><td class="num" id="lim-qstale">--</td><td>s</td></tr>
          </tbody>
        </table>
        <div class="small" style="margin-top:8px;">
          These are read-only here; tune them via parameters on
          <code>/duco_cartesian_control</code>.
        </div>
      </div>
    </div>

    <div class="col">
      <div class="panel">
        <h2>Controller parameters</h2>
        <div class="small" style="margin-bottom:6px;">
          Live tuning of <code id="param-target">--</code>.  Changes apply
          immediately to the running controller.
        </div>
        <table id="param-table">
          <thead>
            <tr><th>name</th><th>current</th><th>new</th><th></th></tr>
          </thead>
          <tbody id="param-body"></tbody>
        </table>
        <div style="margin-top:8px; display:flex; gap:8px;">
          <button id="btn-refresh-params" class="ghost">Refresh</button>
        </div>
      </div>
    </div>
  </div>

  <div id="toast" class="toast"></div>

<script>
(function () {
  const $  = (id) => document.getElementById(id);
  const fmt = (v, d) => (v == null || isNaN(v)) ? "--" : Number(v).toFixed(d ?? 2);
  const fmtAge = (a) => a == null ? "--" : (a < 1 ? (a*1000).toFixed(0) + " ms" : a.toFixed(2) + " s");

  let snapshot = null;

  async function api(path, opts) {
    const r = await fetch(path, opts || {});
    let data = {};
    try { data = await r.json(); } catch (_) { }
    if (!r.ok) throw new Error(data.error || (r.status + " " + r.statusText));
    return data;
  }

  function toast(msg, kind) {
    const el = $("toast");
    el.textContent = msg;
    el.classList.remove("ok", "bad", "show");
    if (kind) el.classList.add(kind);
    el.classList.add("show");
    clearTimeout(toast._t);
    toast._t = setTimeout(() => el.classList.remove("show"), 3000);
  }

  function setEngagedPill(s) {
    const el = $("engaged");
    el.classList.remove("ok", "bad", "warn");
    if (s.trip_reason) {
      el.textContent = "TRIPPED";
      el.classList.add("bad");
    } else if (s.engaged) {
      el.textContent = "ENGAGED";
      el.classList.add("ok");
    } else {
      el.textContent = "idle";
      el.classList.add("warn");
    }
    $("trip").textContent = s.trip_reason || "(none)";
    $("trip").classList.toggle("bad", !!s.trip_reason);
  }

  // Cache of last (active_controller, available_controllers) we rendered
  // into the <select> so we don't blow away an in-progress selection on
  // every 200 ms tick.
  let renderedSelectSig = null;

  function setControllerSelect(ctl) {
    const sel = $("sel-controller");
    const avail = ctl.available_controllers || [];
    const sig = JSON.stringify(avail);
    if (sig !== renderedSelectSig) {
      sel.innerHTML = "";
      for (const c of avail) {
        const opt = document.createElement("option");
        opt.value = c.name;
        opt.textContent = c.name + " (" + c.kind + ")";
        sel.appendChild(opt);
      }
      renderedSelectSig = sig;
    }
    // Sync select to current active unless the user is editing the
    // dropdown right now (handled by document.activeElement guard).
    if (document.activeElement !== sel) {
      sel.value = ctl.active_controller || "";
    }
    // Lock the dropdown while engaged -- the orchestrator refuses the
    // param change anyway, but disabling makes intent obvious.
    sel.disabled = !!ctl.engaged;
    // Show kind pill for whatever is currently *selected* in the
    // dropdown, or the active one if no selection.
    const chosen = sel.value || ctl.active_controller || "";
    const entry = avail.find((c) => c.name === chosen);
    $("kind-pill").textContent = "kind: " + (entry ? entry.kind : "--");
  }

  function setStaleness(elId, ok, age) {
    const el = $(elId);
    el.classList.remove("ok", "bad", "warn");
    if (age == null) {
      el.textContent = "no data";
      el.classList.add("warn");
    } else {
      el.textContent = fmtAge(age);
      el.classList.add(ok ? "ok" : "bad");
    }
  }

  function setOrchPill(s) {
    const el = $("orch-status");
    el.classList.remove("ok", "bad", "warn");
    if (!s || !s.control_state_age || s.control_state_age == null) {
      el.textContent = "no orchestrator";
      el.classList.add("bad");
      return;
    }
    if (s.control_state_age > 5.0) {
      el.textContent = "stale (" + fmtAge(s.control_state_age) + ")";
      el.classList.add("warn");
    } else {
      el.textContent = "live (" + fmtAge(s.control_state_age) + ")";
      el.classList.add("ok");
    }
  }

  function applyLive(s) {
    if (!s) return;
    const ctl = s.control || {};
    setEngagedPill(ctl);
    setControllerSelect(ctl);
    setOrchPill(s);

    const w = s.wrench;
    $("wfx").textContent = w ? fmt(w.fx) : "--";
    $("wfy").textContent = w ? fmt(w.fy) : "--";
    $("wfz").textContent = w ? fmt(w.fz) : "--";
    $("wmx").textContent = w ? fmt(w.tx, 3) : "--";
    $("wmy").textContent = w ? fmt(w.ty, 3) : "--";
    $("wmz").textContent = w ? fmt(w.tz, 3) : "--";
    if (w) {
      const fm = Math.hypot(w.fx, w.fy, w.fz);
      const tm = Math.hypot(w.tx, w.ty, w.tz);
      $("fmag").textContent = fmt(fm) + " N";
      $("tmag").textContent = fmt(tm, 3) + " Nm";
    } else {
      $("fmag").textContent = "-- N";
      $("tmag").textContent = "-- Nm";
    }

    setStaleness("ft-status", !!s.ft_ok, w ? w.age : null);
    setStaleness("q-status",  !!s.joint_states_ok, s.joint_states_age);

    $("conn").textContent = "live";
    $("conn").classList.remove("warn", "bad");
    $("conn").classList.add("ok");

    // Engage button enabled only if the orchestrator is alive AND not engaged.
    const orchAlive = (s.control_state_age != null && s.control_state_age <= 5.0);
    $("btn-engage").disabled    = ctl.engaged || !orchAlive;
    $("btn-disengage").disabled = !ctl.engaged || !orchAlive;
  }

  // Track which parameter names we've already rendered so we can avoid
  // rebuilding the rows on every refresh (which would clobber whatever
  // the operator is typing into the "new" <input>).
  let renderedParamSig = null;

  function buildParamRows(params) {
    const body = $("param-body");
    body.innerHTML = "";
    renderedParamSig = null;
    if (!params || params.length === 0) {
      body.innerHTML = '<tr><td colspan="4" class="small">no parameters available (controller not running?)</td></tr>';
      return;
    }
    for (const p of params) {
      const tr = document.createElement("tr");
      const td1 = document.createElement("td");
      td1.innerHTML = '<code>' + p.name + '</code>';
      const td2 = document.createElement("td");
      td2.className = "num";
      td2.id = "cur-" + p.name;
      td2.textContent = (p.value == null) ? "--" :
        (p.kind === "integer") ? String(p.value) : fmt(p.value, 4);
      const td3 = document.createElement("td");
      const inp = document.createElement("input");
      inp.type = "number";
      inp.step = (p.kind === "integer") ? "1" : "0.001";
      inp.value = (p.value == null) ? "" : p.value;
      inp.dataset.name = p.name;
      inp.dataset.kind = p.kind;
      // Submit on Enter so users don't have to reach for the mouse.
      inp.addEventListener("keydown", (ev) => {
        if (ev.key === "Enter") {
          ev.preventDefault();
          setParam(p.name, p.kind, inp.value);
        }
      });
      td3.appendChild(inp);
      const td4 = document.createElement("td");
      const btn = document.createElement("button");
      btn.textContent = "Set";
      btn.addEventListener("click", () => setParam(p.name, p.kind, inp.value));
      td4.appendChild(btn);
      tr.append(td1, td2, td3, td4);
      body.appendChild(tr);
    }
    renderedParamSig = params.map((p) => p.name + ":" + p.kind).join("|");
  }

  function syncParamRows(params) {
    // Re-build only if the list of (name, kind) tuples changed; otherwise
    // just update the "current" cells via applyParams() so we don't wipe
    // out whatever the operator is typing into a "new" <input>.
    const sig = (params || []).map((p) => p.name + ":" + p.kind).join("|");
    if (sig !== renderedParamSig) {
      buildParamRows(params || []);
    } else {
      applyParams(params);
    }
  }

  function applyParams(params) {
    if (!params) return;
    for (const p of params) {
      const el = document.getElementById("cur-" + p.name);
      if (!el) continue;
      el.textContent = (p.value == null) ? "--" :
        (p.kind === "integer") ? String(p.value) : fmt(p.value, 4);
    }
  }

  async function setParam(name, kind, raw) {
    if (raw === "" || raw === null) {
      toast("enter a value first", "bad"); return;
    }
    const value = (kind === "integer") ? parseInt(raw, 10) : parseFloat(raw);
    if (isNaN(value)) { toast("not a number", "bad"); return; }
    try {
      const r = await api("/api/param", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, kind, value })
      });
      toast("set " + name + " = " + value, "ok");
      // Re-fetch params to reflect what the controller actually applied.
      await refreshParams();
    } catch (e) {
      toast("set " + name + " failed: " + e.message, "bad");
    }
  }

  async function refresh() {
    try {
      snapshot = await api("/api/state");
      $("controller-name").textContent = snapshot.controller_name || "--";
      $("param-target").textContent    = snapshot.controller_name || "--";
      const ctl = snapshot.control || {};
      const lim = ctl.limits || {};
      $("lim-f").textContent       = fmt(lim.max_wrench_force);
      $("lim-t").textContent       = fmt(lim.max_wrench_torque, 3);
      $("lim-qe").textContent      = fmt(lim.engage_max_joint_velocity, 3);
      $("lim-ftstale").textContent = fmt(lim.ft_stale_after, 2);
      $("lim-qstale").textContent  = fmt(lim.joint_states_stale_after, 2);
      syncParamRows(snapshot.params || []);
      applyLive(snapshot);
    } catch (e) {
      $("conn").textContent = "offline";
      $("conn").classList.remove("ok");
      $("conn").classList.add("bad");
    }
  }

  async function tick() {
    try {
      const s = await api("/api/live");
      applyLive(s);
    } catch (e) {
      $("conn").textContent = "offline";
      $("conn").classList.remove("ok");
      $("conn").classList.add("bad");
    }
  }

  async function refreshParams() {
    try {
      const r = await api("/api/params");
      applyParams(r.params || []);
    } catch (e) {
      toast("refresh params failed: " + e.message, "bad");
    }
  }

  $("btn-engage").addEventListener("click", async () => {
    try { await api("/api/engage", { method: "POST" }); await refresh(); }
    catch (e) { toast("engage failed: " + e.message, "bad"); }
  });

  $("btn-disengage").addEventListener("click", async () => {
    try { await api("/api/disengage", { method: "POST" }); await refresh(); }
    catch (e) { toast("disengage failed: " + e.message, "bad"); }
  });

  $("btn-refresh-params").addEventListener("click", refreshParams);

  // Switch the orchestrator's ``active_controller_name`` parameter on
  // dropdown change.  The orchestrator refuses if currently engaged --
  // surface that as a toast and snap the dropdown back.
  $("sel-controller").addEventListener("change", async (ev) => {
    const name = ev.target.value;
    if (!name) return;
    try {
      await api("/api/active_controller", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name })
      });
      toast("active controller -> " + name, "ok");
      await refresh();
    } catch (e) {
      toast("switch failed: " + e.message, "bad");
      // Re-sync to whatever the orchestrator actually thinks is active.
      await refresh();
    }
  });

  refresh().then(() => setInterval(tick, 200));
  setInterval(refresh, 3000);
})();
</script>
</body>
</html>
"""


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

    def _send_html(self, body: bytes) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
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

    # ---- routes -----------------------------------------------------------
    def do_GET(self):  # noqa: N802
        path = urlparse(self.path).path
        try:
            if path in ("/", "/index.html"):
                self._send_html(_HTML_PAGE.encode("utf-8"))
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
