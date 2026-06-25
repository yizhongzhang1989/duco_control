#!/usr/bin/env python3
"""Tiny web dashboard for the ``spacemouse_servo`` teleop bridge.

Shows the live servo status and exposes ONE control: switch the SpaceMouse
teleop SENDER on/off (calls ``<servo_ns>/set_enabled``). OFF stops the servo
from streaming ``target_pose`` and releases the pose commander, so another
source (e.g. the pose-commander dashboard) can drive; ON resumes jogging and
re-enables the commander.

Launched by ``spacemouse_servo.launch.py`` / ``spacemouse_teleop.launch.py``
when a ``dashboard_port`` argument is provided.
"""
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool

_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>SpaceMouse Teleop</title>
<style>
  :root { --bg:#0d1117; --card:#161b22; --bd:#30363d; --fg:#e6edf3; --mut:#8b949e;
          --on:#2ea043; --off:#da3633; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:14px/1.5 system-ui,Segoe UI,Roboto,sans-serif; }
  .wrap { max-width:520px; margin:0 auto; padding:24px 16px; }
  h1 { font-size:18px; margin:0 0 2px; display:flex; align-items:center; gap:10px; }
  .sub { color:var(--mut); font-size:12px; margin-bottom:18px; }
  .card { background:var(--card); border:1px solid var(--bd); border-radius:10px;
          padding:18px; margin-bottom:14px; }
  .state { display:flex; align-items:center; gap:12px; }
  .dot { width:14px; height:14px; border-radius:50%; background:var(--mut); }
  .dot.on { background:var(--on); box-shadow:0 0 10px var(--on); }
  .dot.off { background:var(--off); }
  .big { font-size:22px; font-weight:700; letter-spacing:.5px; }
  .btns { display:flex; gap:10px; margin-top:16px; }
  button { flex:1; padding:14px; border-radius:8px; border:1px solid var(--bd);
           font-size:15px; font-weight:600; cursor:pointer; color:var(--fg);
           background:#21262d; }
  button.on { background:var(--on); border-color:var(--on); }
  button.off { background:var(--off); border-color:var(--off); }
  button:disabled { opacity:.45; cursor:default; }
  .grid { display:grid; grid-template-columns:auto 1fr; gap:6px 14px;
          font-size:13px; }
  .grid .k { color:var(--mut); }
  .pill { font-size:11px; padding:2px 8px; border-radius:20px; border:1px solid var(--bd);
          color:var(--mut); font-weight:600; }
  .pill.ok { color:var(--on); border-color:var(--on); }
  .pill.bad { color:var(--off); border-color:var(--off); }
  .msg { color:var(--mut); font-size:12px; min-height:16px; margin-top:10px; }
  /* device viz (axes + 3D preview) */
  .dev-head { display:flex; align-items:center; gap:10px; font-weight:600;
              font-size:15px; margin-bottom:4px; }
  .dev-note { color:var(--mut); font-size:11px; margin-bottom:12px; }
  .dev-grid { display:grid; grid-template-columns:190px 1fr; gap:16px;
              align-items:center; }
  @media (max-width:540px){ .dev-grid{ grid-template-columns:1fr; } }
  .scene { height:190px; perspective:700px; display:flex; align-items:center;
           justify-content:center; }
  /* camera: true right-handed Z-up view. This matrix replicates the 3dconnexion
     reference camera (Z up, +Y left, +X into the screen). Its determinant is -1,
     which is REQUIRED: CSS's basis (x-right, y-down, z-out) is left-handed, so a
     right-handed world must be reflected into it for the frame to read right-handed.
     (A plain rotateX/rotateZ can never fix handedness -- rotations are det +1.) */
  .view { transform-style:preserve-3d;
          transform:matrix3d(0.2402,-0.4746,-0.8468,0, -0.9707,-0.1174,-0.2095,0,
                             0,-0.8723,0.4889,0, 0,0,0,1); }
  .cube { position:relative; width:84px; height:84px; transform-style:preserve-3d;
          cursor:pointer; }
  .face { position:absolute; width:84px; height:84px;
          border:1px solid rgba(255,255,255,.4); display:flex; align-items:center;
          justify-content:center; font-weight:700; font-size:13px;
          color:#0d1117; opacity:.55; }
  .f-px{ background:#e5534b; transform:rotateY(90deg) translateZ(42px); }
  .f-nx{ background:#9e3d37; transform:rotateY(-90deg) translateZ(42px); }
  .f-py{ background:#3fb950; transform:rotateX(-90deg) translateZ(42px); }
  .f-ny{ background:#2c8038; transform:rotateX(90deg) translateZ(42px); }
  .f-pz{ background:#2f81f7; transform:translateZ(42px); }
  .f-nz{ background:#2059b0; transform:rotateY(180deg) translateZ(42px); }
  /* FIXED reference frame (does NOT rotate) -- align the SpaceMouse base to it */
  .refframe { position:absolute; left:50%; top:50%; width:0; height:0;
              transform-style:preserve-3d; pointer-events:none; }
  .rf { position:absolute; left:0; top:-1px; height:2px; width:96px;
        transform-origin:0 50%; }
  .rf::after { content:''; position:absolute; right:-1px; top:50%; width:0; height:0;
               transform:translateY(-50%); border-top:5px solid transparent;
               border-bottom:5px solid transparent; border-left:9px solid currentColor; }
  .rf.x { color:#ff6b61;
          background:repeating-linear-gradient(90deg,#ff6b61 0 7px,transparent 7px 12px);
          transform:rotateY(0deg); }
  .rf.y { color:#56d364;
          background:repeating-linear-gradient(90deg,#56d364 0 7px,transparent 7px 12px);
          transform:rotateZ(90deg); }
  .rf.z { color:#58a6ff;
          background:repeating-linear-gradient(90deg,#58a6ff 0 7px,transparent 7px 12px);
          transform:rotateY(-90deg); }
  .triad-legend { font-size:11px; color:var(--mut); text-align:center;
                  margin-top:6px; }
  .axes { display:flex; flex-direction:column; gap:7px; }
  .arow { display:flex; align-items:center; gap:8px; }
  .alabel { width:42px; font-size:11px; color:var(--mut); text-align:right; }
  .atrack { flex:1; height:14px; background:#21262d; border-radius:4px;
            position:relative; overflow:hidden; }
  .acenter { position:absolute; left:50%; top:0; bottom:0; width:1px;
             background:var(--bd); }
  .afill { position:absolute; top:2px; bottom:2px; border-radius:3px;
           transition:left .05s, width .05s; }
  .aval { width:46px; font-family:monospace; font-size:11px; }
  .btns-dev { display:flex; gap:6px; flex-wrap:wrap; margin-top:14px; }
  .bdot { min-width:30px; height:24px; padding:0 6px; border-radius:6px;
          background:#21262d; border:1px solid var(--bd); color:var(--mut);
          font-family:monospace; font-size:11px; line-height:24px;
          text-align:center; }
  .bdot.on { background:var(--on); color:#0d1117; border-color:var(--on); }
  .dev-warn { font-size:11px; color:#e3b341; margin-top:8px; text-align:center; }
</style>
</head>
<body>
<div class="wrap">
  <h1>SpaceMouse Teleop <span id="conn" class="pill bad">connecting…</span></h1>
  <div class="sub">Switch the SpaceMouse sender on/off. <b>Off</b> stops streaming the
    target and releases the commander, so another source (e.g. the pose-commander
    dashboard) can drive. <b>On</b> resumes jogging.</div>

  <div class="card">
    <div class="state">
      <span id="dot" class="dot"></span>
      <span class="big" id="senderState">—</span>
    </div>
    <div class="btns">
      <button id="btnOn" class="on" onclick="setEnabled(true)">Turn ON</button>
      <button id="btnOff" class="off" onclick="setEnabled(false)">Turn OFF</button>
    </div>
    <div class="msg" id="msg"></div>
  </div>

  <div class="card">
    <div class="grid">
      <div class="k">engaged</div><div id="engaged">—</div>
      <div class="k">jog frame</div><div id="jog">—</div>
      <div class="k">speed scale</div><div id="speed">—</div>
      <div class="k">base &rarr; tip</div><div id="frames">—</div>
      <div class="k">input</div><div id="fresh">—</div>
    </div>
  </div>

  <div class="card">
    <div class="dev-head">SpaceMouse device
      <span id="devhz" class="pill bad">— Hz</span></div>
    <div class="dev-note">The dashed frame is the SpaceMouse base / operating frame
      (Z&nbsp;up, right-handed). The cube starts aligned with it &mdash; push or twist
      the puck to translate and rotate the cube within this frame. Click the cube to
      recentre.</div>
    <div class="dev-grid">
      <div>
        <div class="scene"><div class="view">
          <div class="refframe"><div class="rf x"></div><div class="rf y"></div>
            <div class="rf z"></div></div>
          <div class="cube" id="cube">
            <div class="face f-px"></div><div class="face f-nx"></div>
            <div class="face f-py"></div><div class="face f-ny"></div>
            <div class="face f-pz"></div><div class="face f-nz"></div>
          </div>
        </div></div>
        <div class="triad-legend"><b style="color:#e5534b">X</b>
          <b style="color:#3fb950">Y</b> <b style="color:#2f81f7">Z</b> &mdash; dashed
          = SpaceMouse operating frame; the cube moves &amp; rotates within it</div>
      </div>
      <div class="axes" id="axisGroup"></div>
    </div>
    <div class="btns-dev" id="btnGrid"></div>
    <div class="dev-warn">&#9888; Avoid pressing 3+ buttons at once &mdash; the
      device may report a phantom button (matrix ghosting).</div>
  </div>
</div>
<script>
const $ = (id) => document.getElementById(id);
async function poll() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();
    const connected = !!d.connected;
    const s = d.status || {};
    $('conn').textContent = connected ? 'connected' : 'no servo';
    $('conn').className = 'pill ' + (connected ? 'ok' : 'bad');
    const on = !!s.sender_enabled;
    $('senderState').textContent = connected ? (on ? 'SENDER ON' : 'SENDER OFF') : '—';
    $('dot').className = 'dot ' + (!connected ? '' : (on ? 'on' : 'off'));
    $('btnOn').disabled = !connected || on;
    $('btnOff').disabled = !connected || !on;
    const eng = s.engaged;
    $('engaged').textContent = eng === undefined ? '—' : (eng ? 'yes (jogging)' : 'no');
    $('jog').textContent = s.jog_frame || '—';
    const sc = s.speed_scale;
    $('speed').textContent = sc !== undefined ? (sc + '\u00d7') : '—';
    $('frames').textContent = (s.base_frame && s.tip_frame)
      ? (s.base_frame + ' \u2192 ' + s.tip_frame) : '—';
    $('fresh').textContent = s.twist_fresh === undefined
      ? '—' : (s.twist_fresh ? 'moving' : 'idle');
  } catch (e) {
    $('conn').textContent = 'dashboard error';
    $('conn').className = 'pill bad';
  }
}
async function setEnabled(v) {
  $('msg').textContent = 'sending\u2026';
  try {
    const r = await fetch('/api/set', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({enabled: v})
    });
    const d = await r.json();
    $('msg').textContent = d.message || (d.ok ? 'ok' : 'failed');
  } catch (e) {
    $('msg').textContent = 'request failed: ' + e;
  }
  poll();
}
poll();
setInterval(poll, 400);

// ---- SpaceMouse device status: axis bars + 3D preview ----
window.dev = null;
async function pollDev() {
  try { const r = await fetch('/api/device'); window.dev = await r.json(); }
  catch (e) { /* keep last sample */ }
}
setInterval(pollDev, 40); pollDev();

const AX = [['lx', 'Lin X', '#e5534b'], ['ly', 'Lin Y', '#3fb950'],
            ['lz', 'Lin Z', '#2f81f7'], ['ax', 'Ang X', '#e5534b'],
            ['ay', 'Ang Y', '#3fb950'], ['az', 'Ang Z', '#2f81f7']];
const axEls = {};
AX.forEach(([k, l, c]) => {
  const row = document.createElement('div'); row.className = 'arow';
  row.innerHTML = '<span class="alabel">' + l + '</span>'
    + '<div class="atrack"><div class="acenter"></div>'
    + '<div class="afill" id="bar_' + k + '" style="background:' + c + '"></div></div>'
    + '<span class="aval" id="val_' + k + '">0.000</span>';
  $('axisGroup').appendChild(row);
  axEls[k] = {bar: row.querySelector('#bar_' + k), val: row.querySelector('#val_' + k)};
});
function updAxes(t) {
  for (const a of AX) {
    const k = a[0], v = +(t[k] || 0), pct = Math.min(Math.abs(v), 1) * 50;
    const b = axEls[k].bar;
    b.style.left = v >= 0 ? '50%' : (50 - pct) + '%';
    b.style.width = pct + '%';
    axEls[k].val.textContent = v.toFixed(3);
  }
}
// SpaceMouse Pro button names (matches the 3dconnexion_ros2 buttons[] mapping)
const SMP_NAMES = ['1', '2', '3', '4', 'Menu', 'Fit', 'T', 'R', 'F', 'Roll',
                   '\u27F3', 'Esc', 'Alt', 'Shift', 'Ctrl'];
function updBtns(btns) {
  const g = $('btnGrid');
  while (g.children.length < SMP_NAMES.length) {
    const i = g.children.length;
    const d = document.createElement('div'); d.className = 'bdot';
    d.textContent = SMP_NAMES[i];
    d.title = SMP_NAMES[i] + ' \u2014 buttons[' + i + ']';
    g.appendChild(d);
  }
  for (let i = 0; i < g.children.length; i++)
    g.children[i].classList.toggle('on', btns[i] === 1);
}

const cube = $('cube');
let q = {x: 0, y: 0, z: 0, w: 1}, px = 0, py = 0, pz = 0;
const ROT = 2.5, POS = 70, DECAY = 0.9, DT = 1 / 30;
function qmul(a, b) {
  return {
    w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w};
}
function dq(wx, wy, wz) {
  const a = Math.hypot(wx, wy, wz);
  if (a < 1e-9) return {x: 0, y: 0, z: 0, w: 1};
  const s = Math.sin(a / 2) / a;
  return {x: wx*s, y: wy*s, z: wz*s, w: Math.cos(a / 2)};
}
function mat(r, tx, ty, tz) {
  const x = r.x, y = r.y, z = r.z, w = r.w, x2 = x+x, y2 = y+y, z2 = z+z;
  const xx = x*x2, xy = x*y2, xz = x*z2, yy = y*y2, yz = y*z2, zz = z*z2,
        wx = w*x2, wy = w*y2, wz = w*z2;
  return 'matrix3d(' + [1-(yy+zz), xy+wz, xz-wy, 0,
                        xy-wz, 1-(xx+zz), yz+wx, 0,
                        xz+wy, yz-wx, 1-(xx+yy), 0, tx, ty, tz, 1].join(',') + ')';
}
cube.addEventListener('click', () => { q = {x: 0, y: 0, z: 0, w: 1}; px = py = pz = 0; });
(function spin() {
  requestAnimationFrame(spin);
  const d = window.dev, t = (d && d.twist) || {lx: 0, ly: 0, lz: 0, ax: 0, ay: 0, az: 0};
  q = qmul(dq((t.ax||0)*ROT*DT, (t.ay||0)*ROT*DT, (t.az||0)*ROT*DT), q);
  const n = Math.hypot(q.x, q.y, q.z, q.w) || 1;
  q = {x: q.x/n, y: q.y/n, z: q.z/n, w: q.w/n};
  px = (px + (t.lx||0)*POS*DT) * DECAY;
  py = (py + (t.ly||0)*POS*DT) * DECAY;
  pz = (pz + (t.lz||0)*POS*DT) * DECAY;
  cube.style.transform = mat(q, px, py, pz);
  if (d) {
    updAxes(t); updBtns(d.buttons || []);
    const hz = $('devhz');
    hz.textContent = (d.hz != null ? d.hz : '—') + ' Hz';
    hz.className = 'pill ' + (d.age != null && d.age < 1.0 ? 'ok' : 'bad');
  }
})();
</script>
</body>
</html>
"""


class _HzTracker:
    """Rolling 1-second message-rate estimate (thread-safe)."""

    def __init__(self):
        self._count = 0
        self._hz = 0.0
        self._t0 = time.monotonic()
        self._lock = threading.Lock()

    def tick(self):
        with self._lock:
            self._count += 1
            now = time.monotonic()
            dt = now - self._t0
            if dt >= 1.0:
                self._hz = self._count / dt
                self._count = 0
                self._t0 = now

    @property
    def hz(self):
        with self._lock:
            return round(self._hz, 1)


class ServoDashboard(Node):
    def __init__(self):
        super().__init__("spacemouse_servo_dashboard")
        self.declare_parameter("port", 8200)
        self.declare_parameter("servo_ns", "/spacemouse_servo")
        self._port = int(self.get_parameter("port").value)
        ns = str(self.get_parameter("servo_ns").value).strip().rstrip("/")
        if ns and not ns.startswith("/"):
            ns = "/" + ns
        self._status_topic = (ns or "/spacemouse_servo") + "/status"
        self._set_srv_name = (ns or "/spacemouse_servo") + "/set_enabled"

        self._lock = threading.Lock()
        self._status = {}
        self._stamp = 0.0

        self.create_subscription(String, self._status_topic, self._on_status, 10)
        self._set_cli = self.create_client(SetBool, self._set_srv_name)

        # Raw SpaceMouse device feed (same topics as the 3dconnexion dashboard)
        # so the page can show axis bars + a 3D preview of the live input.
        self.declare_parameter("twist_topic", "spacenav/twist")
        self.declare_parameter("joy_topic", "spacenav/joy")
        twist_topic = str(self.get_parameter("twist_topic").value)
        joy_topic = str(self.get_parameter("joy_topic").value)
        self._device = {
            "twist": {"lx": 0.0, "ly": 0.0, "lz": 0.0,
                      "ax": 0.0, "ay": 0.0, "az": 0.0},
            "buttons": [], "axes": [],
        }
        self._dev_stamp = 0.0
        self._hz_twist = _HzTracker()
        self.create_subscription(Twist, twist_topic, self._on_twist, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)

        self._start_http()
        self.get_logger().info(
            "spacemouse teleop dashboard on http://0.0.0.0:%d  "
            "(status=%s, set_enabled=%s)"
            % (self._port, self._status_topic, self._set_srv_name))

    # ---- ROS ---------------------------------------------------------------
    def _on_status(self, msg: String) -> None:
        try:
            s = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self._status = s
            self._stamp = time.monotonic()

    def _snapshot(self):
        with self._lock:
            return dict(self._status), (time.monotonic() - self._stamp
                                        if self._stamp else 1e9)

    def _on_twist(self, msg: Twist) -> None:
        self._hz_twist.tick()
        with self._lock:
            self._device["twist"] = {
                "lx": msg.linear.x, "ly": msg.linear.y, "lz": msg.linear.z,
                "ax": msg.angular.x, "ay": msg.angular.y, "az": msg.angular.z,
            }
            self._dev_stamp = time.monotonic()

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._device["buttons"] = list(msg.buttons)
            self._device["axes"] = list(msg.axes)
            self._dev_stamp = time.monotonic()

    def _device_snapshot(self):
        with self._lock:
            d = {"twist": dict(self._device["twist"]),
                 "buttons": list(self._device["buttons"]),
                 "axes": list(self._device["axes"])}
        d["hz"] = self._hz_twist.hz
        d["age"] = (time.monotonic() - self._dev_stamp
                    if self._dev_stamp else 1e9)
        return d

    def _set_enabled(self, enabled: bool):
        if not self._set_cli.service_is_ready():
            self._set_cli.wait_for_service(timeout_sec=1.0)
        if not self._set_cli.service_is_ready():
            return False, "servo set_enabled service unavailable (%s)" % self._set_srv_name
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self._set_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 2.0:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        if res is None:
            return False, "no response from servo"
        return bool(res.success), res.message

    # ---- HTTP --------------------------------------------------------------
    def _start_http(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send(self, code, body, ctype="application/json"):
                data = body.encode() if isinstance(body, str) else body
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                path = self.path.split("?")[0]
                if path in ("/", "/index.html"):
                    self._send(200, _PAGE, "text/html; charset=utf-8")
                elif path == "/api/status":
                    s, age = node._snapshot()
                    self._send(200, json.dumps({
                        "ok": True,
                        "connected": age < 1.5 and bool(s),
                        "service_ready": node._set_cli.service_is_ready(),
                        "status": s,
                    }))
                elif path == "/api/device":
                    self._send(200, json.dumps(node._device_snapshot()))
                else:
                    self._send(404, json.dumps({"ok": False, "message": "not found"}))

            def do_POST(self):
                if self.path.split("?")[0] != "/api/set":
                    self._send(404, json.dumps({"ok": False, "message": "not found"}))
                    return
                try:
                    n = int(self.headers.get("Content-Length", 0) or 0)
                    body = json.loads(self.rfile.read(n) or b"{}")
                    ok, msg = node._set_enabled(bool(body.get("enabled")))
                    self._send(200, json.dumps({"ok": ok, "message": msg}))
                except Exception as exc:  # noqa: BLE001
                    self._send(500, json.dumps({"ok": False, "message": str(exc)}))

            def log_message(self, *args):
                pass

        server = HTTPServer(("0.0.0.0", self._port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
