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
</script>
</body>
</html>
"""


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
