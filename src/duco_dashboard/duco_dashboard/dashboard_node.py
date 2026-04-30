"""Web dashboard for monitoring Duco robot state."""

from __future__ import annotations

import json
import math
import mimetypes
import os
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional
from urllib.parse import unquote, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from duco_dashboard.kinematics import forward_kinematics, joint_frames, rotation_to_rpy


RAD_TO_DEG = 180.0 / math.pi
JOINT_NAMES = [f'arm_1_joint_{i}' for i in range(1, 7)]
SHORT_NAMES = [f'J{i}' for i in range(1, 7)]

mimetypes.add_type('application/javascript', '.js')
mimetypes.add_type('model/stl', '.stl')


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, dashboard_node=None, html: bytes = b'',
                 duco_support_dir: str = '', **kwargs):
        self._dashboard = dashboard_node
        self._html = html
        self._duco_support_dir = duco_support_dir
        super().__init__(*args, **kwargs)

    def do_GET(self):  # noqa: N802
        if self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(self._html)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(self._html)
            return
        if self.path == '/api/state':
            payload = json.dumps(self._dashboard.get_state()).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(payload)
            return
        if self.path == '/api/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self._dashboard.register_client(self.wfile)
            try:
                while True:
                    time.sleep(1.0)
            except Exception:
                pass
            finally:
                self._dashboard.unregister_client(self.wfile)
            return
        request_path = urlparse(self.path).path
        if request_path.startswith('/duco_support/'):
            self._serve_duco_support_file(request_path.removeprefix('/duco_support/'))
            return
        super().do_GET()

    def _serve_duco_support_file(self, relative_path: str) -> None:
        safe_path = os.path.normpath(unquote(relative_path)).lstrip(os.sep)
        if safe_path.startswith('..') or os.path.isabs(safe_path):
            self.send_error(404)
            return

        file_path = os.path.join(self._duco_support_dir, safe_path)
        if not os.path.isfile(file_path):
            self.send_error(404)
            return

        with open(file_path, 'rb') as file_handle:
            payload = file_handle.read()
        self.send_response(200)
        self.send_header('Content-Type', self.guess_type(file_path))
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'public, max-age=3600')
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format, *args):
        return


class DucoDashboard(Node):
    def __init__(self):
        super().__init__('duco_dashboard')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8090)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wrench_topic', '/duco_ft_sensor/wrench_raw')
        self.declare_parameter('controller_state_topic', '/arm_1_controller/state')
        self.declare_parameter('push_rate', 20.0)
        self.declare_parameter('stale_after', 1.0)

        self._host = str(self.get_parameter('host').value)
        self._port = int(self.get_parameter('port').value)
        self._joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self._wrench_topic = str(self.get_parameter('wrench_topic').value)
        self._controller_state_topic = str(
            self.get_parameter('controller_state_topic').value)
        self._push_interval = 1.0 / max(float(self.get_parameter('push_rate').value), 1.0)
        self._stale_after = float(self.get_parameter('stale_after').value)

        self._lock = threading.Lock()
        self._client_lock = threading.Lock()
        self._sse_clients = []
        self._last_push = 0.0
        self._last_state: Dict[str, object] = {}

        self._joint_positions: Dict[str, float] = {}
        self._joint_velocities: Dict[str, float] = {}
        self._joint_efforts: Dict[str, float] = {}
        self._joint_stamp = 0.0
        self._joint_count = 0
        self._joint_bucket_start = time.monotonic()
        self._joint_bucket_count = 0
        self._joint_rate = 0.0

        self._wrench = {
            'force': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'torque': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'force_mag': 0.0,
        }
        self._wrench_stamp = 0.0
        self._wrench_count = 0

        self._controller = {
            'available': False,
            'desired': [],
            'actual': [],
            'error': [],
        }
        self._controller_stamp = 0.0

        self.create_subscription(
            JointState, self._joint_states_topic, self._joint_state_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            WrenchStamped, self._wrench_topic, self._wrench_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            JointTrajectoryControllerState, self._controller_state_topic,
            self._controller_state_cb, 10)

        pkg_share = get_package_share_directory('duco_dashboard')
        duco_support_share = get_package_share_directory('duco_support')
        static_dir = os.path.join(pkg_share, 'static')
        with open(os.path.join(static_dir, 'index.html'), 'rb') as html_file:
            html = html_file.read()

        handler = partial(
            DashboardHandler, dashboard_node=self, html=html,
            duco_support_dir=duco_support_share, directory=static_dir)
        self._httpd = ThreadingHTTPServer((self._host, self._port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(f'Duco dashboard: http://localhost:{self._port}')
        self.get_logger().info(f'serving Duco model assets from: {duco_support_share}')
        self.get_logger().info(
            f'subscribed: {self._joint_states_topic}, {self._wrench_topic}, '
            f'{self._controller_state_topic}')

    def destroy_node(self):
        self._httpd.shutdown()
        self._httpd.server_close()
        super().destroy_node()

    def _joint_state_cb(self, msg: JointState) -> None:
        now = time.monotonic()
        with self._lock:
            for index, name in enumerate(msg.name):
                if name not in JOINT_NAMES:
                    continue
                self._joint_positions[name] = _value_at(msg.position, index)
                self._joint_velocities[name] = _value_at(msg.velocity, index)
                self._joint_efforts[name] = _value_at(msg.effort, index)
            self._joint_stamp = now
            self._joint_count += 1
            self._joint_bucket_count += 1
            elapsed = now - self._joint_bucket_start
            if elapsed >= 1.0:
                self._joint_rate = self._joint_bucket_count / elapsed
                self._joint_bucket_count = 0
                self._joint_bucket_start = now
        self._maybe_push(now)

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        force = msg.wrench.force
        torque = msg.wrench.torque
        force_mag = math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)
        with self._lock:
            self._wrench = {
                'force': {'x': force.x, 'y': force.y, 'z': force.z},
                'torque': {'x': torque.x, 'y': torque.y, 'z': torque.z},
                'force_mag': force_mag,
            }
            self._wrench_stamp = time.monotonic()
            self._wrench_count += 1

    def _controller_state_cb(self, msg: JointTrajectoryControllerState) -> None:
        with self._lock:
            self._controller = {
                'available': True,
                'desired': _rounded_list(msg.desired.positions),
                'actual': _rounded_list(msg.actual.positions),
                'error': _rounded_list(msg.error.positions),
            }
            self._controller_stamp = time.monotonic()

    def _maybe_push(self, now: float) -> None:
        if now - self._last_push < self._push_interval:
            return
        self._last_push = now
        self._broadcast(json.dumps(self.get_state()))

    def get_state(self) -> Dict[str, object]:
        now = time.monotonic()
        with self._lock:
            positions = [self._joint_positions.get(name, 0.0) for name in JOINT_NAMES]
            velocities = [self._joint_velocities.get(name, 0.0) for name in JOINT_NAMES]
            efforts = [self._joint_efforts.get(name, math.nan) for name in JOINT_NAMES]
            joint_age = _age(now, self._joint_stamp)
            wrench_age = _age(now, self._wrench_stamp)
            controller_age = _age(now, self._controller_stamp)

            frames = joint_frames(positions)
            tcp = forward_kinematics(positions)
            rpy = rotation_to_rpy(tcp)
            links = [[round(frame[0][3], 5), round(frame[1][3], 5), round(frame[2][3], 5)]
                     for frame in frames]
            links.insert(0, [0.0, 0.0, 0.0])

            state = {
                'connected': joint_age is not None and joint_age <= self._stale_after,
                'joint_names': JOINT_NAMES,
                'short_names': SHORT_NAMES,
                'joints_rad': _rounded_list(positions, 5),
                'joints_deg': _rounded_list([value * RAD_TO_DEG for value in positions], 2),
                'velocity': _rounded_list(velocities, 5),
                'effort': _rounded_list(efforts, 3),
                'joint_state': {
                    'topic': self._joint_states_topic,
                    'count': self._joint_count,
                    'rate': round(self._joint_rate, 2),
                    'age': _round_or_none(joint_age, 3),
                    'fresh': joint_age is not None and joint_age <= self._stale_after,
                },
                'wrench': {
                    **_round_nested(self._wrench),
                    'topic': self._wrench_topic,
                    'count': self._wrench_count,
                    'age': _round_or_none(wrench_age, 3),
                    'fresh': wrench_age is not None and wrench_age <= self._stale_after,
                },
                'controller': {
                    **self._controller,
                    'topic': self._controller_state_topic,
                    'age': _round_or_none(controller_age, 3),
                    'fresh': controller_age is not None and controller_age <= self._stale_after,
                },
                'tcp': {
                    'position': _rounded_list([tcp[0][3], tcp[1][3], tcp[2][3]], 5),
                    'rpy_deg': _rounded_list([value * RAD_TO_DEG for value in rpy], 2),
                    'matrix': [[round(value, 5) for value in row] for row in tcp],
                },
                'links': links,
                'time': round(time.time(), 3),
            }
            self._last_state = state
            return state

    def register_client(self, wfile) -> None:
        with self._client_lock:
            self._sse_clients.append(wfile)
        try:
            wfile.write(f'data: {json.dumps(self.get_state())}\n\n'.encode())
            wfile.flush()
        except Exception:
            self.unregister_client(wfile)

    def unregister_client(self, wfile) -> None:
        with self._client_lock:
            if wfile in self._sse_clients:
                self._sse_clients.remove(wfile)

    def _broadcast(self, data: str) -> None:
        with self._client_lock:
            dead = []
            for client in self._sse_clients:
                try:
                    client.write(f'data: {data}\n\n'.encode())
                    client.flush()
                except Exception:
                    dead.append(client)
            for client in dead:
                self._sse_clients.remove(client)


def _value_at(values, index: int) -> float:
    if index < len(values):
        return float(values[index])
    return 0.0


def _rounded_list(values, digits: int = 4) -> List[Optional[float]]:
    result: List[Optional[float]] = []
    for value in values:
        if isinstance(value, float) and math.isnan(value):
            result.append(None)
        else:
            result.append(round(float(value), digits))
    return result


def _round_nested(value):
    if isinstance(value, dict):
        return {key: _round_nested(inner) for key, inner in value.items()}
    if isinstance(value, float):
        return round(value, 4)
    return value


def _age(now: float, stamp: float) -> Optional[float]:
    if stamp <= 0.0:
        return None
    return now - stamp


def _round_or_none(value: Optional[float], digits: int) -> Optional[float]:
    return None if value is None else round(value, digits)


_HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Duco Dashboard</title>
<style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    html, body { width: 100%; height: 100%; overflow: hidden; }
    body {
        background: #1a1a2e;
        color: #e0e6ef;
        font-family: "Segoe UI", system-ui, -apple-system, BlinkMacSystemFont, sans-serif;
    }
    #viewer {
        display: block;
        width: 100vw;
        height: 100vh;
        background: #1a1a2e;
    }
    #panel {
        position: fixed;
        top: 12px;
        right: 12px;
        width: min(420px, calc(100vw - 24px));
        max-height: calc(100vh - 24px);
        overflow: auto;
        background: rgba(22, 33, 62, 0.92);
        border: 1px solid rgba(79, 195, 247, 0.30);
        border-radius: 10px;
        padding: 14px 18px 16px;
        backdrop-filter: blur(8px);
        z-index: 10;
        box-shadow: 0 18px 48px rgba(0, 0, 0, 0.30);
    }
    #panel h1 {
        font-size: 16px;
        color: #4fc3f7;
        margin-bottom: 8px;
        font-weight: 650;
    }
    .status-row {
        display: flex;
        align-items: center;
        gap: 12px;
        margin-bottom: 10px;
        color: #90a4ae;
        font-size: 11px;
    }
    .status-row .push { margin-left: auto; }
    .status-dot {
        display: inline-block;
        width: 8px;
        height: 8px;
        border-radius: 50%;
        margin-right: 5px;
        vertical-align: -1px;
        background: #ef5350;
    }
    .status-dot.connected { background: #66bb6a; box-shadow: 0 0 8px rgba(102, 187, 106, 0.55); }
    .section { margin-top: 11px; }
    .section-title {
        font-size: 10px;
        color: #78909c;
        text-transform: uppercase;
        letter-spacing: 1px;
        margin-bottom: 5px;
    }
    .metrics {
        display: grid;
        grid-template-columns: repeat(3, minmax(0, 1fr));
        gap: 7px;
    }
    .metric {
        background: rgba(15, 25, 35, 0.82);
        border: 1px solid rgba(55, 71, 79, 0.82);
        border-radius: 6px;
        padding: 7px 8px;
        min-width: 0;
    }
    .metric-label {
        color: #78909c;
        font-size: 9px;
        text-transform: uppercase;
        letter-spacing: 0.7px;
        margin-bottom: 3px;
    }
    .metric-value {
        color: #66bb6a;
        font-family: Consolas, "SFMono-Regular", ui-monospace, monospace;
        font-size: 13px;
        white-space: nowrap;
        overflow: hidden;
        text-overflow: ellipsis;
    }
    .metric-value.warn { color: #ffa726; }
    .metric-value.bad { color: #ef5350; }
    .joint-row {
        display: grid;
        grid-template-columns: 25px minmax(90px, 1fr) 62px 56px;
        align-items: center;
        gap: 5px;
        height: 21px;
        margin-bottom: 3px;
    }
    .joint-label {
        font-size: 11px;
        font-weight: 650;
        text-align: center;
    }
    .joint-bar-bg {
        height: 16px;
        background: #0f1923;
        border-radius: 3px;
        position: relative;
        overflow: hidden;
        box-shadow: inset 0 0 0 1px rgba(55, 71, 79, 0.6);
    }
    .joint-center {
        position: absolute;
        left: 50%;
        top: 0;
        bottom: 0;
        width: 1px;
        background: #37474f;
        z-index: 2;
    }
    .joint-bar {
        height: 100%;
        border-radius: 3px;
        transition: width 0.06s ease-out;
        position: absolute;
        top: 0;
    }
    .joint-bar.positive { left: 50%; }
    .joint-bar.negative { right: 50%; }
    .joint-val {
        text-align: right;
        font-size: 10px;
        font-family: Consolas, "SFMono-Regular", ui-monospace, monospace;
        color: #4fc3f7;
        white-space: nowrap;
    }
    .joint-vel { color: #78909c; }
    .info-row {
        display: flex;
        justify-content: space-between;
        gap: 12px;
        padding: 2px 0;
        font-size: 12px;
    }
    .info-label { color: #78909c; }
    .info-value {
        color: #66bb6a;
        font-family: Consolas, "SFMono-Regular", ui-monospace, monospace;
        text-align: right;
    }
    .info-value.force { color: #ffa726; }
    #tcpDisplay, #topicDisplay {
        font-family: Consolas, "SFMono-Regular", ui-monospace, monospace;
        font-size: 10px;
        color: #4fc3f7;
        white-space: pre;
        background: rgba(15, 25, 35, 0.68);
        border: 1px solid rgba(55, 71, 79, 0.65);
        border-radius: 6px;
        padding: 8px;
        overflow: auto;
    }
    #sceneLabel {
        position: fixed;
        left: 14px;
        bottom: 12px;
        z-index: 5;
        color: #78909c;
        font-size: 11px;
        font-family: Consolas, "SFMono-Regular", ui-monospace, monospace;
        background: rgba(15, 25, 35, 0.52);
        border: 1px solid rgba(55, 71, 79, 0.45);
        border-radius: 6px;
        padding: 6px 8px;
    }
    @media (max-width: 760px) {
        #panel { left: 10px; right: 10px; width: auto; }
        .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
</style>
</head>
<body>
<canvas id="viewer"></canvas>
<div id="sceneLabel">base_link / link_6</div>
<div id="panel">
    <h1>Duco GCR5-910</h1>
    <div class="status-row">
        <div><span class="status-dot" id="connDot"></span><span id="connText">Disconnected</span></div>
        <div class="push"><span id="rateVal">-- Hz</span></div>
    </div>
    <div class="metrics">
        <div class="metric"><div class="metric-label">Joint Age</div><div class="metric-value" id="jointAge">--</div></div>
        <div class="metric"><div class="metric-label">Controller</div><div class="metric-value" id="controllerState">--</div></div>
        <div class="metric"><div class="metric-label">Wrench</div><div class="metric-value" id="wrenchState">--</div></div>
    </div>
    <div class="section">
        <div class="section-title">Joints</div>
        <div id="joints"></div>
    </div>
    <div class="section">
        <div class="section-title">Force / Torque</div>
        <div class="info-row"><span class="info-label">F</span><span class="info-value force" id="forceVal">--</span></div>
        <div class="info-row"><span class="info-label">T</span><span class="info-value force" id="torqueVal">--</span></div>
        <div class="info-row"><span class="info-label">|F|</span><span class="info-value force" id="forceMag">--</span></div>
    </div>
    <div class="section">
        <div class="section-title">TCP Pose</div>
        <div id="tcpDisplay">--</div>
    </div>
    <div class="section">
        <div class="section-title">Topics</div>
        <div id="topicDisplay">--</div>
    </div>
</div>
<script>
const COLORS = ['#42a5f5', '#66bb6a', '#ffa726', '#ef5350', '#ab47bc', '#26c6da'];
const JOINT_RANGE_DEG = [360, 360, 360, 360, 360, 360];
const jointHost = document.getElementById('joints');
for (let i = 0; i < 6; i++) {
    jointHost.insertAdjacentHTML('beforeend', `<div class="joint-row">
        <div class="joint-label" style="color:${COLORS[i]}">J${i + 1}</div>
        <div class="joint-bar-bg"><div class="joint-center"></div><div class="joint-bar" id="jbar${i}" style="width:0%"></div></div>
        <div class="joint-val" id="jval${i}">0.0°</div>
        <div class="joint-val joint-vel" id="jvel${i}">0.000</div>
    </div>`);
}
const canvas = document.getElementById('viewer');
const ctx = canvas.getContext('2d');
let latest = null;
function fmt(v, digits = 3) { return v === null || v === undefined || Number.isNaN(Number(v)) ? '--' : Number(v).toFixed(digits); }
function freshnessClass(fresh, count) { if (!count) return 'bad'; return fresh ? '' : 'warn'; }
function setMetric(id, value, klass = '') { const el = document.getElementById(id); el.textContent = value; el.className = `metric-value ${klass}`; }
function updateBar(index, deg) {
    const bar = document.getElementById(`jbar${index}`);
    const halfRange = JOINT_RANGE_DEG[index] / 2;
    const pct = Math.min(Math.abs(deg) / halfRange * 50, 50);
    bar.style.width = `${pct}%`;
    bar.className = 'joint-bar ' + (deg >= 0 ? 'positive' : 'negative');
    bar.style.background = `linear-gradient(${deg >= 0 ? '90deg' : '270deg'}, ${COLORS[index]}88, ${COLORS[index]})`;
    document.getElementById(`jval${index}`).textContent = `${fmt(deg, 1)}°`;
}
function update(state) {
    latest = state;
    const dot = document.getElementById('connDot');
    dot.className = 'status-dot' + (state.connected ? ' connected' : '');
    document.getElementById('connText').textContent = state.connected ? 'Connected' : 'Disconnected';
    document.getElementById('rateVal').textContent = `${fmt(state.joint_state.rate, 1)} Hz`;
    setMetric('jointAge', `${fmt(state.joint_state.age, 3)} s`, freshnessClass(state.joint_state.fresh, state.joint_state.count));
    setMetric('controllerState', state.controller.available ? 'active' : 'missing', freshnessClass(state.controller.fresh, state.controller.available));
    setMetric('wrenchState', `${fmt(state.wrench.age, 3)} s`, freshnessClass(state.wrench.fresh, state.wrench.count));
    for (let i = 0; i < 6; i++) {
        updateBar(i, state.joints_deg[i] || 0);
        document.getElementById(`jvel${i}`).textContent = fmt(state.velocity[i], 3);
    }
    const force = state.wrench.force;
    const torque = state.wrench.torque;
    document.getElementById('forceVal').textContent = `(${fmt(force.x, 2)}, ${fmt(force.y, 2)}, ${fmt(force.z, 2)}) N`;
    document.getElementById('torqueVal').textContent = `(${fmt(torque.x, 3)}, ${fmt(torque.y, 3)}, ${fmt(torque.z, 3)}) Nm`;
    document.getElementById('forceMag').textContent = `${fmt(state.wrench.force_mag, 2)} N`;
    document.getElementById('tcpDisplay').textContent =
        `Pos:  (${state.tcp.position.map(v => fmt(v, 4)).join(', ')}) m\n` +
        `RPY:  (${state.tcp.rpy_deg.map(v => fmt(v, 2)).join(', ')})°\n` +
        state.tcp.matrix.map(row => row.map(v => fmt(v, 4).padStart(8)).join(' ')).join('\n');
    document.getElementById('topicDisplay').textContent =
        `${state.joint_state.topic}   ${state.joint_state.count} msgs\n` +
        `${state.wrench.topic}   ${state.wrench.count} msgs\n` +
        `${state.controller.topic}   age ${fmt(state.controller.age, 3)} s`;
    drawScene();
}
function resizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.floor(window.innerWidth * dpr);
    canvas.height = Math.floor(window.innerHeight * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
function drawGrid(width, height) {
    ctx.strokeStyle = '#333355';
    ctx.lineWidth = 1;
    const spacing = 48;
    for (let x = -spacing; x < width + spacing; x += spacing) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x + height * 0.38, height); ctx.stroke();
    }
    for (let y = height * 0.12; y < height + spacing; y += spacing) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }
}
function project(point, scale, originX, originY) {
    const x = point[0], y = point[1], z = point[2];
    return [originX + (x - y * 0.35) * scale, originY - (z + y * 0.16) * scale];
}
function drawScene() {
    resizeCanvas();
    const width = window.innerWidth;
    const height = window.innerHeight;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, width, height);
    drawGrid(width, height);
    ctx.save();
    const originX = width * 0.43;
    const originY = height * 0.72;
    const scale = Math.min(width, height) * 0.72;
    ctx.fillStyle = 'rgba(0, 0, 0, 0.22)';
    ctx.beginPath();
    ctx.ellipse(originX + 40, originY + 18, scale * 0.38, scale * 0.06, 0, 0, Math.PI * 2);
    ctx.fill();
    if (!latest || !latest.links) {
        ctx.restore();
        return;
    }
    const pts = latest.links.map(point => project(point, scale, originX, originY));
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.36)';
    ctx.lineWidth = 14;
    ctx.beginPath();
    pts.forEach((p, i) => i ? ctx.lineTo(p[0] + 4, p[1] + 6) : ctx.moveTo(p[0] + 4, p[1] + 6));
    ctx.stroke();
    for (let i = 1; i < pts.length; i++) {
        const grad = ctx.createLinearGradient(pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1]);
        grad.addColorStop(0, COLORS[(i - 1) % COLORS.length]);
        grad.addColorStop(1, COLORS[i % COLORS.length]);
        ctx.strokeStyle = grad;
        ctx.lineWidth = i === 1 ? 13 : 11;
        ctx.beginPath(); ctx.moveTo(pts[i - 1][0], pts[i - 1][1]); ctx.lineTo(pts[i][0], pts[i][1]); ctx.stroke();
    }
    pts.forEach((p, i) => {
        ctx.fillStyle = i === pts.length - 1 ? '#ffa726' : '#e0e6ef';
        ctx.strokeStyle = '#16213e';
        ctx.lineWidth = 3;
        ctx.beginPath(); ctx.arc(p[0], p[1], i === 0 ? 8 : 6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    });
    const tcp = pts[pts.length - 1];
    ctx.strokeStyle = '#ef5350';
    ctx.lineWidth = 3;
    const mag = Math.min(Math.max((latest.wrench.force_mag || 0) * 2.2, 0), 130);
    if (mag > 5) {
        const fx = latest.wrench.force.x || 0;
        const fz = latest.wrench.force.z || 0;
        const len = Math.hypot(fx, fz) || 1;
        const dx = fx / len * mag;
        const dy = -fz / len * mag;
        ctx.beginPath(); ctx.moveTo(tcp[0], tcp[1]); ctx.lineTo(tcp[0] + dx, tcp[1] + dy); ctx.stroke();
        ctx.fillStyle = '#ef5350';
        ctx.beginPath(); ctx.arc(tcp[0] + dx, tcp[1] + dy, 4, 0, Math.PI * 2); ctx.fill();
    }
    ctx.restore();
}
window.addEventListener('resize', drawScene);
function connectSSE() {
    const source = new EventSource('/api/events');
    source.onmessage = event => { try { update(JSON.parse(event.data)); } catch (_) {} };
    source.onerror = () => { source.close(); setTimeout(connectSSE, 1500); };
}
fetch('/api/state').then(response => response.json()).then(update).catch(() => drawScene());
connectSSE();
drawScene();
</script>
</body>
</html>'''


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DucoDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()