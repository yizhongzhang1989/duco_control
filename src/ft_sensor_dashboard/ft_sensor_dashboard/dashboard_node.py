"""FastAPI-based web dashboard for a geometry_msgs/WrenchStamped topic.

Endpoints
---------
  GET /            HTML page with two HTML5 Canvas plots (Force / Torque).
  GET /events      Server-Sent Events stream (newest samples since the last
                   push), updated push_rate times per second.
  GET /api/info    JSON snapshot of dashboard parameters.
  GET /api/latest  JSON snapshot of the latest wrench sample.

Multiple browsers can connect at the same time.

Parameters
----------
  topic            string  default "/duco_ft_sensor/wrench_raw"
  host             string  default "0.0.0.0"
  port             int     default 8080
  window_seconds   double  default 10.0
  push_rate        double  default 30.0
  reliability      string  default "best_effort"   ("reliable" or "best_effort")
  title            string  default ""
"""

from __future__ import annotations

import asyncio
import json
import logging
import socket
import threading
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

import rclpy
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


Sample = Tuple[float, float, float, float, float, float, float]
# (t, fx, fy, fz, tx, ty, tz)


# ---------------------------------------------------------------------------
# Shared state between ROS thread and HTTP request handlers
# ---------------------------------------------------------------------------
class WrenchBuffer:
    # Length of the tumbling rate-measurement bucket.
    RATE_WINDOW = 1.0  # seconds

    def __init__(self, window_seconds: float):
        self.window = window_seconds
        cap = max(int(window_seconds * 2000.0), 2000)
        self._buf: Deque[Sample] = deque(maxlen=cap)
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        # Tumbling 1-second rate counter:
        #   _bucket_start  -- monotonic time of the current bucket's start
        #   _bucket_count  -- samples received since _bucket_start
        #   _last_rate     -- count from the most recently *completed* bucket
        self._bucket_start: float = self._t0
        self._bucket_count: int = 0
        self._last_rate: Optional[float] = None
        self.received = 0

    def push(self, msg: WrenchStamped) -> None:
        # NOTE: the wire frame has no per-sample timestamp; both wall-clock
        # receive time and publisher header.stamp are bursty (each batch of
        # buffered USB frames is processed in microseconds), so 1/dt between
        # adjacent samples is meaningless. The displayed rate is computed
        # from arrival counts over a fixed 1-second tumbling window.
        now = time.monotonic()
        t = now - self._t0
        f = msg.wrench.force
        m = msg.wrench.torque
        sample: Sample = (t, f.x, f.y, f.z, m.x, m.y, m.z)
        with self._lock:
            self._buf.append(sample)
            self.received += 1
            elapsed = now - self._bucket_start
            if elapsed >= self.RATE_WINDOW:
                # Close the current bucket: rate = count / actual_duration.
                # Using actual elapsed time (not exactly 1.0) avoids over-
                # counting on jitter; the rate refreshes once per second
                # with no smoothing across buckets.
                self._last_rate = self._bucket_count / elapsed
                self._bucket_start = now
                self._bucket_count = 1  # the sample we just received
            else:
                self._bucket_count += 1

    def rate_hz(self) -> Optional[float]:
        """Sample count from the most recently completed 1-second bucket.

        Returns ``None`` until at least one full bucket has finished. The
        value updates exactly once per second when a new bucket closes; it
        is never averaged or smoothed across buckets.
        """
        with self._lock:
            return self._last_rate

    def snapshot(self) -> Tuple[List[Sample], int]:
        with self._lock:
            if not self._buf:
                return [], self.received
            t_now = self._buf[-1][0]
            t_min = t_now - self.window
            kept: List[Sample] = []
            for s in reversed(self._buf):
                if s[0] < t_min:
                    break
                kept.append(s)
            kept.reverse()
            return kept, self.received

    def latest(self) -> Optional[Sample]:
        with self._lock:
            return self._buf[-1] if self._buf else None

    def newer_than(self, t_ref: Optional[float]) -> Tuple[List[Sample], int]:
        with self._lock:
            recv = self.received
            if not self._buf:
                return [], recv
            if t_ref is None:
                # send a single seed sample so the client gets immediate data
                return [self._buf[-1]], recv
            new: List[Sample] = []
            for s in reversed(self._buf):
                if s[0] <= t_ref:
                    break
                new.append(s)
            new.reverse()
            return new, recv


# ---------------------------------------------------------------------------
# ROS subscriber node
# ---------------------------------------------------------------------------
class WrenchSubscriber(Node):
    def __init__(self, topic: str, reliability: str, buf: WrenchBuffer):
        super().__init__("ft_sensor_dashboard")

        rel = (ReliabilityPolicy.BEST_EFFORT
               if reliability.lower().startswith("best")
               else ReliabilityPolicy.RELIABLE)

        qos = QoSProfile(
            reliability=rel,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self._buf = buf
        self.create_subscription(WrenchStamped, topic, self._cb, qos)
        self.get_logger().info(
            f"subscribed to {topic} (reliability={rel.name})")

    def _cb(self, msg: WrenchStamped) -> None:
        self._buf.push(msg)


# ---------------------------------------------------------------------------
# HTML page (single self-contained file)
# ---------------------------------------------------------------------------
_HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  :root { color-scheme: light dark; }
  body { font-family: ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
         margin: 0; padding: 16px; background: #0f1115; color: #e6e6e6; }
  h1   { margin: 0 0 4px; font-size: 1.05rem; font-weight: 600; }
  .sub { color: #8a93a6; font-size: 0.85rem; margin-bottom: 12px; }
  .panel { background: #181b22; border: 1px solid #262a33; border-radius: 8px;
           padding: 10px 12px 12px; margin-bottom: 12px; }
  .row { display: flex; gap: 24px; flex-wrap: wrap; align-items: baseline;
         font-size: 0.85rem; color: #cfd3dc; margin-bottom: 6px; }
  .row b { color: #ffffff; }
  .legend { display: flex; gap: 14px; font-size: 0.8rem; }
  .legend span { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { display: inline-block; width: 10px; height: 10px; border-radius: 2px; }
  canvas { width: 100%; height: 260px; display: block; background: #0c0e13;
           border-radius: 6px; }
  .status { font-size: 0.8rem; color: #8a93a6; }
  .status.ok  { color: #6fcf97; }
  .status.bad { color: #ff7575; }
  code { background: #11141a; padding: 1px 5px; border-radius: 3px; }
  .controls { display: flex; gap: 12px; flex-wrap: wrap; align-items: center;
              margin-bottom: 12px; font-size: 0.85rem; color: #cfd3dc; }
  .controls label { display: inline-flex; align-items: center; gap: 6px; }
  .controls input[type="number"] {
    width: 72px; background: #11141a; color: #e6e6e6;
    border: 1px solid #2b3140; border-radius: 4px;
    padding: 4px 6px; font: inherit;
  }
  .controls input[type="number"]:focus { outline: none; border-color: #4f7cff; }
  .controls button {
    background: #2563eb; color: white; border: 0; border-radius: 4px;
    padding: 5px 12px; font: inherit; cursor: pointer;
  }
  .controls button:hover  { background: #1d4ed8; }
  .controls button.frozen { background: #f59e0b; }
  .controls button.frozen:hover { background: #d97706; }
  .badge {
    display: inline-block; padding: 2px 7px; border-radius: 999px;
    background: #1f2937; color: #cfd3dc; font-variant-numeric: tabular-nums;
  }
  .badge.ok  { background: #14532d; color: #c6f7d6; }
  .badge.bad { background: #5a1d1d; color: #ffb4b4; }
</style>
</head>
<body>
  <h1>__TITLE__</h1>
  <div class="sub">topic <code>__TOPIC__</code> · server <code>FastAPI</code>
       · <span id="conn" class="status">connecting...</span></div>

  <div class="controls">
    <label>window <input id="in-window" type="number" min="0.5" step="0.5" value="__WINDOW__"> s</label>
    <label>push rate <input id="in-rate" type="number" min="1" max="200" step="1" value="__RATE__"> Hz</label>
    <button id="btn-apply" type="button">Apply</button>
    <button id="btn-freeze" type="button">Freeze</button>
    <span style="margin-left:auto">
      msg rate <span id="hz" class="badge">-- Hz</span>
      &nbsp;received <span id="cnt" class="badge">0</span>
    </span>
  </div>

  <div class="panel">
    <div class="row">
      <div><b>Force [N]</b></div>
      <div class="legend">
        <span><i class="swatch" style="background:#ef4444"></i>Fx <b id="vfx">--</b></span>
        <span><i class="swatch" style="background:#22c55e"></i>Fy <b id="vfy">--</b></span>
        <span><i class="swatch" style="background:#3b82f6"></i>Fz <b id="vfz">--</b></span>
      </div>
    </div>
    <canvas id="cvF"></canvas>
  </div>

  <div class="panel">
    <div class="row">
      <div><b>Torque [N·m]</b></div>
      <div class="legend">
        <span><i class="swatch" style="background:#ef4444"></i>Mx <b id="vmx">--</b></span>
        <span><i class="swatch" style="background:#22c55e"></i>My <b id="vmy">--</b></span>
        <span><i class="swatch" style="background:#3b82f6"></i>Mz <b id="vmz">--</b></span>
      </div>
    </div>
    <canvas id="cvT"></canvas>
  </div>

<script>
(() => {
  let WINDOW = __WINDOW__;
  let RATE   = __RATE__;
  const COLORS = ["#ef4444", "#22c55e", "#3b82f6"];

  // --- storage --------------------------------------------------------------
  // Plain typed arrays + a `len` counter. We periodically copy the surviving
  // window down to index 0 (compact) instead of using a ring buffer; that
  // keeps the draw code (which needs contiguous indices for binary search)
  // simple while still being O(1) amortised per push.
  const CAP = 200000;                          // ~200 s @ 1 kHz, plenty for a 60 s window
  const ts   = new Float64Array(CAP);          // sensor-side seconds
  const vals = [                               // values, indices match channels[]
    new Float32Array(CAP), new Float32Array(CAP), new Float32Array(CAP),  // Fx Fy Fz
    new Float32Array(CAP), new Float32Array(CAP), new Float32Array(CAP),  // Mx My Mz
  ];
  let len = 0;
  let tNow = 0;
  let received = 0;
  let frozen = false;
  const FORCE_CHS  = [0, 1, 2];
  const TORQUE_CHS = [3, 4, 5];

  function pushSample(t, fx, fy, fz, mx, my, mz) {
    if (len === CAP) compact(true);
    const i = len++;
    ts[i] = t;
    vals[0][i] = fx; vals[1][i] = fy; vals[2][i] = fz;
    vals[3][i] = mx; vals[4][i] = my; vals[5][i] = mz;
  }

  // First index i in [0, len) where ts[i] >= x. Standard lower_bound.
  function lowerBound(x) {
    let lo = 0, hi = len;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (ts[mid] < x) lo = mid + 1; else hi = mid;
    }
    return lo;
  }

  // Drop samples older than the visible window (with 5% hysteresis to avoid
  // shuffling on every push). If the buffer has filled completely, drop the
  // oldest half unconditionally as a safety valve.
  function compact(forceDrop) {
    const cutoff = tNow - WINDOW * 1.05;
    const i0 = lowerBound(cutoff);
    if (i0 > 0) {
      const survive = len - i0;
      if (survive > 0) {
        ts.copyWithin(0, i0, len);
        for (const v of vals) v.copyWithin(0, i0, len);
      }
      len = survive;
    }
    if (forceDrop && len === CAP) {
      const drop = CAP >>> 1;
      ts.copyWithin(0, drop, CAP);
      for (const v of vals) v.copyWithin(0, drop, CAP);
      len = CAP - drop;
    }
  }

  const cvF = document.getElementById("cvF");
  const cvT = document.getElementById("cvT");
  const conn = document.getElementById("conn");
  const cnt  = document.getElementById("cnt");
  const hzEl = document.getElementById("hz");
  const inWindow = document.getElementById("in-window");
  const inRate   = document.getElementById("in-rate");
  const btnApply  = document.getElementById("btn-apply");
  const btnFreeze = document.getElementById("btn-freeze");
  const els = {
    Fx: document.getElementById("vfx"), Fy: document.getElementById("vfy"), Fz: document.getElementById("vfz"),
    Mx: document.getElementById("vmx"), My: document.getElementById("vmy"), Mz: document.getElementById("vmz"),
  };

  // --- live frequency: server-side wall-clock count ----------------------
  // The server measures samples-per-second over a fixed 1 s window
  // (analogous to `ros2 topic hz`) and ships rate_hz in every SSE event.
  // Each value reflects the most recent 1 s of traffic without any
  // smoothing across SSE events. We blank the badge if no fresh value has
  // arrived in the last second.
  let lastRateArrival = 0;  // performance.now() of last rate update
  let serverRateHz = NaN;
  function updateHz() {
    if (performance.now() - lastRateArrival > 1500) {
      hzEl.textContent = "-- Hz";
      hzEl.className = "badge bad";
      return;
    }
    if (!isFinite(serverRateHz) || serverRateHz <= 0) {
      hzEl.textContent = "-- Hz";
      hzEl.className = "badge";
      return;
    }
    hzEl.textContent = serverRateHz.toFixed(1) + " Hz";
    hzEl.className = "badge ok";
  }
  setInterval(updateHz, 100);

  function fitCanvas(c) {
    const dpr = window.devicePixelRatio || 1;
    const r = c.getBoundingClientRect();
    c.width  = Math.max(1, Math.round(r.width  * dpr));
    c.height = Math.max(1, Math.round(r.height * dpr));
    return dpr;
  }

  function drawPlot(c, channels, ylabel) {
    const dpr = fitCanvas(c);
    const ctx = c.getContext("2d");
    const W = c.width, H = c.height;
    ctx.fillStyle = "#0c0e13"; ctx.fillRect(0, 0, W, H);

    const pad = { l: 50*dpr, r: 14*dpr, t: 10*dpr, b: 24*dpr };
    const plotW = W - pad.l - pad.r;
    const plotH = H - pad.t - pad.b;

    const tMin = tNow - WINDOW;
    const tMax = tNow;
    const i0 = lowerBound(tMin);

    // Min/max-per-pixel-column downsample. For each visible channel build
    // mins[col], maxs[col], have[col] in a single linear pass over the
    // visible samples. Drawing then needs O(plotW) work per channel
    // regardless of how many samples the window contains.
    const cols = Math.max(1, plotW | 0);
    const inv = cols / WINDOW;
    const downs = new Array(channels.length);
    let yMin = Infinity, yMax = -Infinity;
    for (let ci = 0; ci < channels.length; ci++) {
      const arr = vals[channels[ci]];
      const colMin  = new Float32Array(cols);
      const colMax  = new Float32Array(cols);
      const colHave = new Uint8Array(cols);
      for (let i = i0; i < len; i++) {
        let px = ((ts[i] - tMin) * inv) | 0;
        if (px < 0) px = 0; else if (px >= cols) px = cols - 1;
        const v = arr[i];
        if (colHave[px]) {
          if (v < colMin[px]) colMin[px] = v;
          if (v > colMax[px]) colMax[px] = v;
        } else {
          colHave[px] = 1; colMin[px] = v; colMax[px] = v;
        }
        if (v < yMin) yMin = v;
        if (v > yMax) yMax = v;
      }
      downs[ci] = { min: colMin, max: colMax, have: colHave };
    }
    if (!isFinite(yMin) || !isFinite(yMax)) { yMin = -1; yMax = 1; }
    if (yMin === yMax) { yMin -= 1; yMax += 1; }
    const yPad = 0.1 * (yMax - yMin);
    yMin -= yPad; yMax += yPad;

    const xPx = (px) => pad.l + px + 0.5;
    const yPx = (v) => pad.t + (1 - (v - yMin) / (yMax - yMin)) * plotH;

    // Grid + axes.
    ctx.strokeStyle = "#22262f"; ctx.lineWidth = 1*dpr;
    ctx.fillStyle   = "#8a93a6"; ctx.font = (10*dpr) + "px ui-sans-serif, sans-serif";
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    const yTicks = 5;
    for (let i = 0; i <= yTicks; i++) {
      const v = yMin + (yMax - yMin) * i / yTicks;
      const yy = yPx(v);
      ctx.beginPath(); ctx.moveTo(pad.l, yy); ctx.lineTo(W - pad.r, yy); ctx.stroke();
      ctx.fillText(v.toFixed(2), pad.l - 4*dpr, yy);
    }
    ctx.textAlign = "center"; ctx.textBaseline = "top";
    const xTicks = 5;
    for (let i = 0; i <= xTicks; i++) {
      const t = tMin + (tMax - tMin) * i / xTicks;
      const xx = pad.l + ((t - tMin) / WINDOW) * plotW;
      ctx.beginPath(); ctx.moveTo(xx, pad.t); ctx.lineTo(xx, H - pad.b); ctx.stroke();
      ctx.fillText((t - tNow).toFixed(1) + "s", xx, H - pad.b + 3*dpr);
    }
    if (0 > yMin && 0 < yMax) {
      ctx.strokeStyle = "#3a4050";
      const yz = yPx(0);
      ctx.beginPath(); ctx.moveTo(pad.l, yz); ctx.lineTo(W - pad.r, yz); ctx.stroke();
    }
    ctx.save();
    ctx.translate(12*dpr, pad.t + plotH/2);
    ctx.rotate(-Math.PI/2);
    ctx.fillStyle = "#cfd3dc"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(ylabel, 0, 0);
    ctx.restore();

    // Trace: each populated column draws a vertical line spanning the column's
    // min..max, and consecutive columns are joined. Adjacent line segments
    // share endpoints so a slow-changing signal looks like a thin line and a
    // noisy one looks like a band -- the same envelope rendering used by
    // oscilloscope min/max plots.
    ctx.lineWidth = 1.4 * dpr;
    for (let ci = 0; ci < downs.length; ci++) {
      const d = downs[ci];
      ctx.strokeStyle = COLORS[ci];
      ctx.beginPath();
      let started = false;
      for (let px = 0; px < cols; px++) {
        if (!d.have[px]) continue;
        const xa = xPx(px);
        const yTop = yPx(d.max[px]);
        const yBot = yPx(d.min[px]);
        if (!started) {
          ctx.moveTo(xa, yTop);
          started = true;
        } else {
          ctx.lineTo(xa, yTop);
        }
        if (yBot !== yTop) ctx.lineTo(xa, yBot);
      }
      ctx.stroke();
    }
  }

  function render() {
    drawPlot(cvF, FORCE_CHS,  "Force [N]");
    drawPlot(cvT, TORQUE_CHS, "Torque [N·m]");
    requestAnimationFrame(render);
  }
  requestAnimationFrame(render);

  function setConn(state, msg) {
    conn.textContent = msg;
    conn.classList.remove("ok", "bad");
    if (state) conn.classList.add(state);
  }

  let es = null;
  function open() {
    if (es) { try { es.close(); } catch {} es = null; }
    es = new EventSource("/events?rate=" + encodeURIComponent(RATE));
    es.onopen  = () => setConn("ok",  "live");
    es.onerror = () => setConn("bad", "disconnected (retrying)");
    es.onmessage = (ev) => {
      let d;
      try { d = JSON.parse(ev.data); } catch { return; }
      const samples = d.samples || [];
      received = d.received || received;
      cnt.textContent = received;
      if (frozen || !samples.length) { return; }
      for (const s of samples) {
        tNow = s[0];
        pushSample(s[0], s[1], s[2], s[3], s[4], s[5], s[6]);
      }
      // Server-computed wall-clock rate; one value per SSE event, no smoothing.
      if (typeof d.rate_hz === "number") {
        serverRateHz = d.rate_hz;
        lastRateArrival = performance.now();
      }
      const last = samples[samples.length - 1];
      els.Fx.textContent = last[1].toFixed(2);
      els.Fy.textContent = last[2].toFixed(2);
      els.Fz.textContent = last[3].toFixed(2);
      els.Mx.textContent = last[4].toFixed(3);
      els.My.textContent = last[5].toFixed(3);
      els.Mz.textContent = last[6].toFixed(3);
      compact(false);
    };
  }

  // --- UI controls -------------------------------------------------------
  function applyControls() {
    const w = parseFloat(inWindow.value);
    const r = parseFloat(inRate.value);
    if (isFinite(w) && w >= 0.5) WINDOW = w;
    let needReopen = false;
    if (isFinite(r) && r >= 1 && r <= 200 && Math.abs(r - RATE) > 0.01) {
      RATE = r;
      needReopen = true;
    }
    compact(false);
    if (needReopen) open();
  }
  btnApply.addEventListener("click", applyControls);
  for (const el of [inWindow, inRate]) {
    el.addEventListener("keydown", (ev) => {
      if (ev.key === "Enter") applyControls();
    });
  }
  btnFreeze.addEventListener("click", () => {
    frozen = !frozen;
    btnFreeze.textContent = frozen ? "Resume" : "Freeze";
    btnFreeze.classList.toggle("frozen", frozen);
  });

  open();
})();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# FastAPI app factory
# ---------------------------------------------------------------------------
def build_app(buf: WrenchBuffer, *, topic: str, title: str,
              window: float, push_rate: float) -> FastAPI:
    app = FastAPI(title="ft_sensor_dashboard", version="0.1.0",
                  docs_url=None, redoc_url=None)

    page_title = title or f"F/T sensor dashboard ({topic})"
    html_body = (_HTML_TEMPLATE
                 .replace("__TITLE__", page_title)
                 .replace("__TOPIC__", topic)
                 .replace("__WINDOW__", f"{window:g}")
                 .replace("__RATE__",   f"{push_rate:g}"))

    @app.get("/", response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        return HTMLResponse(content=html_body,
                            headers={"Cache-Control": "no-store"})

    @app.get("/api/info")
    async def info() -> JSONResponse:
        return JSONResponse({
            "topic": topic, "title": title,
            "window_seconds": window, "push_rate": push_rate,
            "server": "fastapi", "received": buf.received,
            "rate_hz": buf.rate_hz(),
        })

    @app.get("/api/latest")
    async def latest() -> JSONResponse:
        s = buf.latest()
        if s is None:
            return JSONResponse({"sample": None, "received": buf.received})
        return JSONResponse({
            "sample": {
                "t": s[0],
                "force":  {"x": s[1], "y": s[2], "z": s[3]},
                "torque": {"x": s[4], "y": s[5], "z": s[6]},
            },
            "received": buf.received,
        })

    async def event_stream(period: float):
        last_t: Optional[float] = None
        try:
            while True:
                samples, recv = buf.newer_than(last_t)
                if samples:
                    last_t = samples[-1][0]
                    payload = json.dumps(
                        {"samples": samples, "received": recv,
                         "rate_hz": buf.rate_hz()},
                        separators=(",", ":"))
                    yield f"data: {payload}\n\n"
                else:
                    yield ": keepalive\n\n"
                await asyncio.sleep(period)
        except asyncio.CancelledError:  # client disconnected
            return

    @app.get("/events")
    async def events(rate: Optional[float] = None) -> StreamingResponse:
        # Per-client push rate; falls back to server default. Clamp 1..200 Hz.
        eff = push_rate if rate is None else float(rate)
        if not (eff > 0):
            eff = push_rate
        eff = max(1.0, min(200.0, eff))
        return StreamingResponse(
            event_stream(1.0 / eff),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-store",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    return app


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _ros_spin(executor: SingleThreadedExecutor, stop: threading.Event) -> None:
    while not stop.is_set() and rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


def _local_ip_hint() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "<this-host>"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    rclpy.init()

    bootstrap = rclpy.create_node("ft_sensor_dashboard_bootstrap")
    bootstrap.declare_parameter("topic", "/duco_ft_sensor/wrench_raw")
    bootstrap.declare_parameter("host", "0.0.0.0")
    bootstrap.declare_parameter("port", 8080)
    bootstrap.declare_parameter("window_seconds", 10.0)
    bootstrap.declare_parameter("push_rate", 30.0)
    bootstrap.declare_parameter("reliability", "best_effort")
    bootstrap.declare_parameter("title", "")

    topic = bootstrap.get_parameter("topic").get_parameter_value().string_value
    host = bootstrap.get_parameter("host").get_parameter_value().string_value
    port = bootstrap.get_parameter("port").get_parameter_value().integer_value
    window = bootstrap.get_parameter("window_seconds").get_parameter_value().double_value
    push_rate = bootstrap.get_parameter("push_rate").get_parameter_value().double_value
    reliability = bootstrap.get_parameter("reliability").get_parameter_value().string_value
    title = bootstrap.get_parameter("title").get_parameter_value().string_value
    bootstrap.destroy_node()

    if window <= 0.0:
        window = 10.0
    if push_rate <= 0.0:
        push_rate = 30.0

    buf = WrenchBuffer(window)

    sub_node = WrenchSubscriber(topic, reliability, buf)
    executor = SingleThreadedExecutor()
    executor.add_node(sub_node)
    stop = threading.Event()
    ros_thread = threading.Thread(target=_ros_spin, args=(executor, stop), daemon=True)
    ros_thread.start()

    app = build_app(buf, topic=topic, title=title,
                    window=window, push_rate=push_rate)

    sub_node.get_logger().info(
        f"web dashboard at  http://{_local_ip_hint()}:{port}/  "
        f"(FastAPI/uvicorn bound on {host}:{port})")

    # Quiet uvicorn's own access log to keep the ROS console readable.
    log_config = uvicorn.config.LOGGING_CONFIG
    log_config["loggers"]["uvicorn.access"]["level"] = "WARNING"
    logging.getLogger("uvicorn.error").setLevel(logging.INFO)

    config = uvicorn.Config(app, host=host, port=port,
                            log_config=log_config, log_level="info",
                            access_log=False, lifespan="off")
    server = uvicorn.Server(config)

    try:
        server.run()  # blocks until SIGINT/SIGTERM
    finally:
        stop.set()
        ros_thread.join(timeout=1.0)
        try:
            sub_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
