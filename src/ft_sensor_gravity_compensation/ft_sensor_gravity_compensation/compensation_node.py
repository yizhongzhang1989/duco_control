"""ROS 2 node: subscribe to raw wrench + /tf, publish gravity-compensated wrench.

Also runs an embedded web dashboard so the user can:

* create / rename / delete named end-effector profiles,
* record the current ``(R_sensor_in_world, F_raw, T_raw)`` as a sample,
* run a least-squares fit over the recorded samples,
* manually edit mass / CoM / bias for an end-effector,
* activate any profile live -- the next outgoing wrench uses it immediately.

Topics
------
subscribes:
  ``input_topic``  geometry_msgs/WrenchStamped   raw F/T (default
                                                  /duco_ft_sensor/wrench_raw)
  /tf, /tf_static                                  TCP orientation (via tf2_ros)

publishes:
  ``output_topic`` geometry_msgs/WrenchStamped   compensated wrench (default
                                                  /duco_ft_sensor/wrench_compensated)

Parameters
----------
  input_topic       string  default "/duco_ft_sensor/wrench_raw"
  output_topic      string  default "/duco_ft_sensor/wrench_compensated"
  world_frame       string  default "base_link"
  sensor_frame      string  default "link_6"
  reliability       string  default "best_effort"
  publish_when_no_tf bool   default False  -- if True, publish raw-minus-bias
                                              with zero gravity when /tf is
                                              stale (useful for bench tests).
  storage_path      string  default "~/.ros/ft_sensor_gravity_compensation/"
                            "end_effectors.yaml"
  host              string  default "0.0.0.0"
  port              int     default 8100
  gravity           double  default 9.80665
"""

from __future__ import annotations

import json
import os
import threading
import time
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import Buffer, TransformException, TransformListener

from .calibration import (
    CompensationParams,
    DEFAULT_GRAVITY,
    calibrate_least_squares,
    compensate,
    quat_to_rotation,
)
from .ee_store import EndEffector, EndEffectorStore, Sample


# ===========================================================================
# Embedded HTML (served at "/")
# ===========================================================================
_HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>F/T gravity compensation</title>
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
  tr.selected { background: #1f2937; }
  button { background: #2563eb; color: white; border: 0; border-radius: 4px;
           padding: 4px 10px; font: inherit; cursor: pointer; font-size: 0.82rem; }
  button:hover { background: #1d4ed8; }
  button.danger { background: #b91c1c; }
  button.danger:hover { background: #991b1b; }
  button.muted { background: #374151; }
  button.muted:hover { background: #4b5563; }
  button.success { background: #15803d; }
  button.success:hover { background: #166534; }
  button:disabled { opacity: 0.5; cursor: not-allowed; }
  input[type=text], input[type=number], textarea {
    background: #11141a; color: #e6e6e6;
    border: 1px solid #2b3140; border-radius: 4px;
    padding: 4px 6px; font: inherit; font-size: 0.82rem;
  }
  input[type=number] { width: 90px; font-variant-numeric: tabular-nums; }
  input:focus, textarea:focus { outline: none; border-color: #4f7cff; }
  .status { display: inline-block; padding: 2px 7px; border-radius: 999px;
            background: #1f2937; color: #cfd3dc; font-variant-numeric: tabular-nums; }
  .status.ok { background: #14532d; color: #c6f7d6; }
  .status.bad { background: #5a1d1d; color: #ffb4b4; }
  .status.warn { background: #553e1a; color: #ffe6a8; }
  .grid3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; align-items: center; }
  .grid3 label { font-size: 0.78rem; color: #8a93a6; }
  .toolbar { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; margin-bottom: 8px; }
  .small { font-size: 0.78rem; color: #8a93a6; }
  .warn { color: #f0c46d; }
  .num { font-variant-numeric: tabular-nums; }
  .pill { display: inline-block; padding: 1px 6px; border-radius: 999px;
          background: #11141a; border: 1px solid #2b3140; font-size: 0.75rem;
          color: #cfd3dc; margin-right: 4px; }
  .pill.active { background: #14532d; color: #c6f7d6; border-color: #14532d; }
  .err { color: #ff7575; }
  .ok { color: #6fcf97; }
  ul.warns { margin: 4px 0 0; padding-left: 18px; }
  ul.warns li { color: #f0c46d; font-size: 0.78rem; }
</style>
</head>
<body>
  <h1>F/T gravity compensation</h1>
  <div class="sub">
    raw <code id="topic-in">--</code>
    -> compensated <code id="topic-out">--</code>
    &nbsp; world <code id="frame-w">--</code> -> sensor <code id="frame-s">--</code>
    &nbsp; <span id="conn" class="status">connecting...</span>
  </div>

  <div class="row">
    <!-- ============================================================= -->
    <div class="col">
      <div class="panel">
        <h2>Live wrench</h2>
        <table>
          <thead><tr><th></th><th>raw</th><th>compensated</th></tr></thead>
          <tbody>
            <tr><td>Fx [N]</td><td class="num" id="rfx">--</td><td class="num" id="cfx">--</td></tr>
            <tr><td>Fy [N]</td><td class="num" id="rfy">--</td><td class="num" id="cfy">--</td></tr>
            <tr><td>Fz [N]</td><td class="num" id="rfz">--</td><td class="num" id="cfz">--</td></tr>
            <tr><td>Mx [Nm]</td><td class="num" id="rmx">--</td><td class="num" id="cmx">--</td></tr>
            <tr><td>My [Nm]</td><td class="num" id="rmy">--</td><td class="num" id="cmy">--</td></tr>
            <tr><td>Mz [Nm]</td><td class="num" id="rmz">--</td><td class="num" id="cmz">--</td></tr>
          </tbody>
        </table>
        <div class="small" style="margin-top:8px;">
          raw rate <span id="raw-rate" class="status">-- Hz</span>
          &nbsp; tf <span id="tf-status" class="status">--</span>
          &nbsp; samples <span id="raw-recv" class="status">0</span>
        </div>
      </div>

      <div class="panel">
        <h2>End-effectors</h2>
        <div class="toolbar">
          <input id="new-name" type="text" placeholder="new name" maxlength="64" style="flex:1; min-width:120px;">
          <button id="btn-create" type="button">Add</button>
        </div>
        <table id="ee-table">
          <thead>
            <tr><th></th><th>name</th><th>mass [kg]</th><th>samples</th><th></th></tr>
          </thead>
          <tbody id="ee-tbody"></tbody>
        </table>
      </div>
    </div>

    <!-- ============================================================= -->
    <div class="col">
      <div class="panel">
        <h2>Editing: <span id="sel-name" class="num">--</span>
          <span id="sel-active" class="pill">inactive</span></h2>
        <div class="toolbar">
          <input id="sel-desc" type="text" placeholder="description" style="flex:1; min-width:160px;">
          <button id="btn-save-desc" type="button" class="muted">Save desc</button>
          <button id="btn-rename"   type="button" class="muted">Rename</button>
        </div>
      </div>

      <div class="row">
        <div class="panel" style="flex: 1 1 300px; min-width: 280px;">
          <h2 style="margin-top:0;">Calibration parameters
            <span class="small" id="cal-meta"></span>
          </h2>
          <div class="grid3">
            <label>mass [kg]</label>
            <label>gravity [m/s^2]</label>
            <span></span>
            <input id="p-mass"    type="number" step="0.0001" value="0">
            <input id="p-gravity" type="number" step="0.0001" value="9.80665">
            <span></span>

            <label>CoM x [m]</label><label>CoM y [m]</label><label>CoM z [m]</label>
            <input id="p-cx" type="number" step="0.0001" value="0">
            <input id="p-cy" type="number" step="0.0001" value="0">
            <input id="p-cz" type="number" step="0.0001" value="0">

            <label>F bias x [N]</label><label>F bias y [N]</label><label>F bias z [N]</label>
            <input id="p-fbx" type="number" step="0.001" value="0">
            <input id="p-fby" type="number" step="0.001" value="0">
            <input id="p-fbz" type="number" step="0.001" value="0">

            <label>T bias x [Nm]</label><label>T bias y [Nm]</label><label>T bias z [Nm]</label>
            <input id="p-tbx" type="number" step="0.0001" value="0">
            <input id="p-tby" type="number" step="0.0001" value="0">
            <input id="p-tbz" type="number" step="0.0001" value="0">
          </div>
          <div class="toolbar" style="margin-top:8px;">
            <button id="btn-save-params" type="button">Save parameters</button>
            <button id="btn-zero-params" type="button" class="muted">Reset to zero</button>
          </div>
          <ul class="warns" id="cal-warns"></ul>
        </div>

        <div class="panel" style="flex: 1 1 300px; min-width: 280px;">
          <h2 style="margin-top:0;">Sample orientation distribution
            <span class="small" id="sphere-meta"></span>
          </h2>
          <canvas id="sphere-canvas" style="width:100%; height:280px; display:block;
                  background:#0c0e13; border-radius:6px; cursor:grab;"></canvas>
          <div class="small" style="margin-top:6px;">
            Each dot is one sample's <b>gravity direction in the sensor frame</b>
            (<code>g_S = R&#8348; &middot; [0,0,-1]</code>); the yellow ring is the
            live pose. Drag to rotate, double-click to reset.
            <span style="color:#ef4444;">x_S</span>
            <span style="color:#22c55e;">y_S</span>
            <span style="color:#3b82f6;">z_S</span>
            are the sensor-frame axes.
            <br>
            <b>For a good calibration</b> the dots must span the sphere in 3D, not
            lie on a circle. Rotating joint 6 alone only sweeps a horizontal
            circle (mass identifiable, CoM <i>not</i>); vary <b>joints 4 / 5</b>
            to change latitude. Aim for 6+ samples spread over front and back of
            the sphere &middot; spread score above ~0.4 is healthy.
          </div>
        </div>
      </div>

      <div class="panel">
        <h2>Samples
          <span class="small" id="samples-count">0</span>
        </h2>
        <div class="toolbar">
          <input id="sample-note" type="text" placeholder="optional note" style="flex:1; min-width:120px;">
          <button id="btn-record" type="button" class="success">Record current</button>
          <button id="btn-calibrate" type="button">Calibrate</button>
          <button id="btn-clear-samples" type="button" class="danger">Clear samples</button>
        </div>
        <table>
          <thead>
            <tr><th>#</th><th>g_in_sensor</th><th>F_raw</th><th>T_raw</th><th>note</th><th></th></tr>
          </thead>
          <tbody id="samples-tbody"></tbody>
        </table>
      </div>
    </div>
  </div>

<script>
(() => {
  const $ = (id) => document.getElementById(id);
  const fmt = (v, n = 2) => (typeof v === "number" && isFinite(v))
    ? v.toFixed(n) : "--";
  let state = null;
  let selected = null;          // currently-selected end-effector name
  let lastRecord = null;        // {warnings:[], rms_force, rms_torque, ...}
  let liveG = null;             // live gravity-direction in sensor frame [x,y,z]
  let sphereSamples = [];       // [[gx,gy,gz], ...] for the *selected* end-effector

  // ===== 3D orientation sphere ============================================
  // A self-contained min-3D viewer: orthographic projection, mouse-drag
  // trackball, latitude/longitude wireframe, axis arrows, depth-sorted
  // sample dots, plus a hollow ring for the live pose. Pure 2D canvas.
  const Sphere = (() => {
    const cv = $("sphere-canvas");
    let az = 0.6, el = 0.4;
    let dragging = false, lastX = 0, lastY = 0;

    cv.addEventListener("mousedown", (e) => {
      dragging = true; lastX = e.clientX; lastY = e.clientY;
      cv.style.cursor = "grabbing";
    });
    window.addEventListener("mouseup", () => {
      dragging = false; cv.style.cursor = "grab";
    });
    window.addEventListener("mousemove", (e) => {
      if (!dragging) return;
      az += (e.clientX - lastX) * 0.01;
      el += (e.clientY - lastY) * 0.01;
      el = Math.max(-Math.PI / 2 + 0.05, Math.min(Math.PI / 2 - 0.05, el));
      lastX = e.clientX; lastY = e.clientY;
      draw();
    });
    cv.addEventListener("dblclick", () => { az = 0.6; el = 0.4; draw(); });

    function camera() {
      // World-to-camera = Rx(el) @ Ry(az). After this transform:
      //   x_cam = right   y_cam = up   z_cam = toward viewer
      const ca = Math.cos(az), sa = Math.sin(az);
      const ce = Math.cos(el), se = Math.sin(el);
      return [
        [ca,        0.0,    sa     ],
        [se * sa,   ce,    -se * ca],
        [-ce * sa,  se,     ce * ca],
      ];
    }
    function project(M, v) {
      return [
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2],
      ];
    }

    function fitCanvas() {
      const dpr = window.devicePixelRatio || 1;
      const r = cv.getBoundingClientRect();
      cv.width  = Math.max(1, Math.round(r.width  * dpr));
      cv.height = Math.max(1, Math.round(r.height * dpr));
      return dpr;
    }

    function draw() {
      const dpr = fitCanvas();
      const ctx = cv.getContext("2d");
      const W = cv.width, H = cv.height;
      ctx.fillStyle = "#0c0e13"; ctx.fillRect(0, 0, W, H);

      const cx = W / 2, cy = H / 2;
      const R = Math.min(W, H) * 0.42;
      const M = camera();
      const X = (p) => cx + p[0] * R;
      const Y = (p) => cy - p[1] * R;

      // Outline circle (silhouette of the unit sphere under orthographic proj).
      ctx.strokeStyle = "#2b3140";
      ctx.lineWidth = 1 * dpr;
      ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.stroke();

      // Lat/lon wireframe. Hide back-facing segments by clipping where z < 0.
      const drawArc = (pointsFn, segments, color, dashed) => {
        ctx.strokeStyle = color;
        ctx.setLineDash(dashed ? [3 * dpr, 4 * dpr] : []);
        ctx.beginPath();
        let started = false, prevBack = true;
        for (let i = 0; i <= segments; i++) {
          const v = pointsFn(i / segments);
          const p = project(M, v);
          const back = (p[2] < 0);
          if (back !== prevBack) {
            // segment crossed the silhouette: start a new sub-path so we
            // don't connect across the back of the sphere.
            started = false;
          }
          prevBack = back;
          if (back && !dashed) continue;        // hide back-of-sphere unless explicitly dashed
          if (!started) { ctx.moveTo(X(p), Y(p)); started = true; }
          else          { ctx.lineTo(X(p), Y(p)); }
        }
        ctx.stroke();
        ctx.setLineDash([]);
      };

      // 7 latitudes (skip poles)
      for (let k = 1; k <= 7; k++) {
        const lat = -Math.PI / 2 + k * Math.PI / 8;
        const cl = Math.cos(lat), sl = Math.sin(lat);
        drawArc((u) => {
          const lon = u * 2 * Math.PI;
          return [cl * Math.cos(lon), sl, cl * Math.sin(lon)];
        }, 64, "#1f2937");
      }
      // 8 longitudes
      for (let k = 0; k < 8; k++) {
        const lon = k * Math.PI / 4;
        const cl = Math.cos(lon), sl = Math.sin(lon);
        drawArc((u) => {
          const lat = -Math.PI / 2 + u * Math.PI;
          const c = Math.cos(lat), s = Math.sin(lat);
          return [c * cl, s, c * sl];
        }, 64, "#1f2937");
      }

      // Equator emphasised.
      drawArc((u) => {
        const lon = u * 2 * Math.PI;
        return [Math.cos(lon), 0, Math.sin(lon)];
      }, 96, "#374151");

      // Axes (sensor-frame x/y/z) drawn as small arrows from origin.
      const axes = [
        { dir: [1, 0, 0], color: "#ef4444", label: "x_S" },
        { dir: [0, 1, 0], color: "#22c55e", label: "y_S" },
        { dir: [0, 0, 1], color: "#3b82f6", label: "z_S" },
      ];
      for (const a of axes) {
        const tip = project(M, a.dir);
        const back = tip[2] < 0;
        const o = project(M, [0, 0, 0]);
        ctx.strokeStyle = a.color;
        ctx.globalAlpha = back ? 0.35 : 1.0;
        ctx.lineWidth = 1.5 * dpr;
        ctx.beginPath();
        ctx.moveTo(X(o), Y(o));
        ctx.lineTo(X(tip), Y(tip));
        ctx.stroke();
        ctx.fillStyle = a.color;
        ctx.beginPath(); ctx.arc(X(tip), Y(tip), 2.5 * dpr, 0, 2 * Math.PI); ctx.fill();
        ctx.font = (10 * dpr) + "px ui-sans-serif, sans-serif";
        ctx.fillText(a.label, X(tip) + 4 * dpr, Y(tip) - 4 * dpr);
        ctx.globalAlpha = 1.0;
      }

      // Sample points: project, depth-sort, then draw with size + alpha
      // depth-fading so back-sphere points read as further away.
      const pts = sphereSamples.map((g) => {
        const p = project(M, g);
        return { g, p };
      });
      pts.sort((a, b) => a.p[2] - b.p[2]);
      for (const { p } of pts) {
        const back = p[2] < 0;
        ctx.globalAlpha = back ? 0.35 : 1.0;
        ctx.fillStyle = back ? "#7dd3fc" : "#38bdf8";
        const r = (back ? 3.0 : 4.5) * dpr;
        ctx.beginPath(); ctx.arc(X(p), Y(p), r, 0, 2 * Math.PI); ctx.fill();
      }
      ctx.globalAlpha = 1.0;

      // Live pose: hollow ring (so it's visible even on top of a sample).
      if (liveG) {
        const lg = (() => {
          const n = Math.hypot(liveG[0], liveG[1], liveG[2]);
          if (n < 1e-6) return null;
          return [liveG[0] / n, liveG[1] / n, liveG[2] / n];
        })();
        if (lg) {
          const p = project(M, lg);
          const back = p[2] < 0;
          ctx.globalAlpha = back ? 0.5 : 1.0;
          ctx.strokeStyle = "#facc15";
          ctx.lineWidth = 2 * dpr;
          ctx.beginPath(); ctx.arc(X(p), Y(p), 7 * dpr, 0, 2 * Math.PI); ctx.stroke();
          ctx.fillStyle = "#facc15";
          ctx.beginPath(); ctx.arc(X(p), Y(p), 1.5 * dpr, 0, 2 * Math.PI); ctx.fill();
          ctx.globalAlpha = 1.0;
        }
      }

      // Coverage diagnostic: show the smallest singular value of the
      // matrix whose rows are the sample gravity directions. If <0.4 the
      // distribution is borderline; <0.15 the LSQ for mass/CoM is poorly
      // conditioned. (Cheap 3-vector covariance eigen lower bound.)
      $("sphere-meta").textContent = sphereSamples.length
        ? "(" + sphereSamples.length + " samples, "
          + "spread " + coverageScore(sphereSamples).toFixed(2) + ")"
        : "";
    }

    // 0..1 score: 1.0 = ideal isotropic distribution, 0.0 = all samples
    // collinear. Computed as 3 * lambda_min(C) where C is the sample
    // covariance of unit vectors, since trace(C)=1 for unit vectors and
    // an isotropic distribution has lambda_min = 1/3.
    function coverageScore(pts) {
      if (pts.length < 2) return 0.0;
      let cxx = 0, cyy = 0, czz = 0, cxy = 0, cxz = 0, cyz = 0;
      for (const v of pts) {
        const n = Math.hypot(v[0], v[1], v[2]) || 1;
        const x = v[0]/n, y = v[1]/n, z = v[2]/n;
        cxx += x*x; cyy += y*y; czz += z*z;
        cxy += x*y; cxz += x*z; cyz += y*z;
      }
      const N = pts.length;
      const C = [
        [cxx/N, cxy/N, cxz/N],
        [cxy/N, cyy/N, cyz/N],
        [cxz/N, cyz/N, czz/N],
      ];
      // Smallest eigenvalue via the trace/determinant formula (3x3 sym).
      const tr = C[0][0] + C[1][1] + C[2][2];
      const det = C[0][0]*(C[1][1]*C[2][2] - C[1][2]*C[2][1])
                - C[0][1]*(C[1][0]*C[2][2] - C[1][2]*C[2][0])
                + C[0][2]*(C[1][0]*C[2][1] - C[1][1]*C[2][0]);
      // p2 = sum of 2x2 principal minors
      const p2 = C[0][0]*C[1][1] + C[0][0]*C[2][2] + C[1][1]*C[2][2]
               - C[0][1]*C[0][1] - C[0][2]*C[0][2] - C[1][2]*C[1][2];
      // Solve l^3 - tr*l^2 + p2*l - det = 0 by Cardano via trig.
      const a = -tr, b = p2, c = -det;
      const Q = (3*b - a*a) / 9;
      const Rr = (9*a*b - 27*c - 2*a*a*a) / 54;
      const disc = Q*Q*Q + Rr*Rr;
      let lambdas;
      if (disc <= 0) {
        const m = Math.sqrt(-Q);
        const th = Math.acos(Math.max(-1, Math.min(1, Rr / (m*m*m))));
        const k = 2 * m;
        lambdas = [
          k * Math.cos(th/3)               - a/3,
          k * Math.cos((th + 2*Math.PI)/3) - a/3,
          k * Math.cos((th + 4*Math.PI)/3) - a/3,
        ];
      } else {
        // Numerical fallback (rare for sym matrices); approximate via QR.
        lambdas = [tr/3, tr/3, tr/3];
      }
      const lmin = Math.max(0, Math.min(...lambdas));
      return Math.max(0, Math.min(1, 3 * lmin));
    }

    return { draw, fitCanvas };
  })();

  window.addEventListener("resize", () => Sphere.draw());

  function setConn(state, msg) {
    const el = $("conn");
    el.textContent = msg;
    el.classList.remove("ok", "bad", "warn");
    if (state) el.classList.add(state);
  }

  function setStatus(el, cls, txt) {
    el.textContent = txt;
    el.classList.remove("ok", "bad", "warn");
    if (cls) el.classList.add(cls);
  }

  async function api(path, opts) {
    const res = await fetch(path, opts);
    let body = null;
    try { body = await res.json(); } catch { /* */ }
    if (!res.ok) {
      const msg = (body && body.error) || (res.status + " " + res.statusText);
      throw new Error(msg);
    }
    return body || {};
  }

  function renderInfo(s) {
    $("topic-in").textContent  = s.input_topic;
    $("topic-out").textContent = s.output_topic;
    $("frame-w").textContent   = s.world_frame;
    $("frame-s").textContent   = s.sensor_frame;
  }

  function renderLive(s) {
    const r = s.raw   || {};
    const c = s.compensated || {};
    $("rfx").textContent = fmt(r.fx, 2);
    $("rfy").textContent = fmt(r.fy, 2);
    $("rfz").textContent = fmt(r.fz, 2);
    $("rmx").textContent = fmt(r.tx, 3);
    $("rmy").textContent = fmt(r.ty, 3);
    $("rmz").textContent = fmt(r.tz, 3);
    $("cfx").textContent = fmt(c.fx, 2);
    $("cfy").textContent = fmt(c.fy, 2);
    $("cfz").textContent = fmt(c.fz, 2);
    $("cmx").textContent = fmt(c.tx, 3);
    $("cmy").textContent = fmt(c.ty, 3);
    $("cmz").textContent = fmt(c.tz, 3);

    setStatus($("raw-rate"),
      (s.raw_rate_hz > 0) ? "ok" : "bad",
      (s.raw_rate_hz != null) ? fmt(s.raw_rate_hz, 1) + " Hz" : "no data");
    setStatus($("raw-recv"), null, "" + (s.raw_received || 0));
    setStatus($("tf-status"),
      s.tf_ok ? "ok" : "bad",
      s.tf_ok ? "ok (" + fmt(s.tf_age, 2) + " s)" : (s.tf_error || "no transform"));

    liveG = (s.tf_ok && Array.isArray(s.live_g)) ? s.live_g : null;
    Sphere.draw();
  }

  function renderEEList(s) {
    const tbody = $("ee-tbody");
    tbody.innerHTML = "";
    const names = Object.keys(s.end_effectors).sort();
    for (const name of names) {
      const ee = s.end_effectors[name];
      const tr = document.createElement("tr");
      if (name === selected) tr.classList.add("selected");
      tr.dataset.name = name;
      tr.innerHTML = "" +
        '<td>' + (name === s.active ? '<span class="pill active">active</span>' : '') + '</td>' +
        '<td><b>' + escapeHtml(name) + '</b><div class="small">' + escapeHtml(ee.description || "") + '</div></td>' +
        '<td class="num">' + fmt(ee.mass, 3) + '</td>' +
        '<td class="num">' + (ee.samples ? ee.samples.length : 0) + '</td>' +
        '<td>' +
          (name === s.active ? '' : '<button class="success" data-action="activate">Activate</button>') +
          (name === "default" ? '' : ' <button class="danger" data-action="delete">Delete</button>') +
        '</td>';
      tr.addEventListener("click", (ev) => {
        if (ev.target.tagName === "BUTTON") return;
        selected = name;
        renderEditor(s);
        renderEEList(s);
      });
      tr.querySelectorAll("button[data-action]").forEach((btn) => {
        btn.addEventListener("click", async (ev) => {
          ev.stopPropagation();
          const action = btn.dataset.action;
          try {
            if (action === "activate") {
              await api("/api/activate?name=" + encodeURIComponent(name), { method: "POST" });
            } else if (action === "delete") {
              if (!confirm("Delete '" + name + "'?")) return;
              await api("/api/delete?name=" + encodeURIComponent(name), { method: "POST" });
              if (selected === name) selected = s.active;
            }
            await refresh();
          } catch (e) { alert(e.message); }
        });
      });
      tbody.appendChild(tr);
    }
    if (selected == null || !(selected in s.end_effectors)) {
      selected = s.active;
    }
  }

  function renderEditor(s) {
    const ee = s.end_effectors[selected];
    if (!ee) return;
    $("sel-name").textContent = selected;
    const isActive = (selected === s.active);
    const pill = $("sel-active");
    pill.textContent = isActive ? "active" : "inactive";
    pill.classList.toggle("active", isActive);
    $("sel-desc").value = ee.description || "";
    $("p-mass").value    = numStr(ee.mass);
    $("p-gravity").value = numStr(ee.gravity);
    $("p-cx").value = numStr(ee.com[0]); $("p-cy").value = numStr(ee.com[1]); $("p-cz").value = numStr(ee.com[2]);
    $("p-fbx").value = numStr(ee.force_bias[0]); $("p-fby").value = numStr(ee.force_bias[1]); $("p-fbz").value = numStr(ee.force_bias[2]);
    $("p-tbx").value = numStr(ee.torque_bias[0]); $("p-tby").value = numStr(ee.torque_bias[1]); $("p-tbz").value = numStr(ee.torque_bias[2]);

    const meta = $("cal-meta");
    if (ee.last_calibration) {
      const lc = ee.last_calibration;
      const ago = lc.timestamp ? Math.round(Date.now()/1000 - lc.timestamp) : null;
      meta.textContent = " (last fit: " + (lc.n_samples || 0) + " samples"
        + ", rms F=" + fmt(lc.rms_force, 3) + " N"
        + ", rms T=" + fmt(lc.rms_torque, 4) + " Nm"
        + (ago != null ? ", " + ago + " s ago" : "") + ")";
    } else {
      meta.textContent = "";
    }
    const warns = $("cal-warns");
    warns.innerHTML = "";
    const w = (lastRecord && lastRecord.name === selected) ? lastRecord.warnings : [];
    if (ee.last_calibration && ee.last_calibration.warnings) {
      for (const m of ee.last_calibration.warnings) w.push(m);
    }
    for (const m of w) {
      const li = document.createElement("li");
      li.textContent = m;
      warns.appendChild(li);
    }

    const sb = $("samples-tbody");
    sb.innerHTML = "";
    const samples = ee.samples || [];
    $("samples-count").textContent = "(" + samples.length + ")";
    sphereSamples = samples.map((sm) => {
      const r = sm.rotation;
      // gravity-direction unit vector in sensor frame: g_S = R^T @ [0,0,-1] = -R[2,:]
      return [-r[2][0], -r[2][1], -r[2][2]];
    });
    Sphere.draw();
    samples.forEach((sm, idx) => {
      // gravity-direction in sensor frame: g_S = R^T @ [0,0,-g]
      // we display the unit vector for orientation diversity
      const r = sm.rotation;
      const g = [-r[2][0], -r[2][1], -r[2][2]];   // = R^T @ [0,0,-1]
      const tr = document.createElement("tr");
      tr.innerHTML =
        '<td>' + idx + '</td>' +
        '<td class="num">[' + fmt(g[0], 2) + ', ' + fmt(g[1], 2) + ', ' + fmt(g[2], 2) + ']</td>' +
        '<td class="num">[' + fmt(sm.force[0], 2) + ', ' + fmt(sm.force[1], 2) + ', ' + fmt(sm.force[2], 2) + ']</td>' +
        '<td class="num">[' + fmt(sm.torque[0], 3) + ', ' + fmt(sm.torque[1], 3) + ', ' + fmt(sm.torque[2], 3) + ']</td>' +
        '<td>' + escapeHtml(sm.note || "") + '</td>' +
        '<td><button class="danger" data-idx="' + idx + '">x</button></td>';
      tr.querySelector("button").addEventListener("click", async () => {
        try {
          await api("/api/sample/delete?name=" + encodeURIComponent(selected) + "&index=" + idx, { method: "POST" });
          await refresh();
        } catch (e) { alert(e.message); }
      });
      sb.appendChild(tr);
    });
  }

  function numStr(v) {
    if (typeof v !== "number" || !isFinite(v)) return "0";
    return v.toFixed(6).replace(/0+$/, "").replace(/\.$/, "");
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, (c) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    })[c]);
  }

  async function refresh() {
    try {
      state = await api("/api/state");
      renderInfo(state);
      renderLive(state);
      renderEEList(state);
      renderEditor(state);
      setConn("ok", "live");
    } catch (e) {
      setConn("bad", "disconnected: " + e.message);
    }
  }

  async function tick() {
    try {
      const s = await api("/api/live");
      renderLive(s);
      setConn("ok", "live");
    } catch (e) {
      setConn("bad", "disconnected: " + e.message);
    }
  }

  // wire buttons
  $("btn-create").addEventListener("click", async () => {
    const name = $("new-name").value.trim();
    if (!name) return;
    try {
      await api("/api/create?name=" + encodeURIComponent(name), { method: "POST" });
      $("new-name").value = "";
      selected = name;
      await refresh();
    } catch (e) { alert(e.message); }
  });

  $("btn-save-desc").addEventListener("click", async () => {
    if (!selected) return;
    try {
      await api("/api/describe?name=" + encodeURIComponent(selected) +
                "&description=" + encodeURIComponent($("sel-desc").value),
                { method: "POST" });
      await refresh();
    } catch (e) { alert(e.message); }
  });

  $("btn-rename").addEventListener("click", async () => {
    if (!selected) return;
    const next = prompt("New name for '" + selected + "':", selected);
    if (!next || next === selected) return;
    try {
      await api("/api/rename?old_name=" + encodeURIComponent(selected) +
                "&new_name=" + encodeURIComponent(next),
                { method: "POST" });
      selected = next;
      await refresh();
    } catch (e) { alert(e.message); }
  });

  function readParams() {
    return {
      mass: parseFloat($("p-mass").value),
      gravity: parseFloat($("p-gravity").value),
      com: [parseFloat($("p-cx").value), parseFloat($("p-cy").value), parseFloat($("p-cz").value)],
      force_bias: [parseFloat($("p-fbx").value), parseFloat($("p-fby").value), parseFloat($("p-fbz").value)],
      torque_bias: [parseFloat($("p-tbx").value), parseFloat($("p-tby").value), parseFloat($("p-tbz").value)],
    };
  }

  $("btn-save-params").addEventListener("click", async () => {
    if (!selected) return;
    try {
      await api("/api/params?name=" + encodeURIComponent(selected),
                {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify(readParams()),
                });
      await refresh();
    } catch (e) { alert(e.message); }
  });

  $("btn-zero-params").addEventListener("click", async () => {
    if (!selected) return;
    try {
      await api("/api/params?name=" + encodeURIComponent(selected),
                {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({
                    mass: 0, gravity: 9.80665,
                    com: [0, 0, 0], force_bias: [0, 0, 0], torque_bias: [0, 0, 0],
                  }),
                });
      await refresh();
    } catch (e) { alert(e.message); }
  });

  $("btn-record").addEventListener("click", async () => {
    if (!selected) return;
    try {
      await api("/api/sample/record?name=" + encodeURIComponent(selected) +
                "&note=" + encodeURIComponent($("sample-note").value),
                { method: "POST" });
      $("sample-note").value = "";
      await refresh();
    } catch (e) { alert(e.message); }
  });

  $("btn-calibrate").addEventListener("click", async () => {
    if (!selected) return;
    try {
      const r = await api("/api/calibrate?name=" + encodeURIComponent(selected),
                          { method: "POST" });
      lastRecord = { name: selected, warnings: r.warnings || [] };
      await refresh();
    } catch (e) { alert(e.message); }
  });

  $("btn-clear-samples").addEventListener("click", async () => {
    if (!selected) return;
    if (!confirm("Clear all samples for '" + selected + "'?")) return;
    try {
      await api("/api/sample/clear?name=" + encodeURIComponent(selected),
                { method: "POST" });
      await refresh();
    } catch (e) { alert(e.message); }
  });

  refresh().then(() => setInterval(tick, 200));
  setInterval(refresh, 3000);   // periodic full refresh to pick up table changes
})();
</script>
</body>
</html>
"""


# ===========================================================================
# HTTP request handler -- methods talk to a CompensationNode via partial()
# ===========================================================================
class _DashboardHandler(BaseHTTPRequestHandler):
    """Routes JSON API + serves the static HTML."""

    def __init__(self, *args, dashboard=None, **kwargs):
        self._dashboard: "CompensationNode" = dashboard
        super().__init__(*args, **kwargs)

    # silence the per-request stderr access log (the ROS log already records launches)
    def log_message(self, _format, *args):  # noqa: D401, N802
        return

    # ---- response helpers --------------------------------------------------
    def _send_json(self, status: int, payload: dict) -> None:
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

    def _read_json_body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8") or "{}")
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid JSON body: {exc}") from exc

    def _query(self) -> dict:
        return {k: v[0] for k, v in parse_qs(urlparse(self.path).query).items()}

    # ---- routes ------------------------------------------------------------
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
            self.send_response(404)
            self.end_headers()
        except Exception as exc:  # noqa: BLE001
            self._send_json(500, {"error": f"{type(exc).__name__}: {exc}"})

    def do_POST(self):  # noqa: N802
        path = urlparse(self.path).path
        try:
            q = self._query()
            if path == "/api/create":
                self._dashboard.api_create(q.get("name", ""),
                                           q.get("description", ""))
            elif path == "/api/delete":
                self._dashboard.api_delete(q.get("name", ""))
            elif path == "/api/rename":
                self._dashboard.api_rename(q.get("old_name", ""),
                                           q.get("new_name", ""))
            elif path == "/api/describe":
                self._dashboard.api_describe(q.get("name", ""),
                                             q.get("description", ""))
            elif path == "/api/activate":
                self._dashboard.api_activate(q.get("name", ""))
            elif path == "/api/params":
                body = self._read_json_body()
                self._dashboard.api_set_params(q.get("name", ""), body)
            elif path == "/api/sample/record":
                self._dashboard.api_record_sample(q.get("name", ""),
                                                  q.get("note", ""))
            elif path == "/api/sample/delete":
                idx = int(q.get("index", "0"))
                self._dashboard.api_delete_sample(q.get("name", ""), idx)
            elif path == "/api/sample/clear":
                self._dashboard.api_clear_samples(q.get("name", ""))
            elif path == "/api/calibrate":
                result = self._dashboard.api_calibrate(q.get("name", ""))
                self._send_json(200, result)
                return
            else:
                self.send_response(404)
                self.end_headers()
                return
            self._send_json(200, {"ok": True})
        except (ValueError, KeyError, IndexError) as exc:
            self._send_json(400, {"error": f"{type(exc).__name__}: {exc}"})
        except Exception as exc:  # noqa: BLE001
            self._send_json(500, {"error": f"{type(exc).__name__}: {exc}"})


# ===========================================================================
# the ROS node
# ===========================================================================
class CompensationNode(Node):

    def __init__(self) -> None:
        super().__init__("ft_sensor_gravity_compensation")

        # --- parameters ---------------------------------------------------
        self.declare_parameter("input_topic",  "/duco_ft_sensor/wrench_raw")
        self.declare_parameter("output_topic", "/duco_ft_sensor/wrench_compensated")
        self.declare_parameter("world_frame",  "base_link")
        self.declare_parameter("sensor_frame", "link_6")
        self.declare_parameter("reliability",  "best_effort")
        self.declare_parameter("publish_when_no_tf", False)
        self.declare_parameter(
            "storage_path",
            "~/.ros/ft_sensor_gravity_compensation/end_effectors.yaml")
        self.declare_parameter("enable_dashboard", False)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8100)
        self.declare_parameter("gravity", DEFAULT_GRAVITY)
        self.declare_parameter("tf_timeout", 0.05)        # sec, per lookup
        self.declare_parameter("tf_max_age", 1.0)         # sec, before tf considered stale

        self._input_topic  = self.get_parameter("input_topic").get_parameter_value().string_value
        self._output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._world_frame  = self.get_parameter("world_frame").get_parameter_value().string_value
        self._sensor_frame = self.get_parameter("sensor_frame").get_parameter_value().string_value
        reliability        = self.get_parameter("reliability").get_parameter_value().string_value
        self._publish_when_no_tf = self.get_parameter("publish_when_no_tf").get_parameter_value().bool_value
        storage_path       = self.get_parameter("storage_path").get_parameter_value().string_value
        enable_dashboard   = self.get_parameter("enable_dashboard").get_parameter_value().bool_value
        host               = self.get_parameter("host").get_parameter_value().string_value
        port               = self.get_parameter("port").get_parameter_value().integer_value
        self._gravity      = self.get_parameter("gravity").get_parameter_value().double_value
        self._tf_timeout   = self.get_parameter("tf_timeout").get_parameter_value().double_value
        self._tf_max_age   = self.get_parameter("tf_max_age").get_parameter_value().double_value

        # --- end-effector store ------------------------------------------
        self._store = EndEffectorStore(storage_path)
        self._lock = threading.RLock()

        # --- TF listener --------------------------------------------------
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- live state (protected by self._lock) ------------------------
        self._raw_received: int = 0
        self._raw_rate_bucket_start = time.monotonic()
        self._raw_rate_bucket_count = 0
        self._raw_rate_hz: Optional[float] = None
        self._last_raw: Optional[Tuple[float, np.ndarray, np.ndarray]] = None
        self._last_compensated: Optional[Tuple[float, np.ndarray, np.ndarray]] = None
        self._last_R: Optional[np.ndarray] = None
        self._last_tf_stamp_mono: Optional[float] = None
        self._last_tf_error: str = "no transform yet"

        # --- ROS pub / sub ------------------------------------------------
        rel = (ReliabilityPolicy.BEST_EFFORT
               if reliability.lower().startswith("best")
               else ReliabilityPolicy.RELIABLE)
        wrench_qos = QoSProfile(
            reliability=rel,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )

        self._pub = self.create_publisher(WrenchStamped, self._output_topic, wrench_qos)
        self._sub = self.create_subscription(
            WrenchStamped, self._input_topic, self._on_wrench, wrench_qos)

        # --- HTTP dashboard (optional) -----------------------------------
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None
        if enable_dashboard:
            handler = partial(_DashboardHandler, dashboard=self)
            self._httpd = ThreadingHTTPServer((host, port), handler)
            self._http_thread = threading.Thread(
                target=self._httpd.serve_forever, daemon=True)
            self._http_thread.start()

        self.get_logger().info(
            f"subscribed: {self._input_topic} (reliability={rel.name})")
        self.get_logger().info(
            f"publishing: {self._output_topic}")
        self.get_logger().info(
            f"frames: world={self._world_frame!r}, sensor={self._sensor_frame!r}")
        self.get_logger().info(
            f"end-effector store: {self._store.path}")
        self.get_logger().info(
            f"active end-effector: {self._store.active_name!r}")
        if enable_dashboard:
            self.get_logger().info(
                f"web dashboard: http://{host if host != '0.0.0.0' else _local_ip()}:{port}/")
        else:
            self.get_logger().info(
                "web dashboard: disabled (set enable_dashboard:=true to launch it)")

    def destroy_node(self):  # noqa: D401
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _on_wrench(self, msg: WrenchStamped) -> None:
        f_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        t_raw = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        now_mono = time.monotonic()

        # update arrival-rate counter (1 s tumbling bucket)
        with self._lock:
            self._raw_received += 1
            self._raw_rate_bucket_count += 1
            elapsed = now_mono - self._raw_rate_bucket_start
            if elapsed >= 1.0:
                self._raw_rate_hz = self._raw_rate_bucket_count / elapsed
                self._raw_rate_bucket_start = now_mono
                self._raw_rate_bucket_count = 0

        # look up the sensor orientation at the wrench's timestamp; fall back
        # to "latest available" if the buffer doesn't have that exact time yet.
        R, tf_age, tf_error = self._lookup_rotation(msg.header.stamp)

        params = self._store.active().params
        if R is not None:
            f_comp, t_comp = compensate(f_raw, t_raw, R, params)
        elif self._publish_when_no_tf:
            # no orientation: subtract bias only
            f_comp = f_raw - params.force_bias
            t_comp = t_raw - params.torque_bias
        else:
            f_comp = None
            t_comp = None

        with self._lock:
            self._last_raw = (now_mono, f_raw, t_raw)
            self._last_R = R
            if R is not None:
                self._last_tf_stamp_mono = now_mono
                self._last_tf_error = ""
            else:
                self._last_tf_error = tf_error or "no transform"
            if f_comp is not None:
                self._last_compensated = (now_mono, f_comp, t_comp)

        if f_comp is None:
            return  # cannot publish without a valid TF (and not configured to)

        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.x  = float(f_comp[0])
        out.wrench.force.y  = float(f_comp[1])
        out.wrench.force.z  = float(f_comp[2])
        out.wrench.torque.x = float(t_comp[0])
        out.wrench.torque.y = float(t_comp[1])
        out.wrench.torque.z = float(t_comp[2])
        self._pub.publish(out)

    def _lookup_rotation(self, stamp) -> Tuple[Optional[np.ndarray], Optional[float], Optional[str]]:
        """Return ``(R_sensor_in_world, age_seconds, error_msg)`` for the given header stamp.

        ``R`` is None when no usable transform is available. ``age_seconds``
        is the difference between ``now`` and the transform's timestamp
        (informational only).
        """
        # Try the exact stamp first; if not yet in buffer, fall back to "latest".
        timeout = Duration(seconds=max(self._tf_timeout, 0.0))
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._world_frame, self._sensor_frame, stamp, timeout=timeout)
        except TransformException:
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    self._world_frame, self._sensor_frame, rclpy.time.Time())
            except TransformException as exc:
                return None, None, str(exc)

        # quaternion -> rotation matrix
        q = tf_msg.transform.rotation
        R = quat_to_rotation((q.x, q.y, q.z, q.w))

        # age check (best effort)
        try:
            tf_t = tf_msg.header.stamp.sec + tf_msg.header.stamp.nanosec * 1e-9
            now_t = self.get_clock().now().nanoseconds * 1e-9
            age = max(0.0, now_t - tf_t)
        except Exception:  # noqa: BLE001
            age = None

        if age is not None and age > self._tf_max_age:
            return None, age, f"latest transform is {age:.2f} s old"
        return R, age, None

    # ------------------------------------------------------------------
    # HTTP-side API (called from the HTTP request handler thread(s))
    # ------------------------------------------------------------------
    def api_state_full(self) -> dict:
        with self._lock:
            live = self._snapshot_live_locked()
            store = self._store.snapshot()
        store["input_topic"] = self._input_topic
        store["output_topic"] = self._output_topic
        store["world_frame"] = self._world_frame
        store["sensor_frame"] = self._sensor_frame
        store.update(live)
        return store

    def api_state_live(self) -> dict:
        with self._lock:
            live = self._snapshot_live_locked()
        live["active"] = self._store.active_name
        return live

    def _snapshot_live_locked(self) -> dict:
        now = time.monotonic()
        raw = None
        comp = None
        if self._last_raw is not None:
            t, f, tq = self._last_raw
            raw = {"fx": float(f[0]), "fy": float(f[1]), "fz": float(f[2]),
                   "tx": float(tq[0]), "ty": float(tq[1]), "tz": float(tq[2]),
                   "age": now - t}
        if self._last_compensated is not None:
            t, f, tq = self._last_compensated
            comp = {"fx": float(f[0]), "fy": float(f[1]), "fz": float(f[2]),
                    "tx": float(tq[0]), "ty": float(tq[1]), "tz": float(tq[2]),
                    "age": now - t}
        tf_age = (now - self._last_tf_stamp_mono) if self._last_tf_stamp_mono is not None else None
        # Unit gravity direction in the sensor frame:
        #   g_dir_S = R^T @ [0,0,-1] = -R[2,:]
        # (R is sensor-in-world). The dashboard plots this on a unit sphere
        # so the operator can see how well the recorded samples cover the
        # orientation space.
        live_g = None
        if self._last_R is not None:
            R = self._last_R
            live_g = [float(-R[2, 0]), float(-R[2, 1]), float(-R[2, 2])]
        return {
            "raw": raw,
            "compensated": comp,
            "raw_received": self._raw_received,
            "raw_rate_hz": self._raw_rate_hz,
            "tf_ok": (self._last_R is not None
                      and tf_age is not None
                      and tf_age <= self._tf_max_age),
            "tf_age": tf_age,
            "tf_error": self._last_tf_error,
            "live_g": live_g,
        }

    # --- end-effector CRUD ---------------------------------------------
    def api_create(self, name: str, description: str = "") -> None:
        self._store.create(name, description)

    def api_delete(self, name: str) -> None:
        self._store.delete(name)

    def api_rename(self, old_name: str, new_name: str) -> None:
        self._store.rename(old_name, new_name)

    def api_describe(self, name: str, description: str) -> None:
        self._store.set_description(name, description)

    def api_activate(self, name: str) -> None:
        ee = self._store.set_active(name)
        self.get_logger().info(
            f"active end-effector -> {ee.name!r} (mass={ee.params.mass:.4f} kg)")

    def api_set_params(self, name: str, body: dict) -> None:
        params = CompensationParams.from_dict(body)
        self._store.set_params(name, params, diag=None)
        if name == self._store.active_name:
            self.get_logger().info(
                f"updated parameters for active end-effector {name!r}: "
                f"mass={params.mass:.4f} kg, com=({params.com[0]:.4f}, "
                f"{params.com[1]:.4f}, {params.com[2]:.4f}) m")

    # --- samples --------------------------------------------------------
    def api_record_sample(self, name: str, note: str = "") -> None:
        with self._lock:
            if self._last_raw is None:
                raise RuntimeError("no raw wrench received yet -- check the input topic")
            if self._last_R is None:
                raise RuntimeError(
                    f"no TF available for {self._world_frame} -> {self._sensor_frame}: "
                    f"{self._last_tf_error}")
            _, f_raw, t_raw = self._last_raw
            R = np.array(self._last_R, dtype=float)
            f = f_raw.copy()
            t = t_raw.copy()
        sample = Sample(
            rotation=[[float(R[i, j]) for j in range(3)] for i in range(3)],
            force=[float(v) for v in f],
            torque=[float(v) for v in t],
            stamp=time.time(),
            note=note,
        )
        self._store.add_sample(name, sample)
        self.get_logger().info(
            f"recorded sample for {name!r} "
            f"(F=[{f[0]:.2f}, {f[1]:.2f}, {f[2]:.2f}] N, "
            f"T=[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] Nm)")

    def api_delete_sample(self, name: str, index: int) -> None:
        self._store.delete_sample(name, index)

    def api_clear_samples(self, name: str) -> None:
        self._store.clear_samples(name)

    def api_calibrate(self, name: str) -> dict:
        ee = self._store.get(name)
        if ee is None:
            raise KeyError(f"unknown end-effector '{name}'")
        if not ee.samples:
            raise ValueError("no samples recorded for this end-effector")

        samples_np = [
            (np.array(s.rotation, dtype=float),
             np.array(s.force, dtype=float),
             np.array(s.torque, dtype=float))
            for s in ee.samples
        ]
        result = calibrate_least_squares(samples_np, gravity=self._gravity)

        diag = {
            "n_samples": result.n_samples,
            "rms_force": float(result.rms_force),
            "rms_torque": float(result.rms_torque),
            "cond_force": float(result.cond_force),
            "cond_torque": float(result.cond_torque),
            "warnings": list(result.warnings),
        }
        self._store.set_params(name, result.params, diag=diag)
        self.get_logger().info(
            f"calibrated {name!r} from {result.n_samples} samples: "
            f"mass={result.params.mass:.4f} kg, "
            f"com=({result.params.com[0]:+.4f}, {result.params.com[1]:+.4f}, "
            f"{result.params.com[2]:+.4f}) m, "
            f"rms_F={result.rms_force:.3f} N, rms_T={result.rms_torque:.4f} Nm")
        for w in result.warnings:
            self.get_logger().warning(f"calibration warning ({name}): {w}")
        return diag


# ===========================================================================
# helpers
# ===========================================================================
def _local_ip() -> str:
    import socket as _s
    try:
        s = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "<this-host>"


# ===========================================================================
# entry point
# ===========================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompensationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
