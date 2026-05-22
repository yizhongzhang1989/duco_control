// ===========================================================================
// 3D URDF skeleton renderer.

// ---------------------------------------------------------------------------
// Self-contained Canvas2D viewer: no external libraries, no WebGL.  The
// dashboard receives the URDF tree once (/api/urdf_model) and polls live
// world-frame poses (/api/urdf_tf) a few times per second.  Each link's
// pose is projected through a yaw/pitch/distance orbit camera and drawn
// as a short coordinate triad; joints are drawn as line segments between
// the world positions of their parent and child links.
//
// This is just a skeleton -- no meshes, no lighting -- which matches the
// design intent ("3D window ... no need to include mesh, just skeleton").
// ===========================================================================
(function () {
  const canvas = document.getElementById("urdf-canvas");
  if (!canvas) return;
  const overlay   = document.getElementById("urdf-overlay");
  const statusEl  = document.getElementById("urdf-status");
  const rootEl    = document.getElementById("urdf-root");
  const nLinksEl  = document.getElementById("urdf-nlinks");
  const nJointsEl = document.getElementById("urdf-njoints");
  const tfAgeEl   = document.getElementById("urdf-tfage");
  const labelsCb  = document.getElementById("urdf-labels");
  const axesCb    = document.getElementById("urdf-axes");
  const fitBtn    = document.getElementById("urdf-fit");
  const reloadBtn = document.getElementById("urdf-reload");
  const ctx       = canvas.getContext("2d");

  // ------------------------------ state ----------------------------------
  let model    = null;     // {root_link, links, joints}
  let livePose = {};       // link_name -> {t:[x,y,z], q:[x,y,z,w]} | null
  let lastTfMs = null;

  // Arcball camera: orientation stored as a quaternion (world -> view),
  // plus a target point (world coords) and a scalar distance.  View axes
  // are X right, Y forward (toward the scene), Z up.  Rotation is a free
  // 3-DOF arcball drag rather than constrained yaw/pitch.
  const cam = {
    qcam:   [0, 0, 0, 1],
    dist:   2.5,
    target: [0, 0, 0.4],
    fovY:   Math.PI / 4,    // 45 deg vertical FOV
  };

  // -------------------------- quat / matrix ------------------------------
  function quatRotate(q, v) {
    // q = (x, y, z, w); rotates v (vec3) by q.
    const [x, y, z, w] = q;
    const [vx, vy, vz] = v;
    // t = 2 * cross(q.xyz, v)
    const tx = 2 * (y * vz - z * vy);
    const ty = 2 * (z * vx - x * vz);
    const tz = 2 * (x * vy - y * vx);
    // v' = v + w * t + cross(q.xyz, t)
    return [
      vx + w * tx + (y * tz - z * ty),
      vy + w * ty + (z * tx - x * tz),
      vz + w * tz + (x * ty - y * tx),
    ];
  }

  function quatMul(a, b) {
    // Hamilton product, q = (x, y, z, w).
    const [ax, ay, az, aw] = a;
    const [bx, by, bz, bw] = b;
    return [
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
      aw * bw - ax * bx - ay * by - az * bz,
    ];
  }

  function quatNormalize(q) {
    const n = Math.hypot(q[0], q[1], q[2], q[3]);
    if (n < 1e-12) return [0, 0, 0, 1];
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
  }

  function quatFromAxisAngle(axis, angle) {
    const h = angle * 0.5;
    const s = Math.sin(h);
    return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(h)];
  }

  function quatFromVectors(v1, v2) {
    // Quaternion that rotates unit vector v1 to v2.
    const d  = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    const cx = v1[1]*v2[2] - v1[2]*v2[1];
    const cy = v1[2]*v2[0] - v1[0]*v2[2];
    const cz = v1[0]*v2[1] - v1[1]*v2[0];
    if (d < -0.999999) {
      // Near-opposite: rotate 180 deg around any axis orthogonal to v1.
      const ax = Math.abs(v1[0]) < 0.5 ? [1, 0, 0] : [0, 1, 0];
      const ox = ax[1]*v1[2] - ax[2]*v1[1];
      const oy = ax[2]*v1[0] - ax[0]*v1[2];
      const oz = ax[0]*v1[1] - ax[1]*v1[0];
      return quatNormalize([ox, oy, oz, 0]);
    }
    return quatNormalize([cx, cy, cz, 1 + d]);
  }

  // Default 3/4 view: yaw 0.7 around world Z, then pitch 0.4 about the
  // (rotated) camera X axis -- same starting pose as the previous
  // yaw/pitch orbit camera so the user sees the familiar opening view.
  cam.qcam = quatNormalize(quatMul(
    quatFromAxisAngle([1, 0, 0], -0.4),
    quatFromAxisAngle([0, 0, 1], -0.7),
  ));

  // World -> view (camera) coords.  Camera sits at the view origin and
  // looks along +Y; the target lands at view (0, +dist, 0).  View axes:
  // X right, Y forward (into scene), Z up.
  function worldToView(p) {
    const d = [
      p[0] - cam.target[0],
      p[1] - cam.target[1],
      p[2] - cam.target[2],
    ];
    const v = quatRotate(cam.qcam, d);
    return [v[0], v[1] + cam.dist, v[2]];
  }

  function project(p) {
    const v = worldToView(p);
    const w = canvas.width, h = canvas.height;
    // Standard pinhole: project (x, z) onto screen using the camera's
    // forward axis (here +y after worldToView() above).  Reject points
    // behind the camera (v[1] <= 0).
    if (v[1] <= 0.01) return null;
    const f = (h * 0.5) / Math.tan(cam.fovY * 0.5);  // focal length in px
    const sx = w * 0.5 + (v[0] * f) / v[1];
    const sy = h * 0.5 - (v[2] * f) / v[1];
    return { x: sx, y: sy, depth: v[1] };
  }

  // -------------------------- HiDPI sizing -------------------------------
  function resize() {
    const rect = canvas.getBoundingClientRect();
    const dpr  = window.devicePixelRatio || 1;
    const w = Math.max(64, Math.round(rect.width  * dpr));
    const h = Math.max(64, Math.round(rect.height * dpr));
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w; canvas.height = h;
    }
  }
  // Drawing is driven by requestAnimationFrame (see renderLoop below),
  // so event handlers only need to mutate state -- they don't have to
  // call draw() themselves.  This keeps mouse interaction at the
  // browser's compositor rate (typically 60 Hz) and decouples render
  // smoothness from data-update frequency.
  window.addEventListener("resize", resize);

  // -------------------------- arcball mapping ----------------------------
  // Map a canvas-space pixel to a unit vector on the arcball (in view
  // space).  Inside the unit disk we lift to the sphere; outside we snap
  // to the equator.  View axes: X right, Y forward, Z up; "out of screen
  // toward the viewer" is -Y.
  function mapToSphere(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const cx = rect.width  * 0.5;
    const cy = rect.height * 0.5;
    const r  = Math.min(rect.width, rect.height) * 0.5;
    const x =  (clientX - rect.left - cx) / r;
    const z = -(clientY - rect.top  - cy) / r;
    const d2 = x * x + z * z;
    if (d2 <= 1.0) return [x, -Math.sqrt(1 - d2), z];
    const n = Math.sqrt(d2);
    return [x / n, 0, z / n];
  }

  // -------------------------- mouse interaction --------------------------
  // Bindings:
  //   left   (button 0) -> arcball rotate
  //   middle (button 1) -> dolly (vertical drag)
  //   right  (button 2) -> pan / grab (translate target in view plane)
  //   wheel             -> dolly
  let drag = null;
  canvas.addEventListener("mousedown", (ev) => {
    drag = {
      x: ev.clientX,
      y: ev.clientY,
      button: ev.button,
      qcamStart:  cam.qcam.slice(),
      distStart:  cam.dist,
      targetStart: cam.target.slice(),
      sphereStart: (ev.button === 0)
        ? mapToSphere(ev.clientX, ev.clientY) : null,
    };
    canvas.style.cursor = (ev.button === 0) ? "grabbing"
                       : (ev.button === 2) ? "move"
                       : "ns-resize";
    ev.preventDefault();
  });
  canvas.addEventListener("contextmenu", (ev) => ev.preventDefault());
  window.addEventListener("mouseup", () => {
    drag = null;
    canvas.style.cursor = "grab";
  });
  window.addEventListener("mousemove", (ev) => {
    if (!drag) return;
    if (drag.button === 0 && drag.sphereStart) {
      // Arcball rotate: qcam_new = q_drag * qcam_start.  q_drag rotates
      // the picked sphere point so it follows the cursor in view space.
      const v2 = mapToSphere(ev.clientX, ev.clientY);
      const qDrag = quatFromVectors(drag.sphereStart, v2);
      cam.qcam = quatNormalize(quatMul(qDrag, drag.qcamStart));
    } else if (drag.button === 2) {
      // Right-drag = pan / grab.  Move the camera target so the grabbed
      // world point tracks the cursor in screen space.  Convert the
      // screen-pixel drag to a view-space displacement at depth=dist,
      // then rotate back to world coords via qcam^-1 (= conjugate).
      const dx = ev.clientX - drag.x;
      const dy = ev.clientY - drag.y;
      const hCss = canvas.clientHeight || canvas.height;
      const fCss = (hCss * 0.5) / Math.tan(cam.fovY * 0.5);
      const s   = drag.distStart / fCss;
      // View-space displacement of the *target* such that the world
      // appears to slide with the cursor.  Screen X right (+dx) and
      // screen Y down (+dy) -> target moves -X and +Z in view.
      const dxv = -dx * s;
      const dzv =  dy * s;
      const q = drag.qcamStart;
      const qInv = [-q[0], -q[1], -q[2], q[3]];
      const dWorld = quatRotate(qInv, [dxv, 0, dzv]);
      cam.target = [
        drag.targetStart[0] + dWorld[0],
        drag.targetStart[1] + dWorld[1],
        drag.targetStart[2] + dWorld[2],
      ];
    } else {
      // Middle drag = dolly (vertical only).  Down = zoom out.
      const dy = ev.clientY - drag.y;
      cam.dist = Math.max(0.2, Math.min(20.0,
                  drag.distStart * Math.exp(dy * 0.005)));
    }
  });
  canvas.addEventListener("wheel", (ev) => {
    ev.preventDefault();
    const k = Math.exp(ev.deltaY * 0.001);
    cam.dist = Math.max(0.2, Math.min(20.0, cam.dist * k));
  }, { passive: false });

  // -------------------------- fetch helpers ------------------------------
  async function loadModel() {
    try {
      const r = await fetch("/api/urdf_model");
      const j = await r.json();
      if (!j.ok) {
        statusEl.textContent = "no URDF";
        statusEl.classList.remove("active");
        overlay.textContent = j.message || "URDF unavailable";
        model = null;
        return;
      }
      model = j;
      rootEl.textContent    = j.root_link || "--";
      nLinksEl.textContent  = j.links.length;
      nJointsEl.textContent = j.joints.length;
      statusEl.textContent  = "loaded";
      statusEl.classList.add("active");
      overlay.textContent = "";
      fitView();
    } catch (e) {
      statusEl.textContent = "error";
      overlay.textContent = "fetch /api/urdf_model failed: " + e.message;
    }
  }

  // ---- live TF stream via Server-Sent Events ----------------------------
  // One persistent connection to /api/urdf_tf_stream; the server pushes
  // the latest tf snapshot at ~30 Hz.  Each message just updates the
  // ``livePose`` cache -- the rAF render loop picks it up on the next
  // frame.  EventSource auto-reconnects on transient failures; we add a
  // 1 s backoff for the "server not ready yet" case so we don't flood.
  let evtSource = null;
  function startTfStream() {
    try { if (evtSource) evtSource.close(); } catch (_) {}
    evtSource = new EventSource("/api/urdf_tf_stream");
    evtSource.onmessage = (ev) => {
      try {
        const j = JSON.parse(ev.data);
        if (j && j.ok) {
          livePose = j.transforms || {};
          lastTfMs = performance.now();
        }
      } catch (_) { /* ignore malformed frame */ }
    };
    evtSource.onerror = () => {
      try { evtSource.close(); } catch (_) {}
      evtSource = null;
      setTimeout(startTfStream, 1000);
    };
  }

  // -------------------------- fit view -----------------------------------
  // Walk the URDF tree using either live TFs or the static origins, find
  // a bounding sphere, and centre the camera on it.
  function staticPose(linkName) {
    if (!model) return [0, 0, 0];
    if (linkName === model.root_link) return [0, 0, 0];
    // BFS: accumulate origin_xyz down the tree (ignores rotations -- the
    // result is a rough centroid, good enough for fit-view).
    const byChild = {};
    for (const j of model.joints) byChild[j.child] = j;
    const path = [];
    let cur = linkName;
    let guard = 0;
    while (cur && cur !== model.root_link && guard++ < 64) {
      const j = byChild[cur];
      if (!j) break;
      path.push(j);
      cur = j.parent;
    }
    let p = [0, 0, 0];
    for (let i = path.length - 1; i >= 0; --i) {
      const j = path[i];
      // Apply parent rotation chain to j.origin_xyz: skipped for fit-view
      // estimate -- just sum the origin_xyz so we have a coarse extent.
      p = [p[0] + j.origin_xyz[0],
           p[1] + j.origin_xyz[1],
           p[2] + j.origin_xyz[2]];
    }
    return p;
  }

  function linkWorldPos(linkName) {
    const live = livePose[linkName];
    if (live && live.t) return live.t;
    return staticPose(linkName);
  }

  function fitView() {
    if (!model) return;
    const pts = model.links.map(linkWorldPos);
    if (pts.length === 0) return;
    let cx = 0, cy = 0, cz = 0;
    for (const p of pts) { cx += p[0]; cy += p[1]; cz += p[2]; }
    cx /= pts.length; cy /= pts.length; cz /= pts.length;
    let rmax = 0.0;
    for (const p of pts) {
      const dx = p[0] - cx, dy = p[1] - cy, dz = p[2] - cz;
      rmax = Math.max(rmax, Math.hypot(dx, dy, dz));
    }
    cam.target = [cx, cy, cz];
    // Push back enough that the sphere fits the vertical FOV.
    cam.dist = Math.max(0.4, rmax / Math.tan(cam.fovY * 0.5) * 1.6);
  }

  // -------------------------- drawing ------------------------------------
  function drawSeg(p0, p1, color, width) {
    const a = project(p0), b = project(p1);
    if (!a || !b) return;
    ctx.strokeStyle = color;
    ctx.lineWidth   = (width || 1.5) * (window.devicePixelRatio || 1);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }

  function drawDot(p, color, radius) {
    const s = project(p);
    if (!s) return;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(s.x, s.y,
            (radius || 3) * (window.devicePixelRatio || 1),
            0, Math.PI * 2);
    ctx.fill();
  }

  function drawText(p, text, color) {
    const s = project(p);
    if (!s) return;
    const dpr = window.devicePixelRatio || 1;
    ctx.fillStyle = color;
    ctx.font = (11 * dpr) + "px ui-sans-serif, system-ui, sans-serif";
    ctx.fillText(text, s.x + 6 * dpr, s.y - 4 * dpr);
  }

  function drawTriad(origin, quat, scale) {
    // World-frame axes rotated by quat, with length `scale`.
    const ax = quatRotate(quat, [scale, 0, 0]);
    const ay = quatRotate(quat, [0, scale, 0]);
    const az = quatRotate(quat, [0, 0, scale]);
    const ex = [origin[0]+ax[0], origin[1]+ax[1], origin[2]+ax[2]];
    const ey = [origin[0]+ay[0], origin[1]+ay[1], origin[2]+ay[2]];
    const ez = [origin[0]+az[0], origin[1]+az[1], origin[2]+az[2]];
    drawSeg(origin, ex, "#ef4444", 2);  // X red
    drawSeg(origin, ey, "#22c55e", 2);  // Y green
    drawSeg(origin, ez, "#3b82f6", 2);  // Z blue
  }

  function drawGrid() {
    // 1m grid on the world Z=0 plane, +/- 1m extent.
    const N = 4;          // 4 lines either side of the origin
    const step = 0.25;    // 25 cm spacing
    ctx.strokeStyle = "#1f2937";
    ctx.lineWidth   = 1;
    for (let i = -N; i <= N; ++i) {
      const v = i * step;
      drawSeg([-N * step, v, 0], [ N * step, v, 0],
              i === 0 ? "#2b3140" : "#161a22", 1);
      drawSeg([v, -N * step, 0], [v,  N * step, 0],
              i === 0 ? "#2b3140" : "#161a22", 1);
    }
  }

  function draw() {
    resize();
    ctx.fillStyle = "#0b0e14";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    drawGrid();
    // World origin axes (3 cm) as a reference.
    drawTriad([0, 0, 0], [0, 0, 0, 1], 0.15);

    if (!model) {
      const dpr = window.devicePixelRatio || 1;
      ctx.fillStyle = "#8a93a6";
      ctx.font = (12 * dpr) + "px ui-sans-serif, system-ui, sans-serif";
      ctx.fillText("waiting for URDF...", 12 * dpr, 22 * dpr);
      return;
    }

    // Joints first (parent_pos -> child_pos), so dots/labels paint on top.
    const showAxes   = !!(axesCb && axesCb.checked);
    const showLabels = !!(labelsCb && labelsCb.checked);

    for (const j of model.joints) {
      const p = linkWorldPos(j.parent);
      const c = linkWorldPos(j.child);
      // Color by joint type: fixed = grey, others = teal.
      const color = (j.type === "fixed") ? "#6b7280" : "#22d3ee";
      drawSeg(p, c, color, j.type === "fixed" ? 1.5 : 2.5);
    }

    // Per-link triads + dots.  Labels are collected here and drawn in a
    // post-pass below so that links whose screen positions overlap (e.g.
    // ``ft_sensor_link`` and ``compliance_link`` mounted at the
    // ``link_6`` flange with zero xyz/rpy) get merged into a single
    // comma-separated label instead of stacking illegibly on top of one
    // another.
    const labelHits = [];  // [{ name, sx, sy }]
    for (const linkName of model.links) {
      const pose = livePose[linkName];
      const pos = (pose && pose.t) ? pose.t : staticPose(linkName);
      const quat = (pose && pose.q) ? pose.q : [0, 0, 0, 1];
      if (showAxes) drawTriad(pos, quat, 0.04);
      drawDot(pos,
              linkName === model.root_link ? "#fbbf24" : "#e6e6e6",
              linkName === model.root_link ? 4 : 2.5);
      if (showLabels) {
        const s = project(pos);
        if (s) labelHits.push({ name: linkName, sx: s.x, sy: s.y });
      }
    }

    if (showLabels && labelHits.length) {
      // Greedy O(n^2) clustering by screen distance.  The first label to
      // anchor a cluster keeps it pinned; subsequent labels within
      // ``MERGE_PX`` of that anchor join it.  Threshold scales with DPR
      // so HiDPI displays merge at the same visual radius as 1x.
      const dpr = window.devicePixelRatio || 1;
      const MERGE_PX = 12 * dpr;
      const clusters = [];
      for (const lbl of labelHits) {
        let merged = false;
        for (const c of clusters) {
          if (Math.hypot(lbl.sx - c.sx, lbl.sy - c.sy) < MERGE_PX) {
            c.names.push(lbl.name);
            merged = true;
            break;
          }
        }
        if (!merged) {
          clusters.push({ sx: lbl.sx, sy: lbl.sy, names: [lbl.name] });
        }
      }
      ctx.fillStyle = "#cfd3dc";
      ctx.font = (11 * dpr) + "px ui-sans-serif, system-ui, sans-serif";
      for (const c of clusters) {
        ctx.fillText(c.names.join(", "), c.sx + 6 * dpr, c.sy - 4 * dpr);
      }
    }

    if (lastTfMs != null) {
      const age = (performance.now() - lastTfMs) / 1000;
      tfAgeEl.textContent = age < 1 ? (age * 1000).toFixed(0) + " ms"
                                    : age.toFixed(2) + " s";
    } else {
      tfAgeEl.textContent = "--";
    }
  }

  // -------------------------- wiring -------------------------------------
  fitBtn.addEventListener("click", fitView);
  reloadBtn.addEventListener("click", loadModel);
  // labels / axes toggles just flip state -- the rAF loop redraws.
  labelsCb.addEventListener("change", () => {});
  axesCb.addEventListener("change", () => {});

  // Continuous render loop, decoupled from data.  rAF is throttled
  // automatically when the tab is hidden so this is free in the
  // background.
  function renderLoop() {
    draw();
    requestAnimationFrame(renderLoop);
  }

  resize();
  loadModel();
  startTfStream();
  requestAnimationFrame(renderLoop);
  // Re-try model fetch every 5 s while it's still missing.
  setInterval(() => { if (!model) loadModel(); }, 5000);
})();
