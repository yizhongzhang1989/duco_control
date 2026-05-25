// =====================================================================
// force_plot.js
// ---------------------------------------------------------------------
// Renders a live force / torque plot in the cartesian_controller_dashboard
// (NOT the standalone ft_sensor_dashboard).  Two stacked canvases share
// the same time window:
//
//   #ft-plot-force   →  Fx / Fy / Fz  (N)
//   #ft-plot-torque  →  Tx / Ty / Tz  (Nm)
//
// Data source: backend ring buffer fed by the orchestrator's
// /duco_ft_sensor/wrench_compensated subscriber.  This module polls
// /api/wrench_samples?since=<lastT> at ~30 Hz and only receives the
// delta since the previous poll, so payload size is proportional to
// the publish rate, not the window length.
//
// Drawing approach is the min/max-per-pixel-column envelope used by
// the standalone ft_sensor_dashboard (oscilloscope-style).  At 1 kHz
// publish rate with a 10 s window the visible buffer holds ~10 000
// samples but each channel renders in O(plotW) per frame.
//
// The data structure is a plain typed-array pair with a `len`
// counter + periodic in-place compaction; that lets the lower_bound
// scan over a contiguous array which is much simpler than a ring
// buffer.
// =====================================================================
(() => {
  "use strict";

  // ---- Tunable constants ----------------------------------------------
  // Capacity must comfortably hold the longest window the user might
  // type (60 s) at the highest expected publish rate (~1 kHz) plus
  // headroom for hysteresis and a couple of compaction cycles.
  const CAP = 200000;
  // Poll rate in Hz.  30 Hz matches the live overlay refresh rate
  // elsewhere in this dashboard and keeps per-poll JSON payloads under
  // ~30 samples at 1 kHz publish rate.
  const POLL_HZ = 30;
  // Channel colors: x = red, y = green, z = blue (matches the
  // ft_sensor_dashboard convention so users get consistent visuals
  // across the two dashboards).
  const COLORS = ["#ef4444", "#22c55e", "#3b82f6"];
  // Status-pill staleness threshold.  If we haven't received any new
  // samples in this many seconds, the pill turns red.
  const STALE_S = 1.0;

  // ---- DOM lookups ----------------------------------------------------
  const cvForce  = document.getElementById("ft-plot-force");
  const cvTorque = document.getElementById("ft-plot-torque");
  const winInput = document.getElementById("ft-plot-window");
  const btnFreeze = document.getElementById("ft-plot-freeze");
  const pillStatus = document.getElementById("ft-plot-status");
  const els = {
    fx: document.getElementById("ft-vfx"),
    fy: document.getElementById("ft-vfy"),
    fz: document.getElementById("ft-vfz"),
    mx: document.getElementById("ft-vmx"),
    my: document.getElementById("ft-vmy"),
    mz: document.getElementById("ft-vmz"),
  };
  if (!cvForce || !cvTorque) {
    // Panel not present (e.g. dashboard with the plot disabled).
    // Silently skip; the rest of the dashboard works without us.
    return;
  }

  // ---- State ----------------------------------------------------------
  // ts[i] is a monotonic seconds timestamp (the same time base the
  // backend uses for ``now``); the six value arrays hold Fx/Fy/Fz/Mx/My/Mz.
  const ts = new Float64Array(CAP);
  const vals = [
    new Float32Array(CAP), new Float32Array(CAP), new Float32Array(CAP),
    new Float32Array(CAP), new Float32Array(CAP), new Float32Array(CAP),
  ];
  let len = 0;
  // ``tNow`` is the server's most recent monotonic clock reading; we
  // use that (rather than performance.now()) so the plot's x-axis
  // exactly matches the timestamps inside the buffer.
  let tNow = 0;
  // Used to detect "no recent traffic" for the status pill.
  let tLastReceived = 0;
  // Latest server-reported wrench rate (separate from our own poll
  // rate); the backend recomputes this over a 1 s window.
  let serverHz = NaN;
  let frozen = false;
  // User-tunable window length (seconds); 10 s default matches the
  // ft_sensor_dashboard.
  let WINDOW = 10;
  // Cursor for the /api/wrench_samples ?since= query.  Tracks the
  // monotonic timestamp of the most recent sample we've ingested so
  // the backend only sends us new tuples.
  let cursor = null;

  const FORCE_CHS  = [0, 1, 2];
  const TORQUE_CHS = [3, 4, 5];

  // ---- Buffer mechanics -----------------------------------------------
  function pushSample(t, fx, fy, fz, mx, my, mz) {
    if (len === CAP) compact(true);
    const i = len++;
    ts[i] = t;
    vals[0][i] = fx; vals[1][i] = fy; vals[2][i] = fz;
    vals[3][i] = mx; vals[4][i] = my; vals[5][i] = mz;
  }

  // Standard lower_bound: first index i in [0, len) with ts[i] >= x.
  function lowerBound(x) {
    let lo = 0, hi = len;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (ts[mid] < x) lo = mid + 1; else hi = mid;
    }
    return lo;
  }

  // Drop samples older than the visible window.  5% hysteresis avoids
  // shuffling memory on every push.  ``forceDrop`` is the safety
  // valve for a full buffer (drops the oldest half unconditionally).
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

  // ---- Canvas helpers -------------------------------------------------
  // Resize the canvas backing store to match its CSS box × devicePixelRatio
  // so 1-pixel strokes stay crisp on hi-DPI displays.  Returns the DPR so
  // the caller can scale paddings, line widths, font sizes, etc.
  function fitCanvas(c) {
    const dpr = window.devicePixelRatio || 1;
    const r = c.getBoundingClientRect();
    c.width  = Math.max(1, Math.round(r.width  * dpr));
    c.height = Math.max(1, Math.round(r.height * dpr));
    return dpr;
  }

  function drawPlot(c, channels) {
    const dpr = fitCanvas(c);
    const ctx = c.getContext("2d");
    const W = c.width, H = c.height;
    ctx.fillStyle = "#0c0e13"; ctx.fillRect(0, 0, W, H);

    // Smaller left pad than the ft_sensor_dashboard because the
    // rotated y-axis label lives in HTML next to the canvas, not
    // inside the canvas.
    const pad = { l: 40 * dpr, r: 10 * dpr, t: 8 * dpr, b: 20 * dpr };
    const plotW = W - pad.l - pad.r;
    const plotH = H - pad.t - pad.b;

    const tMin = tNow - WINDOW;
    const tMax = tNow;
    const i0 = lowerBound(tMin);

    // Min/max envelope downsample.  For each visible channel build
    // mins[col], maxs[col], have[col] in a single linear pass; drawing
    // then needs O(plotW) work per channel regardless of how many
    // samples are in the window.
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

    // Grid + tick labels.
    ctx.strokeStyle = "#22262f"; ctx.lineWidth = 1 * dpr;
    ctx.fillStyle   = "#8a93a6"; ctx.font = (10 * dpr) + "px ui-sans-serif, sans-serif";
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    const yTicks = 5;
    for (let i = 0; i <= yTicks; i++) {
      const v = yMin + (yMax - yMin) * i / yTicks;
      const yy = yPx(v);
      ctx.beginPath(); ctx.moveTo(pad.l, yy); ctx.lineTo(W - pad.r, yy); ctx.stroke();
      ctx.fillText(v.toFixed(2), pad.l - 4 * dpr, yy);
    }
    ctx.textAlign = "center"; ctx.textBaseline = "top";
    const xTicks = 5;
    for (let i = 0; i <= xTicks; i++) {
      const t = tMin + (tMax - tMin) * i / xTicks;
      const xx = pad.l + ((t - tMin) / WINDOW) * plotW;
      ctx.beginPath(); ctx.moveTo(xx, pad.t); ctx.lineTo(xx, H - pad.b); ctx.stroke();
      // Negative seconds = "this long ago"; the rightmost label is
      // always 0.0 s (now).
      ctx.fillText((t - tNow).toFixed(1) + "s", xx, H - pad.b + 3 * dpr);
    }
    // Highlight the zero crossing on the y axis if it's inside view.
    if (0 > yMin && 0 < yMax) {
      ctx.strokeStyle = "#3a4050";
      const yz = yPx(0);
      ctx.beginPath(); ctx.moveTo(pad.l, yz); ctx.lineTo(W - pad.r, yz); ctx.stroke();
    }

    // Trace: vertical min..max line per populated column, joined to
    // neighbors.  Slow-changing signals look like a thin line; noisy
    // signals look like a band -- the same envelope render style as
    // an oscilloscope's min/max display mode.
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

  // ---- Status pill / legend numbers -----------------------------------
  function fmtNum(v) {
    if (!isFinite(v)) return "--";
    const a = Math.abs(v);
    if (a >= 100) return v.toFixed(1);
    if (a >= 10)  return v.toFixed(2);
    return v.toFixed(3);
  }
  function updateLegend() {
    if (len === 0) {
      els.fx.textContent = els.fy.textContent = els.fz.textContent = "--";
      els.mx.textContent = els.my.textContent = els.mz.textContent = "--";
      return;
    }
    const i = len - 1;
    els.fx.textContent = fmtNum(vals[0][i]);
    els.fy.textContent = fmtNum(vals[1][i]);
    els.fz.textContent = fmtNum(vals[2][i]);
    els.mx.textContent = fmtNum(vals[3][i]);
    els.my.textContent = fmtNum(vals[4][i]);
    els.mz.textContent = fmtNum(vals[5][i]);
  }
  function updateStatusPill() {
    if (!pillStatus) return;
    const age = tNow - tLastReceived;
    if (!tLastReceived || age > STALE_S || !isFinite(serverHz)) {
      pillStatus.textContent = "no data";
      pillStatus.className = "pill";
      return;
    }
    pillStatus.textContent = serverHz.toFixed(1) + " Hz";
    pillStatus.className = "pill active";
  }

  // ---- Render loop ----------------------------------------------------
  function render() {
    drawPlot(cvForce,  FORCE_CHS);
    drawPlot(cvTorque, TORQUE_CHS);
    requestAnimationFrame(render);
  }
  requestAnimationFrame(render);

  // ---- Polling loop ---------------------------------------------------
  // We deliberately don't use setInterval; that can drift and pile up
  // overlapping fetches on slow connections.  Self-scheduling with
  // setTimeout(... after fetch resolves) keeps at most one in-flight
  // request and naturally throttles when the server is slow.
  let inFlight = false;
  async function poll() {
    if (inFlight) return;
    inFlight = true;
    try {
      const url = cursor === null
        ? "/api/wrench_samples"
        : "/api/wrench_samples?since=" + encodeURIComponent(cursor.toFixed(6));
      const resp = await fetch(url, { cache: "no-store" });
      if (!resp.ok) throw new Error("HTTP " + resp.status);
      const data = await resp.json();
      // Always update tNow from server clock so the x-axis right edge
      // tracks reality even when frozen (we just won't append samples).
      if (typeof data.now === "number") tNow = data.now;
      if (typeof data.hz === "number" || data.hz === null) {
        serverHz = (data.hz == null) ? NaN : data.hz;
      }
      const samples = data.samples || [];
      if (samples.length && !frozen) {
        for (const s of samples) {
          pushSample(s[0], s[1], s[2], s[3], s[4], s[5], s[6]);
        }
        cursor = samples[samples.length - 1][0];
        tLastReceived = tNow;
      } else if (samples.length && frozen) {
        // Even when frozen, advance the cursor so we don't replay the
        // same samples again on unfreeze; they'll just be skipped.
        cursor = samples[samples.length - 1][0];
        tLastReceived = tNow;
      }
      compact(false);
      updateLegend();
      updateStatusPill();
    } catch (_e) {
      // Network blips are normal on a busy ROS host; just retry.
      updateStatusPill();
    } finally {
      inFlight = false;
      setTimeout(poll, 1000 / POLL_HZ);
    }
  }
  poll();

  // ---- Controls -------------------------------------------------------
  if (winInput) {
    const applyWindow = () => {
      const v = parseFloat(winInput.value);
      if (isFinite(v) && v > 0 && v <= 600) {
        WINDOW = v;
        compact(false);
      }
    };
    winInput.addEventListener("change", applyWindow);
    winInput.addEventListener("input",  applyWindow);
    applyWindow();
  }
  if (btnFreeze) {
    btnFreeze.addEventListener("click", () => {
      frozen = !frozen;
      btnFreeze.textContent = frozen ? "Resume" : "Freeze";
      btnFreeze.classList.toggle("active", frozen);
    });
  }
})();
