(function () {
  const $  = (id) => document.getElementById(id);
  const fmt = (v, d) => (v == null || isNaN(v)) ? "--" : Number(v).toFixed(d ?? 2);
  const fmtAge = (a) => a == null ? "--" : (a < 1 ? (a*1000).toFixed(0) + " ms" : a.toFixed(2) + " s");
  // Publish-rate formatter for the sticky-bar freshness pills.  The
  // backend ships ``..._hz`` fields next to ``..._age``; we display
  // the Hz directly.  Sub-10 Hz gets one decimal so a 5 Hz
  // orchestrator feed doesn't show "5 Hz" when it's actually 4.7 Hz;
  // higher rates round to an integer to avoid jitter in the readout.
  const fmtHz = (hz) =>
    (hz == null || !isFinite(hz) || hz <= 0)
      ? "--"
      : (hz < 10 ? hz.toFixed(1) : Math.round(hz).toString()) + " Hz";

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
    const kind = entry ? entry.kind : "--";
    $("kind-pill").textContent = "kind: " + kind;
    // Sticky-bar pill: always reflects the *active* controller's kind
    // (independent of dropdown selection).  When the dropdown happens
    // to track the active controller, these two pills agree.
    const activeEntry = avail.find((c) => c.name === ctl.active_controller);
    const activeKind = activeEntry ? activeEntry.kind : "--";
    const activePill = $("active-kind-pill");
    if (activePill) activePill.textContent = "kind: " + activeKind;
    setCommandPanel(kind, !!ctl.engaged);
  }

  function setCommandPanel(kind, engaged) {
    $("cmd-kind-pill").textContent = "kind: " + kind;
    const showForce = (kind === "force");
    const showJog   = (kind === "motion" || kind === "compliance");
    $("cmd-force").hidden               = !showForce;
    $("cmd-motion-compliance").hidden   = !showJog;
    // Kind-aware help text + snap-button label: compliance is the
    // pose-hold spring (snapshot on engage; push to deflect; release
    // to spring back); motion is rigid pose-hold (drive to target).
    const helpMotion = $("cmd-help-motion");
    const helpComp   = $("cmd-help-compliance");
    if (helpMotion && helpComp) {
      helpMotion.classList.toggle("active", kind === "motion");
      helpComp.classList.toggle("active", kind === "compliance");
    }
    const snapBtn = $("btn-snap-target");
    if (snapBtn) {
      snapBtn.textContent = (kind === "compliance")
        ? "Re-snap hold pose"
        : "Snap target to current pose";
      snapBtn.classList.toggle("warn", kind === "compliance");
    }
    // Jog buttons act on the *engaged* controller -- if not engaged,
    // disable them to make intent obvious.  (Pre-engage targets get
    // clobbered by FZI's on_activate snapshot anyway.)
    const dis = !engaged;
    document.querySelectorAll("#cmd-motion-compliance button.jog")
      .forEach((b) => { b.disabled = dis; });
    if (snapBtn) snapBtn.disabled = dis;
  }

  // Render a publish-rate pill.  ``hz`` is the backend's measured
  // rate (sliding-window Hz); ``age`` is only used to decide whether
  // the stream is OK / bad according to the source's freshness
  // threshold -- when ``hz`` is null we fall back to displaying
  // "no data" instead of an inverted age, because for high publish
  // rates 1/age is too jittery to be useful.
  function setRatePill(elId, ok, hz) {
    const el = $(elId);
    el.classList.remove("ok", "bad", "warn");
    if (hz == null) {
      el.textContent = "no data";
      el.classList.add("warn");
    } else {
      el.textContent = fmtHz(hz);
      el.classList.add(ok ? "ok" : "bad");
    }
  }

  function setOrchPill(s) {
    const el = $("orch-status");
    el.classList.remove("ok", "bad", "warn");
    if (!s || s.control_state_age == null) {
      el.textContent = "no orchestrator";
      el.classList.add("bad");
      return;
    }
    const alive = s.control_state_age <= 5.0;
    const hz = s.control_state_hz;
    if (hz == null) {
      // Fresh enough that age <= 5s but rate tracker doesn't have
      // two samples yet (e.g. just-started orchestrator): say
      // "live" rather than "no data" so the operator isn't alarmed.
      el.textContent = alive ? "live" : "stale";
      el.classList.add(alive ? "ok" : "warn");
      return;
    }
    el.textContent = fmtHz(hz);
    el.classList.add(alive ? "ok" : "warn");
  }

  // Sticky-bar TCP-pose readout.  Values come from /api/live's
  // ``tcp`` field, which the backend fills via tf2 (base_frame ->
  // tool_frame).  When the lookup fails (no /tf yet or stale TF)
  // the field is null; we show "--" in the TCP cells and a "no tf"
  // pill so failure is obvious without dimming half of the shared
  // data row (the wrench cells are unaffected).  The base/tool
  // frame names live in the row's title= tooltip so the operator
  // can hover to confirm what frame the pose is expressed in.
  //
  // The TF pill lives in the row-1 status strip and shows publish
  // rate in Hz (capped at the backend's 50 Hz sampler), or "static"
  // for a static/unstamped TF, or "no tf" when the lookup fails.
  function setTcpPose(s) {
    const row = $("sb-data-row");
    const tcp = s && s.tcp;
    const base = (s && s.base_frame) || "?";
    const tool = (s && s.tool_frame) || "?";
    if (row) row.title =
        "XYZ / RPY in " + base + " \u2192 " + tool + "\n"
      + "F / T in sensor frame";
    const statusPill = $("tcp-status");
    if (!tcp) {
      $("tcp-x").textContent  = "--";
      $("tcp-y").textContent  = "--";
      $("tcp-z").textContent  = "--";
      $("tcp-r").textContent  = "--";
      $("tcp-p").textContent  = "--";
      $("tcp-yaw").textContent = "--";
      if (statusPill) {
        statusPill.textContent = "no tf";
        statusPill.classList.remove("ok", "warn");
        statusPill.classList.add("bad");
      }
      return;
    }
    $("tcp-x").textContent  = fmt(tcp.x, 4);
    $("tcp-y").textContent  = fmt(tcp.y, 4);
    $("tcp-z").textContent  = fmt(tcp.z, 4);
    $("tcp-r").textContent  = fmt(tcp.roll_deg, 1);
    $("tcp-p").textContent  = fmt(tcp.pitch_deg, 1);
    $("tcp-yaw").textContent = fmt(tcp.yaw_deg, 1);
    if (statusPill) {
      statusPill.classList.remove("ok", "warn", "bad");
      if (tcp.age == null) {
        // Static / unstamped transform -- no rate to compute.
        statusPill.textContent = "static";
        statusPill.classList.add("ok");
      } else if (tcp.hz != null) {
        statusPill.textContent = fmtHz(tcp.hz);
        statusPill.classList.add(tcp.age > 1.0 ? "warn" : "ok");
      } else {
        // Have a stamped TF but the rate tracker doesn't have two
        // samples yet (just started).
        statusPill.textContent = tcp.age > 1.0 ? "stale" : "live";
        statusPill.classList.add(tcp.age > 1.0 ? "warn" : "ok");
      }
    }
  }

  function applyLive(s) {
    if (!s) return;
    const ctl = s.control || {};
    setEngagedPill(ctl);
    setControllerSelect(ctl);
    setOrchPill(s);
    setTcpPose(s);

    const w = s.wrench;
    $("wfx").textContent = w ? fmt(w.fx) : "--";
    $("wfy").textContent = w ? fmt(w.fy) : "--";
    $("wfz").textContent = w ? fmt(w.fz) : "--";
    $("wmx").textContent = w ? fmt(w.tx, 3) : "--";
    $("wmy").textContent = w ? fmt(w.ty, 3) : "--";
    $("wmz").textContent = w ? fmt(w.tz, 3) : "--";
    // Force / torque magnitudes are no longer shown in the sticky
    // bar (compact "XYZ RPY F T" format); keep guarded writes for
    // any external consumer that may still inject those elements.
    const fmagEl = $("fmag"), tmagEl = $("tmag");
    if (fmagEl || tmagEl) {
      if (w) {
        const fm = Math.hypot(w.fx, w.fy, w.fz);
        const tm = Math.hypot(w.tx, w.ty, w.tz);
        if (fmagEl) fmagEl.textContent = fmt(fm) + " N";
        if (tmagEl) tmagEl.textContent = fmt(tm, 3) + " Nm";
      } else {
        if (fmagEl) fmagEl.textContent = "-- N";
        if (tmagEl) tmagEl.textContent = "-- Nm";
      }
    }

    setRatePill("ft-status", !!s.ft_ok, w ? w.hz : null);
    setRatePill("q-status",  !!s.joint_states_ok, s.joint_states_hz);

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
      const nameEl = $("controller-name");
      if (nameEl) nameEl.textContent = snapshot.controller_name || "--";
      $("param-target").textContent    = snapshot.controller_name || "--";
      const ctl = snapshot.control || {};
      const lim = ctl.limits || {};
      $("lim-f").textContent       = fmt(lim.max_wrench_force);
      $("lim-t").textContent       = fmt(lim.max_wrench_torque, 3);
      $("lim-qe").textContent      = fmt(lim.engage_max_joint_velocity, 3);
      $("lim-ftstale").textContent = fmt(lim.ft_stale_after, 2);
      $("lim-qstale").textContent  = fmt(lim.joint_states_stale_after, 2);
      syncSafetyInputs(lim);
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
      const lim = (s.control || {}).limits || {};
      $("lim-f").textContent       = fmt(lim.max_wrench_force);
      $("lim-t").textContent       = fmt(lim.max_wrench_torque, 3);
      $("lim-qe").textContent      = fmt(lim.engage_max_joint_velocity, 3);
      $("lim-ftstale").textContent = fmt(lim.ft_stale_after, 2);
      $("lim-qstale").textContent  = fmt(lim.joint_states_stale_after, 2);
      syncSafetyInputs(lim);
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

  // ---- jog / snap-target handlers ----
  function fmtPos(p) {
    if (!p) return "--";
    return "x=" + fmt(p.x, 4) + "  y=" + fmt(p.y, 4) + "  z=" + fmt(p.z, 4);
  }

  async function postJog(body) {
    try {
      const r = await api("/api/jog", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body)
      });
      $("last-target-pos").textContent = fmtPos(r.target && r.target.position);
    } catch (e) {
      toast("jog failed: " + e.message, "bad");
    }
  }

  // Translation jog buttons.
  document.querySelectorAll("#cmd-motion-compliance button.jog[data-axis]")
    .forEach((btn) => {
      btn.addEventListener("click", () => {
        const step = parseFloat($("jog-trans-step").value);
        if (isNaN(step) || step <= 0) {
          toast("trans step must be > 0", "bad"); return;
        }
        const sign = parseInt(btn.dataset.sign, 10);
        const body = { dx: 0, dy: 0, dz: 0 };
        body["d" + btn.dataset.axis] = sign * step;
        postJog(body);
      });
    });

  // Rotation jog buttons.
  document.querySelectorAll("#cmd-motion-compliance button.jog[data-raxis]")
    .forEach((btn) => {
      btn.addEventListener("click", () => {
        const stepDeg = parseFloat($("jog-rot-step").value);
        if (isNaN(stepDeg) || stepDeg <= 0) {
          toast("rot step must be > 0", "bad"); return;
        }
        const rad = stepDeg * Math.PI / 180.0;
        const sign = parseInt(btn.dataset.sign, 10);
        const body = { drx: 0, dry: 0, drz: 0 };
        body["d" + btn.dataset.raxis] = sign * rad;
        postJog(body);
      });
    });

  $("btn-snap-target").addEventListener("click", async () => {
    try {
      const r = await api("/api/snap_target", { method: "POST" });
      $("last-target-pos").textContent = fmtPos(r.target && r.target.position);
      const kind = (r && r.kind) || "motion";
      toast(
        kind === "compliance"
          ? "spring re-anchored at current pose"
          : "target snapped to current pose",
        "ok");
    } catch (e) {
      toast("snap failed: " + e.message, "bad");
    }
  });

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

  // ---- Tool-frames editor ------------------------------------------------
  // Server-side write-back path is:
  //   GET  /api/aux_frames -> { frames: [{name, parent, xyz, rpy}, ...] }
  //   POST /api/aux_frames { frames: [...] } -> writes config/robot_config.yaml
  //
  // The on-disk format is the source of truth; we never auto-refresh
  // the rendered table from server data while the user is editing --
  // doing so could clobber in-progress typing.  Dirty inputs get a
  // yellow ring until Save commits them.
  function clearAuxDirty() {
    document.querySelectorAll("#aux-table input.aux-num.dirty")
      .forEach((el) => el.classList.remove("dirty"));
    $("aux-dirty-pill").textContent = "";
  }
  function markAuxDirty(inp) {
    inp.classList.add("dirty");
    const n = document.querySelectorAll("#aux-table input.aux-num.dirty").length;
    $("aux-dirty-pill").textContent =
      n + " unsaved field" + (n === 1 ? "" : "s");
  }
  function renderAuxFrames(payload) {
    const frames = (payload && payload.frames) || [];
    if (payload && payload.config_path) {
      $("aux-config-path").textContent = payload.config_path;
    }
    const body = $("aux-body");
    body.innerHTML = "";
    if (!frames.length) {
      $("aux-status").textContent = "none";
      body.innerHTML =
        '<tr><td colspan="8" class="small">no aux_frames in config</td></tr>';
      return;
    }
    $("aux-status").textContent =
      frames.length + " frame" + (frames.length === 1 ? "" : "s");
    for (const f of frames) {
      const tr = document.createElement("tr");
      const nameTd = document.createElement("td");
      nameTd.className = "aux-name";
      nameTd.innerHTML = "<code>" + (f.name || "?") + "</code>";
      const parentTd = document.createElement("td");
      parentTd.innerHTML = "<code>" + (f.parent || "?") + "</code>";
      tr.append(nameTd, parentTd);
      const xyz = Array.isArray(f.xyz) && f.xyz.length === 3 ? f.xyz : [0,0,0];
      const rpy = Array.isArray(f.rpy) && f.rpy.length === 3 ? f.rpy : [0,0,0];
      const addInput = (kind, idx, val) => {
        const td = document.createElement("td");
        const inp = document.createElement("input");
        inp.type = "number"; inp.step = "0.0001";
        inp.classList.add("aux-num");
        inp.dataset.frame = f.name;
        inp.dataset.kind  = kind;
        inp.dataset.idx   = String(idx);
        inp.value = Number(val);
        inp.addEventListener("input", () => markAuxDirty(inp));
        td.appendChild(inp); tr.appendChild(td);
      };
      for (let i = 0; i < 3; i++) addInput("xyz", i, xyz[i]);
      for (let i = 0; i < 3; i++) addInput("rpy", i, rpy[i]);
      body.appendChild(tr);
    }
    clearAuxDirty();
  }
  function collectAuxFrames() {
    const byName = {};
    document.querySelectorAll("#aux-table input.aux-num").forEach((inp) => {
      const name = inp.dataset.frame;
      const kind = inp.dataset.kind;
      const idx  = parseInt(inp.dataset.idx, 10);
      const v    = parseFloat(inp.value);
      if (!byName[name]) byName[name] = { name, xyz: [0,0,0], rpy: [0,0,0] };
      byName[name][kind][idx] = isNaN(v) ? 0 : v;
    });
    return Object.values(byName);
  }
  async function reloadAuxFrames() {
    try {
      const r = await api("/api/aux_frames");
      renderAuxFrames(r);
      $("aux-msg").textContent =
        (r.message || "loaded") + " -- "
        + (r.frames || []).length + " frame(s)";
    } catch (e) {
      $("aux-status").textContent = "error";
      $("aux-msg").textContent = "load failed: " + e.message;
      toast("aux_frames load failed: " + e.message, "bad");
    }
  }
  async function saveAuxFrames() {
    const frames = collectAuxFrames();
    if (!frames.length) {
      toast("nothing to save", "warn"); return;
    }
    try {
      const r = await api("/api/aux_frames", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ frames })
      });
      // The backend now reports a structured `live` block describing
      // whether the new URDF made it into robot_state_publisher.  When
      // live update succeeded the message says so; when it failed we
      // fall back to telling the operator the yaml is saved but a
      // bringup restart is needed to apply the change.
      let msg = "saved " + r.updated + " entrie(s) to " + r.config_path;
      if (r.live && r.live.ok) {
        msg += " -- applied live via robot_state_publisher";
        toast("aux_frames applied live (" + r.updated + ")", "ok");
      } else {
        const why = (r.live && r.live.error) ? r.live.error : "unknown";
        msg += " -- live URDF push failed (" + why
             + "); restart duco_robot_bringup to apply";
        toast("aux_frames saved (live push failed)", "warn");
      }
      $("aux-msg").textContent = msg;
      // Re-fetch so the rendered values match exactly what landed on
      // disk (the round-trip canonicalises numbers).
      await reloadAuxFrames();
    } catch (e) {
      toast("save failed: " + e.message, "bad");
    }
  }
  $("btn-aux-reload").addEventListener("click", reloadAuxFrames);
  $("btn-aux-save").addEventListener("click", saveAuxFrames);
  // Kick off an initial load (does not block the main refresh loop).
  reloadAuxFrames();

  // ---- Safety-thresholds editor ----------------------------------------
  // Backend path:
  //   GET  /api/safety_thresholds -> read-only summary (we don't actually
  //                                  need it; the main /api/live tick
  //                                  already carries control.limits).
  //   POST /api/safety_thresholds { thresholds: [{name, value}, ...] }
  //
  // The "current" column tracks the orchestrator state on every tick;
  // the "new" inputs are only auto-filled when not dirty so the
  // operator's in-progress typing isn't clobbered.
  function markSafetyDirty(inp) {
    inp.classList.add("dirty");
    const n = document.querySelectorAll(
      "#safety-table input.safety-num.dirty").length;
    $("safety-dirty-pill").textContent =
      n + " unsaved field" + (n === 1 ? "" : "s");
  }
  function clearSafetyDirty() {
    document.querySelectorAll("#safety-table input.safety-num.dirty")
      .forEach((el) => el.classList.remove("dirty"));
    $("safety-dirty-pill").textContent = "";
  }
  function syncSafetyInputs(limits) {
    // Auto-fill each input from the orchestrator's published limit
    // *only* when the operator hasn't already edited it.  Inputs that
    // are dirty or currently focused are left alone.
    document.querySelectorAll("#safety-table input.safety-num")
      .forEach((inp) => {
        if (inp.classList.contains("dirty")) return;
        if (document.activeElement === inp) return;
        const v = limits ? limits[inp.dataset.name] : undefined;
        if (v == null || isNaN(v)) {
          inp.value = "";
        } else {
          // 4 dp keeps room for ft_stale_after = 0.25 etc.; trailing
          // zeros get trimmed by the browser's number input rendering.
          inp.value = Number(v);
        }
      });
    const anyMissing = (!limits || $("lim-f").textContent === "--");
    const pill = $("safety-status");
    pill.classList.remove("ok", "bad", "warn");
    if (anyMissing) {
      pill.textContent = "no orchestrator";
      pill.classList.add("bad");
    } else {
      pill.textContent = "live";
      pill.classList.add("ok");
    }
  }
  document.querySelectorAll("#safety-table input.safety-num")
    .forEach((inp) => {
      inp.addEventListener("input", () => markSafetyDirty(inp));
      inp.addEventListener("keydown", (ev) => {
        if (ev.key === "Enter") {
          ev.preventDefault();
          saveSafetyThresholds();
        } else if (ev.key === "Escape") {
          inp.classList.remove("dirty");
          // Re-pull from the last snapshot's limits.
          const lim = (snapshot && snapshot.control && snapshot.control.limits)
                       || {};
          syncSafetyInputs(lim);
        }
      });
    });
  async function saveSafetyThresholds() {
    const thresholds = [];
    document.querySelectorAll("#safety-table input.safety-num.dirty")
      .forEach((inp) => {
        const v = parseFloat(inp.value);
        if (!isNaN(v)) {
          thresholds.push({ name: inp.dataset.name, value: v });
        }
      });
    if (!thresholds.length) {
      toast("no threshold changes to apply", "warn"); return;
    }
    try {
      const r = await api("/api/safety_thresholds", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ thresholds })
      });
      const failed = (r.results || []).filter((p) => !p.successful);
      if (r.ok && !failed.length) {
        toast("applied " + r.updated + " threshold(s)", "ok");
        clearSafetyDirty();
      } else {
        const why = failed.map((p) => p.name + ": " + p.reason).join("; ");
        toast("orchestrator rejected: " + (why || r.message), "bad");
      }
      // Re-read state so the "current" column updates.
      await refresh();
    } catch (e) {
      toast("save safety thresholds failed: " + e.message, "bad");
    }
  }
  function resetSafetyEdits() {
    clearSafetyDirty();
    const lim = (snapshot && snapshot.control && snapshot.control.limits)
                 || {};
    syncSafetyInputs(lim);
  }
  $("btn-safety-save").addEventListener("click", saveSafetyThresholds);
  $("btn-safety-reset").addEventListener("click", resetSafetyEdits);

  // ---- Wrench-deadband editor ------------------------------------------
  // Backend paths:
  //   GET  /api/wrench_deadband  -> per-axis force / torque dead-zone
  //                                 currently set on the gravity-comp node.
  //   POST /api/wrench_deadband  -> push edits
  //     { deadbands: [{name: "force_deadband", value: [x,y,z]}, ...] }
  //
  // The deadband doesn't change from the outside (only the operator
  // edits it), so we don't poll it on every tick.  We fetch it on the
  // slow ``refresh()`` path (every 3 s) and after every successful
  // save so the "current" inputs reflect what the node actually
  // accepted; a manual Reload button is provided for impatient users.
  let deadbandCache = null;   // last response from GET /api/wrench_deadband

  function markDeadbandDirty(inp) {
    inp.classList.add("dirty");
    const n = document.querySelectorAll(
      "#deadband-table input.deadband-num.dirty").length;
    $("deadband-dirty-pill").textContent =
      n + " unsaved field" + (n === 1 ? "" : "s");
  }
  function clearDeadbandDirty() {
    document.querySelectorAll("#deadband-table input.deadband-num.dirty")
      .forEach((el) => el.classList.remove("dirty"));
    $("deadband-dirty-pill").textContent = "";
  }
  function syncDeadbandInputs(payload) {
    // payload = api_get_wrench_deadband() response.  Each item is
    //   {name, label, unit, value: [x,y,z] | null}
    // Inputs that are dirty or focused are left alone so in-progress
    // edits aren't clobbered by a periodic refresh.
    const byName = {};
    if (payload && Array.isArray(payload.deadbands)) {
      for (const d of payload.deadbands) byName[d.name] = d.value;
    }
    document.querySelectorAll("#deadband-table input.deadband-num")
      .forEach((inp) => {
        if (inp.classList.contains("dirty")) return;
        if (document.activeElement === inp) return;
        const vec = byName[inp.dataset.name];
        const idx = parseInt(inp.dataset.axis, 10);
        if (Array.isArray(vec) && isFinite(vec[idx])) {
          inp.value = Number(vec[idx]);
        } else {
          inp.value = "";
        }
      });
    const pill = $("deadband-status");
    pill.classList.remove("ok", "bad", "warn");
    if (!payload) {
      pill.textContent = "loading...";
    } else if (!payload.available) {
      pill.textContent = "node offline";
      pill.classList.add("bad");
    } else {
      pill.textContent = "live";
      pill.classList.add("ok");
    }
  }
  async function loadWrenchDeadband() {
    try {
      deadbandCache = await api("/api/wrench_deadband");
      syncDeadbandInputs(deadbandCache);
    } catch (e) {
      deadbandCache = null;
      syncDeadbandInputs(null);
    }
  }
  function collectDeadbandEdits() {
    // Build {name: [x,y,z]} for any *row* that has at least one dirty
    // cell, falling back to the cached values for the cells the user
    // didn't touch.  This way a single-cell edit doesn't accidentally
    // zero the other two axes (which would happen if we naively only
    // sent dirty cells -- a SetParameters call for force_deadband
    // requires the full 3-element vector).
    const cached = {};
    if (deadbandCache && Array.isArray(deadbandCache.deadbands)) {
      for (const d of deadbandCache.deadbands) {
        if (Array.isArray(d.value)) cached[d.name] = d.value.slice();
      }
    }
    const dirtyByName = {};
    document.querySelectorAll(
        "#deadband-table input.deadband-num.dirty")
      .forEach((inp) => {
        const name = inp.dataset.name;
        const idx  = parseInt(inp.dataset.axis, 10);
        if (!dirtyByName[name]) dirtyByName[name] = [];
        dirtyByName[name].push(idx);
      });
    const updates = [];
    for (const name of Object.keys(dirtyByName)) {
      const vec = (cached[name] || [0, 0, 0]).slice();
      let problem = null;
      document.querySelectorAll(
          "#deadband-table input.deadband-num[data-name='" + name + "']")
        .forEach((inp) => {
          const idx = parseInt(inp.dataset.axis, 10);
          if (inp.value === "" || inp.value == null) {
            problem = name + "[" + idx + "] is empty";
            return;
          }
          const v = parseFloat(inp.value);
          if (isNaN(v)) {
            problem = name + "[" + idx + "] not a number";
            return;
          }
          if (v < 0) {
            problem = name + "[" + idx + "] must be >= 0";
            return;
          }
          vec[idx] = v;
        });
      if (problem) return { error: problem };
      updates.push({ name, value: vec });
    }
    return { updates };
  }
  async function saveWrenchDeadband() {
    const built = collectDeadbandEdits();
    if (built.error) { toast(built.error, "bad"); return; }
    if (!built.updates.length) {
      toast("no deadband changes to apply", "warn"); return;
    }
    try {
      const r = await api("/api/wrench_deadband", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ deadbands: built.updates })
      });
      const failed = (r.results || []).filter((p) => !p.successful);
      if (r.ok && !failed.length) {
        toast("applied " + r.updated + " deadband(s)", "ok");
        clearDeadbandDirty();
      } else {
        const why = failed.map((p) => p.name + ": " + p.reason).join("; ");
        toast("gravity-comp rejected: " + (why || r.message), "bad");
      }
      await loadWrenchDeadband();
    } catch (e) {
      toast("save deadband failed: " + e.message, "bad");
    }
  }
  function resetDeadbandEdits() {
    clearDeadbandDirty();
    syncDeadbandInputs(deadbandCache);
  }
  document.querySelectorAll("#deadband-table input.deadband-num")
    .forEach((inp) => {
      inp.addEventListener("input", () => markDeadbandDirty(inp));
      inp.addEventListener("keydown", (ev) => {
        if (ev.key === "Enter") {
          ev.preventDefault();
          saveWrenchDeadband();
        } else if (ev.key === "Escape") {
          inp.classList.remove("dirty");
          syncDeadbandInputs(deadbandCache);
        }
      });
    });
  $("btn-deadband-save").addEventListener("click", saveWrenchDeadband);
  $("btn-deadband-reset").addEventListener("click", resetDeadbandEdits);
  $("btn-deadband-reload").addEventListener("click", loadWrenchDeadband);

  refresh().then(() => {
    loadWrenchDeadband();
    setInterval(tick, 200);
  });
  setInterval(refresh, 3000);
  // Re-poll the deadband once every 5 s in case it gets changed by an
  // external ``ros2 param set`` call.  Cheap (1 GetParameters round-trip).
  setInterval(loadWrenchDeadband, 5000);
})();
