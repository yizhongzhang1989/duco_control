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
    const kind = entry ? entry.kind : "--";
    $("kind-pill").textContent = "kind: " + kind;
    setCommandPanel(kind, !!ctl.engaged);
  }

  function setCommandPanel(kind, engaged) {
    $("cmd-kind-pill").textContent = "kind: " + kind;
    const showForce = (kind === "force");
    const showJog   = (kind === "motion" || kind === "compliance");
    $("cmd-force").hidden               = !showForce;
    $("cmd-motion-compliance").hidden   = !showJog;
    // Jog buttons act on the *engaged* controller -- if not engaged,
    // disable them to make intent obvious.  (Pre-engage targets get
    // clobbered by FZI's on_activate snapshot anyway.)
    const dis = !engaged;
    document.querySelectorAll("#cmd-motion-compliance button.jog")
      .forEach((b) => { b.disabled = dis; });
    $("btn-snap-target").disabled = dis;
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
      toast("target snapped to current pose", "ok");
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

  refresh().then(() => setInterval(tick, 200));
  setInterval(refresh, 3000);
})();
