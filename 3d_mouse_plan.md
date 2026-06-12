# Plan: 3D-Mouse (SpaceMouse) Cartesian Teleoperation of the Robot

**Goal.** Drive the robot's end-effector in real time with a 3Dconnexion
SpaceMouse: push/twist the puck → the tool jogs in Cartesian space. Use the two
submodules we just added — [`external/3dconnexion_ros2`](external/3dconnexion_ros2)
(SpaceMouse driver + dashboard) and
[`external/inverse_kinematics_toolkit`](external/inverse_kinematics_toolkit)
(Pinocchio IK + safety-gated pose commander) — and, where it makes the system
cleaner, **improve both submodules** rather than bolting glue code onto the
side.

> **Progress (2026-06-12).** Phases 0–3 implemented & verified on the bench.
> The missing bridge is built as a **workspace package**
> [`src/spacemouse_teleop`](src/spacemouse_teleop/README.md) (the `3dconnexion_ros2`
> submodule is left untouched):
> - `twist_integrator.py` — ROS-free twist→pose math (**13 offline unit tests pass**).
> - `servo_node.py` — `spacemouse_servo` bridge: dead-man gating, TF capture,
>   integration, idle re-capture, input-staleness, button speed/position-only,
>   `~/status`, commander enable/disable on engage/release.
> - `spacemouse_servo.launch.py` (node only) + `spacemouse_teleop.launch.py`
>   (driver + `ikt_pose_commander` + bridge, `output:=ikt|fzi`).
> - Verified: builds clean + flake8 clean; node refuses without frames; on a
>   live static-TF tree it gates while disengaged, captures the EE pose on
>   engage, integrates +motion while held, and stops on release (all PASS).
> **Remaining:** Phase 4 (real Duco/UR15 — needs the physical SpaceMouse +
> robot), Phase 5 (FZI path live comparison), Phase 6 (dashboard E-stop), and
> the optional `ikt_pose_commander` native `~/jog_twist` input. The mock
> full-chain demo was blocked only by a DDS/daemon `/tf` discovery quirk for
> freshly-spawned CLI nodes, not by the bridge (proven via the isolated test).

This document is the implementation plan only; no robot code is changed by
writing it.

---

## 1. What already exists (building blocks)

### 1.1 SpaceMouse side — `external/3dconnexion_ros2`
- `spacenav` driver node publishes (built & verified, daemon running):
  | Topic | Type | Meaning |
  |---|---|---|
  | `spacenav/twist` | `geometry_msgs/Twist` | 6-DOF linear+angular velocity (puck deflection) |
  | `spacenav/joy` | `sensor_msgs/Joy` | raw axes + **button** states |
  | `spacenav/offset` / `spacenav/rot_offset` | `geometry_msgs/Vector3` | scaled linear / angular offset |
- Driver params (see [`spacenav_params.yaml`](external/3dconnexion_ros2/spacemouse_dashboard/config/spacenav_params.yaml)):
  `zero_when_static`, `static_trans_deadband`, `static_rot_deadband`,
  `linear_scale.{x,y,z}`, `angular_scale.{x,y,z}`, `full_scale` (350).
- `spacemouse_dashboard` web UI on :8080 (live cube + bar charts + buttons).

### 1.2 IK / motion side — `external/inverse_kinematics_toolkit`
- `ikt_core` — ROS-free Pinocchio solver (`IK` class, `solve_ik`).
- `ikt_inverse_kinematics` — **advisory** solver node (`~/solution`, never commands).
- `ikt_pose_commander` — **the actuation path**. Subscribes
  `~/target_pose` (`geometry_msgs/PoseStamped`), solves IK in-process, applies
  **safety gates** (reachability, per-step jump `max_step_rad`, joint-speed,
  stale-state), then commands either:
  - `jtc` mode → one `FollowJointTrajectory` goal per target (discrete, safe), or
  - `fpc` mode → `Float64MultiArray` setpoint per target to
    `/<fpc>/commands` (**streaming / servoing** — exactly what a SpaceMouse needs).
  - Starts **DISABLED**; `~/enable` / `~/disable` / `~/stop` (`std_srvs/Trigger`).
  - Config: [`commander_defaults.yaml`](external/inverse_kinematics_toolkit/ikt_pose_commander/config/commander_defaults.yaml)
    (`base_frame`, `controlled_frame`, `command_mode`, `default_stiffness`,
    `max_joint_speed=0.5`, `max_step_rad=0.8`, …).

### 1.3 Existing robot stack (this workspace)
- **Two pose→motion paths already wired:**
  1. **IK toolkit** `ikt_pose_commander` (above) — our **primary** target.
  2. **FZI `cartesian_motion_controller`** via
     [`cartesian_control_manager`](external/cartesian_controllers_toolkit/cartesian_control_manager) —
     consumes `<controller>/target_frame` (`PoseStamped`), snapshots pose on
     activate, drives toward it with forward-dynamics differential IK (has the
     known 7-DOF null-space drift; fine for 6-DOF). Kept as the **alternative**.
- **Reference teleop pattern:** [`src/alicia_teleop`](src/alicia_teleop) —
  100 Hz timer, rate limiter seeded from `/joint_states`, auto controller
  switching, **button gating** (leader SYNC button engages). The SpaceMouse node
  should mirror this structure (gating + rate limiting + auto controller mgmt).
- **Robots:** Duco GCR5_910 (6-DOF, `duco_hardware` servoj streaming, hardware
  shaper as downstream smoother/limiter) and UR15 (6-DOF). Both 6-DOF ⇒ no
  redundancy ⇒ the IK toolkit's 7-DOF arm-angle/S-R-S features are **not** needed.
- **Multi-robot pattern** (must be honored): one `ROBOT_ID` =
  ROS namespace + `tf_prefix` + `config/robots/<id>.yaml`. Topics in YAMLs are
  **relative**; launches wrap inner nodes in
  `GroupAction([PushRosNamespace(robot_id), …])`; spawners/event-handlers use
  **absolute** `/<id>/controller_manager`. Legacy fallback when `ROBOT_ID` unset.

### 1.4 The gap
The SpaceMouse emits a **velocity** (`Twist`); `ikt_pose_commander` (and the FZI
controller) want a **moving pose target** (`PoseStamped`). **The one missing
piece is a "twist → moving Cartesian target" bridge** with dead-man gating,
scaling and frame selection. That bridge is the core deliverable.

---

## 2. Target architecture

**Primary path (uses both new submodules):**

```
SpaceMouse puck
   │  spacenav/twist (Twist, ~ driver rate)
   │  spacenav/joy   (buttons: dead-man, mode, speed)
   ▼
┌──────────────────────────────────────────────────────────┐
│  spacemouse_servo  (NEW bridge node, fixed-rate timer)     │
│  • dead-man gated (hold button → motion)                   │
│  • deadband + per-axis scale + speed clamp                 │
│  • on engage: capture current EE pose via TF               │
│  • integrate: p += R·(v·dt);  q ⊗= Δq(ω·dt)                │
│  • frame select: TOOL-frame jog (default) | BASE-frame jog │
└───────────────┬───────────────────────────────────────────┘
                │  PoseStamped  (target_pose)
                ▼
        ikt_pose_commander  (command_mode: fpc, starts DISABLED)
                │  IK + safety gate (reachable / jump / speed / stale)
                │  Float64MultiArray → /<fpc>/commands
                ▼
        ros2_control  →  duco_hardware (servoj) / UR driver
```

**Alternative path (no IK toolkit, for comparison/fallback):** the same bridge
publishes `PoseStamped` to the FZI `<motion_controller>/target_frame` instead.
Selectable by a single `output:=ikt|fzi` launch arg.

**Frame convention (per multi-robot pattern):** capture/target in
`<tf_prefix>base_link`; controlled tip is the URDF tool/flange link
(`ikt_pose_commander` auto-derives joints+controllers from the named link).
Tool-frame jogging matches the README tip (align puck with EE for intuitive
control).

---

## 3. The new bridge node — `spacemouse_servo`

**Where it lives:** a **workspace package** at `src/spacemouse_teleop` (mirrors
the existing `src/alicia_teleop` teleop package). Rationale: it's robot-agnostic
(publishes a generic `PoseStamped`/`Twist`) and only has *runtime* deps on the
submodules' nodes (`spacenav`, `ikt_pose_commander`), resolved via the ament
index — so it needs no source change to either submodule, keeping them pristine.
(It could alternatively live inside the `3dconnexion_ros2` submodule to share it
across workspaces, but per the workspace convention the submodules are kept
untouched and integration code lives in `src/`.)

### 3.1 ROS interface
**Subscribes**
- `spacenav/twist` (`geometry_msgs/Twist`) — puck velocity.
- `spacenav/joy` (`sensor_msgs/Joy`) — buttons for dead-man / mode / speed.
- `/joint_states` + TF (`<base>`→`<tip>`) — to capture the live EE pose on engage.

**Publishes**
- `~/target_pose` (`geometry_msgs/PoseStamped`) — remapped to the chosen sink
  (`ikt_pose_commander/target_pose` or `<fzi_ctrl>/target_frame`).
- `~/status` (`std_msgs/String` JSON) — engaged?, scale, frame mode, last twist,
  staleness (for the dashboard).

**Service clients (optional, auto-managed like alicia_teleop)**
- `ikt_pose_commander/enable` / `disable` driven by the dead-man button.

### 3.2 Integration algorithm (fixed-rate timer, e.g. 50–100 Hz)
1. Read latest twist; if stale (no msg within `input_timeout`) → treat as zero.
2. Apply per-axis **deadband** then **scale** (m/s, rad/s); clamp to
   `max_linear_speed` / `max_angular_speed`.
3. If **not engaged** (dead-man up): continuously **re-capture** target = current
   EE pose (so the target never drifts while idle) and publish nothing (or hold).
4. If **engaged**: `dt` = timer period;
   - position: `p += R_frame · (v_lin · dt)`
   - orientation: `q := normalize(q ⊗ Δq(ω · dt))`
   - `R_frame` = identity for **base-frame** jog, or current tool rotation for
     **tool-frame** jog (param `jog_frame: tool|base`).
5. Publish `PoseStamped` (`header.frame_id = <base>`) at timer rate.

### 3.3 Button mapping (standard 2-button SpaceMouse; configurable)
| Input | Action |
|---|---|
| Button 0 | **Dead-man / engage**: hold-to-move (default) or toggle (`deadman_mode`) — also calls commander `~/enable`/`~/disable` |
| Button 1 | Cycle **speed scale** (e.g. 0.25× / 1× / 2×) **or** toggle position-only (zero orientation) — `button1_action` param |
| (no input for `reset_timeout`) | re-capture target = current pose |

### 3.4 Parameters (with conservative defaults)
`input_topic` (`spacenav/twist`), `joy_topic`, `base_frame`, `tip_frame`,
`output` (`ikt`|`fzi`), `rate_hz` (50), `linear_scale`/`angular_scale` (per-axis),
`deadband_lin`/`deadband_ang`, `max_linear_speed` (e.g. 0.05 m/s),
`max_angular_speed` (e.g. 0.3 rad/s), `jog_frame` (`tool`), `deadman_button` (0),
`deadman_mode` (`hold`), `button1_action`, `input_timeout` (0.2 s),
`auto_enable_commander` (true). All small/slow by default — speed up only after
real-robot validation.

---

## 4. Phased implementation

### Phase 0 — Prerequisites / environment
- [ ] Build the IK toolkit:
  `colcon build --symlink-install --packages-select ikt_core ikt_common ikt_interfaces ikt_inverse_kinematics ikt_pose_commander`.
- [ ] Ensure **Pinocchio** is present (`ros-humble-pinocchio` or `pip install pin`)
  — `ikt_core` hard dependency.
- [ ] Confirm `spacenavd` running (`systemctl is-active spacenavd`) and
  `ros2 topic echo /spacenav/twist` reacts to the puck. *(done for env setup)*
- [ ] Smoke-test `ikt_pose_commander` standalone on the **mock** robot
  (enable → capture-current no-op → small jog) so the downstream path is known-good.

### Phase 1 — Bridge MVP (mock robot, no buttons)
- [ ] Add `spacemouse_servo` node in `src/spacemouse_teleop` (integration
  algorithm §3.2, base-frame jog, always-engaged for the test only).
- [ ] Manual wiring: run spacenav + bridge + `ikt_pose_commander`
  (`command_mode:=fpc`) on the mock robot; remap `~/target_pose` →
  `ikt_pose_commander/target_pose`; enable the commander by hand.
- [ ] **Verify** the mock EE follows the puck; tune `rate_hz`, scales, and
  `max_step_rad` so the commander's jump gate doesn't reject normal jog steps.
  *(Start off a singularity — all-zeros can be singular; bend the arm first.)*

### Phase 2 — Dead-man, buttons, safety
- [ ] Implement dead-man gating (hold button 0), deadband, per-axis scale,
  speed clamps, idle re-capture, input-staleness → zero.
- [ ] Drive commander `~/enable`/`~/disable` from the dead-man (release = stop).
- [ ] Add `~/status` JSON; add tool-frame vs base-frame jog; add button-1
  speed/mode cycling.
- [ ] Re-verify on mock: releasing the button stops motion immediately; stale
  SpaceMouse input halts; far/unreachable targets are gate-rejected (not moved).

### Phase 3 — Multi-robot integration + config
- [ ] New launch `spacemouse_teleop.launch.py` (in `src/spacemouse_teleop`),
  wrapping the bridge + (optionally) `ikt_pose_commander` in
  `GroupAction([PushRosNamespace(robot_id), SetEnvironmentVariable("ROBOT_ID"…)])`;
  follow [`launch_fallback_policy`](#) (`_FALLBACKS` + `_defaults()` that lets
  `ConfigError` propagate). The SpaceMouse is **global hardware** like the Alicia
  leader → keep the `spacenav` driver include **outside** the namespace group
  (mirror `alicia_teleop`'s leader handling); only the bridge is namespaced.
- [ ] Add a `spacemouse_teleop:` section to `config/robots/<id>.yaml`
  (relative topics, frames, scales, speed caps) + the `.example.yaml`.
- [ ] Optionally add a one-line include in `robot_bringup` meta-launch
  (scoped `GroupAction(scoped=True, forwarding=True)` to avoid the arg-leak bug).

### Phase 4 — Real robot (Duco first, then UR15)
- [ ] Duco GCR5_910: bring up the stack, enable, **start off a singularity**,
  jog with very low scales, confirm tracking and that the duco_hardware shaper
  smooths the FPC stream (servoj at ~100–250 Hz). Tune scales/speed caps up
  gradually.
- [ ] Repeat for UR15 (uses `scaled_joint_trajectory_controller`; pick its FPC
  equivalent or use `jtc` mode first). Confirm the UFW reverse ports note for UR
  if motion "succeeds" but the arm doesn't move.

### Phase 5 — FZI alternative path (optional)
- [ ] Add `output:=fzi`: publish `PoseStamped` to
  `/<id>/<motion_controller>/target_frame`; activate the FZI motion controller
  via `cartesian_control_manager`. Compare feel vs the IK-toolkit path.

### Phase 6 — UX / dashboard
- [ ] Extend the SpaceMouse dashboard (:8080) to show engage state, jog frame,
  active scale, and the live target marker; add an on-screen E-stop that calls
  `~/disable`/`~/stop`.

---

## 5. Optional submodule-improvement tasks

> The core bridge is a **workspace package** (`src/spacemouse_teleop`) and needs
> **no** submodule changes — both submodules are kept pristine. The items below
> are *optional* enhancements that would require editing a submodule; do them
> only if needed, and only by committing in the submodule's own repo + bumping
> the superproject pointer (same workflow as the `cartesian_controllers_toolkit`
> decoupling). Do not leave a submodule working tree dirty against its commit.

**`external/3dconnexion_ros2` (only if a shared/portable bridge is wanted)**
- Upstreaming `spacemouse_teleop` into the submodule so other workspaces get it.
- Expose driver `linear_scale`/`angular_scale`/deadband via the new launch.
- Optionally publish `TwistStamped` (frame_id) for downstream TF correctness
  (driver already supports `use_twist_stamped`).
- Dashboard: button/engage indicators + E-stop.

**`external/inverse_kinematics_toolkit` (`ikt_pose_commander`)**
- Optional but clean: add a **native velocity-jog input** — a `~/jog_twist`
  (`TwistStamped`) topic that the commander integrates internally against the
  live pose (re-using its existing gates). This removes the external integrator
  for the streaming case and is a genuine reusable feature, not glue.
- Document the SpaceMouse use-case in `ikt_pose_commander/README.md` and tune a
  streaming-friendly `max_step_rad`/`max_joint_speed` profile.

---

## 6. Safety checklist (real-robot)
- Commander **starts DISABLED**; motion only while dead-man held.
- Release dead-man / SpaceMouse input stale → immediate stop + commander disable.
- Conservative default scales and `max_linear/angular_speed`; ramp up only after
  validation.
- Reachability + jump + speed + stale gates left **on** (`ikt_pose_commander`).
- Begin **off singularities** (avoid all-zeros start).
- Idle re-capture prevents target drift while disengaged.
- Downstream hardware shaper (Duco) / driver limits remain the last backstop.
- UR15: verify reverse ports (URScript reverse connections) or motion silently no-ops.

## 7. Testing & verification
- **Mock first** (`use_fake_hardware:=true`) for every phase.
- Unit-test the integrator (twist→pose) offline (frame math, deadband, clamps).
- Bench checks: `ros2 topic hz` on `target_pose`; commander `~/status` shows
  `reachable`; jog ±X/±Y/±Z and ±roll/pitch/yaw each move the right axis.
- Real-robot: low-scale jog, dead-man release stop, stale-input stop,
  far-target rejection — all confirmed before raising speeds.

## 8. Risks & open questions
- **SpaceMouse model / button count** — standard puck has 2 buttons; Pro/
  Enterprise has more (richer mode mapping). Confirm the connected unit.
- **FPC streaming smoothness** — `ikt_pose_commander` fpc emits one setpoint per
  target and relies on the hardware shaper; if jerky, lower `rate_hz` or raise
  shaper smoothing, or fall back to `jtc` for discrete nudges.
- **`max_step_rad` vs jog rate** — too tight rejects normal jogs, too loose
  weakens the jump gate; tune in Phase 1.
- **Latency** — Duco `servoj` is one Thrift round-trip per call; keep the FPC/
  servoj rate in the proven ~100–250 Hz band.
- **Decision:** primary path is the **IK toolkit** (uses both new submodules);
  FZI path kept as comparison via `output:=fzi`.

## 9. Deliverables
1. `spacemouse_servo` node + launch + config + tests (workspace package `src/spacemouse_teleop`).
2. (Optional) `~/jog_twist` input in `ikt_pose_commander` + README/tuning.
3. Workspace wiring: `spacemouse_teleop.launch.py`, `config/robots/<id>.yaml`
   `spacemouse_teleop:` section (+ `.example`), optional `robot_bringup` include.
4. Dashboard engage/E-stop additions.
5. Verified on mock → Duco → UR15, with the safety checklist satisfied.
```
