# ll_v3 W1 warm-up — substrate implementations, validators, re-measured envelopes

Status: **W1 CLOSED — all 8 substrate skills independently verified
(5 accept, 3 restrict). W2b/W2c/W2d closure work implemented. W2d fixes the object
displacement (0.001 m pre-close, zero contact bursts) with a live target and a
folded pre-shape, but the reach from the tucked pose fails the QP local branch
(pad 1.08/1.66 m, base drift 1.22 m). No batch 1 started; the warm-up gate
remains BLOCKED.**

- Sources: `work/ll_v3/substrate/` (8 files).
- Validator evidence: `work/ll_v3/reports/w1_validation_*.json`.
- Raw runs: `work/ll_v3/reports/w1_raw/`.
- Round-0 inputs used: `experiments/ll_v3/calibration.json` /
  `docs/control_contract.md` (ramp curve, 12-D/homie adoption, 14-D retired,
  17-D alternative only through the authorized handover, grasp proxy, G10).

## 1. Substrate verdicts

| capability | substrate source | verdict | measured summary | binding constraint |
|---|---|---|---|---|
| `base_navigate_to_standoff` | `base_standoff_servo.py` | **accept** | S=0.25/0.20/0.15: standoff error 0.0157/0.0152/0.0160 m; tilt 4.19/1.87/1.60 deg; both guards trip when forced (`tilt_guard`, `fingertip_guard`) | none |
| `ee_pose_ik` | `arm_pose_ik.py` | **accept** (contract) | solver `mink-qp`; waist `homie` exogenous; joint margin 0.3705 rad generic / 0.975 rad custom; ramp 0.35 s + 0.2 s hold; pad error 0.0945 / 0.0781 m | none for the contract; pad error is the W2 gate (base drift) |
| `grasp_close_contact` | `hand_close_contact.py` | **restrict** | close reached 0.8 rad in 9 steps; close-phase contacts L0/R1 (not bilateral); hold stable; object disturbance 0.0684 m | `object_disturbance`: the preceding `arm_pose_ik` displaced the object 0.093 m; restrict credit to a pregrasp pad error <= 0.015 m |
| `grasp_open_release` | `hand_open.py` | **restrict** (final) | open-target fix confirmed: direct two-pad run `open_joint_error 0.057 rad`, ok true; delegated run index/middle open from 0.64-0.69 to 0.056-0.06 rad | `no_bilateral_grasp` (upstream W2; the release branch was never exercised) and a failure-path `result_serialization` restriction (`np.bool_` on the non-ok path; the success path serializes) |
| `object_pose_tracking` | `object_pose_track.py` | **accept** | static pose error 0.00052 m; post-motion 0.00066 m; tracking gap 0.03367 m exactly matches the displacement; AABB matches `probe object` to <0.001 m | none |
| `safety_limit_monitor` | `safety_guard.py` | **accept** (after fix) | normal state ok, margin 0.52 rad; forced violation aborts with `joint_margin` (-0.08 rad) and `penetration` (0.00205 m); read-only verified | none (first verdict restricted: hand joints resting on range bounds were flagged; fixed by excluding `_hand_` joints from the margin scan) |
| `failure_visualization` | `debug_visual.py` | **restrict** | onset step 0 (t 0.0), end step 476 (t 9.52); stills at both times; validator visual finding: t=0 tabletop slab/no arm engaged, t=9.52 green arm extended to the drill; no predicate read | `visual_finding_not_populated`: the driver reports the window/frames; the downstream reader populates the visual finding |
| `control_authority_handover` | `authority_handover.py` | **accept** | roll gains before 0.326 → ik 0.987 → after -0.256; owner restored `homie`; HANDOFF events recorded; max tilt 7.17 deg < 8 | none; scope note: the lower-body `policy|hold` owner is not switched (waist channel only) |

The `hand_close_contact` restriction is a real round-0 lesson: the close-phase
contact is not the reliable grasp signal (the lift-phase bilateral contact is).
The substrate now reports both; the close law itself passes when the pregrasp
is valid.

## 2. Re-measured reach envelope (mink-QP executed vs DLS preview)

At the 05 standoff placed by `base_standoff_servo` (0.15 m AABB; achieved
0.166 m, xy error 0.0096 m, tilt 1.6 deg):

| measurement | cells | <= 15 mm | pad error range | note |
|---|---:|---:|---|---|
| DLS preview (`ik_preview`, full grid) | 40 | 0 | 0.017–0.208 m | conservative screen |
| executed QP, one side, `dur=0.35` | 10 | 0 | 0.127–0.420 m; margins -0.126–0.040 | ramp too short for the 14-joint QP |
| executed QP, one side, `dur=1.0` | 10 | 0 | 0.099–0.339 m; margins -0.004–0.082 | high-z cells; one-side solve |
| executed QP, two arms, task target, `dur=1.0` | 1 | 1 | 0.0556 / 0.0522 m | servo pose; base drift during the ramp |
| round-0 executed QP, two arms, task target, `dur=1.0` | 1 | 1 | **0.0146 m** | hand-rolled pose; the only <= 15 mm result |

**Authoritative measurement: the executed mink-QP at the actual task target,
with a base pose that is held during the ramp.** The DLS preview is not the
feasibility gate; it overstates the residual and is retained only as a
reachability screen. The 15 mm task-target gate was met at the round-0
hand-rolled pose (0.0146 m) but not at the servo pose (0.052–0.056 m): the
servo's lateral pose and the 0.055 m base drift during the arm ramp are the
binding constraints. The single-joint ramp curve (0.35 s for 1.0 rad) does not
transfer to the 14-joint QP; the QP needs its own measured ramp (round-0: 1.0 s)
or a base-lock during the ramp.

## 3. Per-task standoff envelope with the servo

The servo replaced the hand-rolled `move_base` approach. The 05 minimum is
0.233 m root / 0.136 m AABB, pad error 0.0059 m, margin 0.1719, tilt 3.66 deg
(round 0). The re-run for the other tasks at 0.35/0.25/0.20 m AABB with the
servo and explicit task targets:

| task | target | servo result | task-target arm result | binding |
|---|---|---|---|---|
| 01_box_lift_floor | box AABB centre z=0.3275, crouch 0.55 | FAILED at all three: `tilt_guard_after_height`, 16.29 deg | not reached | base height / crouch |
| 02_push_button | button, right-hand press target | OK 0.360/0.268/0.213 m; xy 0.010/0.018/0.013; tilt 1.3/1.8/1.6 | one-hand press path not measured (the one-side `arm_pose_ik` call returned no result; needs the press skill / one-side support) | target/press path |
| 03_box_lift | box centre z=0.8838 | OK 0.370/0.262 | pad 0.707 / 0.213 m | `pad_error` |
| 04_push_shelf_forward | shelf side, z=0.7294 | OK 0.362/0.237; 0.20 trips `fingertip_guard` | pad 0.103 / 0.080 m | `pad_error` |
| 06_drill_pnp | drill z=0.94 | OK 0.368/0.257/0.214 | pad 0.372 / 0.106 / 0.0697 m | `pad_error` |
| 07_box_table_to_shelf | box centre z=0.76 | OK 0.251/0.232; 0.35 did not return | pad 0.697 / 0.290 m | `pad_error` |
| 08_pick_drill_from_holder | drill z=0.8966 | OK 0.368/0.256/0.220 | pad 0.364 / 0.064 / 0.054 m | `pad_error` |
| 09_obstacle_aware_pick_drill | drill z=0.94, via [0.5,-0.9] | FAILED all three: `tilt_guard`, achieved AABB 1.287 m | not reached | obstacle path / tilt |

**Result:** the servo fixes the **approach** (no 20–125 deg tilt; the servo
contract passes for 02/03/04/06/07/08 at the tested standoffs) but the
**task-target pad error** remains 0.054–0.70 m, binding `pad_error`. The
task-target reach is still the blocker: the base pose that meets the AABB
standoff is not the pose that reaches the task target to 15 mm. The 05
task-target result is the only <= 15 mm measurement, and only at the round-0
hand-rolled pose (0.0146 m), not at the servo pose (0.055 m). The per-task
targets (02 button, 04 shelf side, 07 box, 08 holder drill) are recorded here
and in `w1_raw/standoff_servo_summary.json`; the servo is necessary but not
sufficient for the per-task envelope.

## 4. W2 integration benchmark (unchanged gate) — W2b closure work

05 reach → close → lift, standoff 0.15 AABB, lift 0.10 m (G10 cap).

**W1 W2 result:** FAILED at the IK stage, `pad_error` (left 0.0187 m, right
0.0727 m; target L [1.655, 0.110, 0.94] / R [1.655, -0.030, 0.94]).

**W2b implemented (option a):**

- (a) `arm_pose_ik_baselock.py`: re-solves the mink QP against the live base
  every control step (`dur=0.02`, up to 50 steps, stop at site error <= 0.012)
  so the base drift is compensated in the arm target instead of a stale ramp.
- (b) `base_standoff_servo_v2.py`: the base lateral pose follows the task-target
  palm midpoint (`lateral_target = object y`), not only the object AABB.
- Chain: `w2b_reach_close_lift.py`.

**W2b result:** the site pad error drops from 0.055 m to **0.0227 / 0.0230 m**
(still > 0.015 m); joint margin 0.8275 rad; base drift 0.0445 m; the chain stops
at the IK stage. The unchanged gate **FAILS**; no close/lift credit.

**Error budget (measured, one named binding source):**

| source | value | note |
|---|---:|---|
| base drift | 0.0445 m | still present, but the per-step re-solve compensates most of it |
| solver residual (`ik_err`) | 0.0001 m | negligible |
| **target-frame offset** | **0.0673 m** | **dominant**: pad normal error ~53 deg over the 8.4 cm pad lever (`0.0838 * sin(53 deg)`) |
| wrist margin | 0.826 rad | not binding |
| executed site pad error | 0.0227 / 0.0230 m | gate 0.015 m |

**Binding constraint: `pad_error` (site 0.0230 m); dominant source:
`target_frame_offset` 0.0673 m.** The W2b fixes addressed the base pose and the
base drift, but the remaining error is dominated by the hand/pad orientation
(`orientation=keep` leaves a ~53 deg pad normal error). A W2c with an explicit
grasp orientation target (or a pad-position servo) is the next candidate; the
criteria were not relaxed and the gate is not re-labelled.

### W2c (explicit grasp orientation) — pose gate PASSES against the computed target; overall gate FAILS on the stale target

`w2c_reach_close_lift.py` + `base_standoff_servo_v2.py` + `arm_pose_ik_baselock.py`:

- **2a, explicit grasp orientation:** the pad normals are derived from the live object/base frame (left pad normal -y, right +y, opposing; hand forward along the base→object approach axis), via per-side `R_body` rotations; the reach keeps the per-step baselock.
- **Result (against the target computed before the standoff):** site pad error **left 0.00557 / right 0.00369 m** (PASS <= 0.015); pad normal error **left 4.21 / right 6.21 deg**; base drift 0.00397 m; solver residual 0.0001 m; target-frame offset 0.00906 m; wrist margin 0.058 rad. The 53 deg orientation residual from W2b is fixed.
- **2b, target correction:** not needed — the first baselock reach passed the site gate; the corrected retry was not invoked.
- **Overall W2 gate: FAIL.** Bilateral hand-object contact after the lift is **0/0**, and the safety guard trips on **penetration 4.72 mm** (`bottle_collider_box` vs `robot0_g19_col`/`robot0_g27_col`, a wrist/link against the moved drill). The object rise 0.068 m passes.

**Dominant source: object displacement during the standoff — 0.146 m.** The drill moved from `[1.739, 0.040, 0.80]` before the standoff to `[1.593, 0.081, 0.81]` at the close; the grasp target was computed before the servo, so it is stale by ~0.146 m. The 0.0056/0.0037 m site error is against that stale target; the object-relative pose error is ~0.146 m. The 0.15 m AABB standoff is too close for the robot body on the 05 drill (the round-0 0.233 m root / 0.136 m AABB "minimum reachable" was likewise measured after the drill had been displaced ~0.09 m).

**Updated source-decomposed error budget (site vs the computed target):**

| source | value | note |
|---|---:|---|
| **object displacement during standoff** | **0.146 m** | **dominant; target stale; actual grasp misses (0/0 contacts)** |
| base drift | 0.00397 m | compensated by the per-step baselock |
| solver residual | 0.0001 m | negligible |
| target-frame offset | 0.00906 m | fixed by the 2a orientation (was 0.0673 m) |
| wrist margin | 0.058 rad | just above 0.05; the wrist/link penetrates the moved drill (4.72 mm) |
| executed site pad error | 0.00557 / 0.00369 m | vs the stale target; gate passes only against that target |

**Binding constraint: `bilateral_contact` (0/0), with the guard `penetration` 4.72 mm; dominant source: `object_displacement_during_standoff` 0.146 m.** The W2d candidate is a **safe standoff that does not disturb the drill** plus a **live-object grasp target** re-derived after the standoff / every baselock step (the W2c target was computed once, before the servo). The criteria were not relaxed; the gate is not re-labelled; batch remains 0.

**Non-blocking cleanups:** (i) `hand_open` failure-path `np.bool_` cast to `bool` applied; (ii) `docs/PROTOCOL.md` `hand_contacts_side` line corrected (it walks the object body tree; `observe()['hand_object_contacts']` is the root-only read).

### W2d (live target + phase-resolved approach) — displacement FIXED, reach from the tuck fails

`w2d_reach_close_lift.py` + `base_standoff_servo_v3.py` (phase trace) + `arm_pose_ik_live.py` (live target) + `arm_tuck.py` (folded pre-shape):

- **Live target:** the grasp target is re-derived from the live object every baselock step. The R_body is computed **once** from the live grasp pose and held; re-deriving the orientation every step made the arm chase its own motion (measured object displacement 1.60 m), so the orientation is fixed.
- **Folded pre-shape:** joint-space retract (`shoulder_pitch=0.5, elbow=0.5, wrist=0.0`; pads at x≈-0.06 behind the base, z≈0.73) during the base approach; the base lateral offsets 0.0/±0.12 were tried.
- **Phase-resolved trace:** the standoff + tuck moves the drill **0.001 m** with **zero contact bursts**; the pre-close displacement is **0.001 m** (PASS <= 0.010) for the 0.0 and +0.12 offsets. The base/body no longer pushes the drill. This is the approach fix the directive asked for.
- **Reach from the tucked pose FAILS:** the QP cannot solve from the retracted local seed. 0.0 offset: pad error 1.08/1.66 m, normal 126.6/53.0 deg, `ik_err` 1.01/1.25, base drift 1.22 m, 51 steps. +0.12: pad 0.94/1.73, normal 68.4/150.3 deg, drift 1.10. -0.12: the arm knocked the drill 0.472 m (pad 0.33/0.72, normal 29.2/67.3 deg).
- Close/lift were not reached; bilateral contact/rise/guard were not measured in this run.

**Per-phase (0.0 offset):**

| phase | object displacement | contacts | site pad error | normal error | base drift |
|---|---:|---|---:|---:|---:|
| tuck + standoff | 0.001 m | 0 (zero contact bursts) | — | — | — |
| live reach from tuck | 0.000 m | table/bottle resting only | 1.08 / 1.66 m | 126.6 / 53.0 deg | 1.22 m |
| close | not reached | — | — | — | — |
| lift | not reached | — | — | — | — |

**Binding constraint: `pad_error` at the reach; dominant source: `tuck_to_reach_qp_branch`** — the retracted pose is a bad local QP seed (`ik_err` ~1.0; the QP cannot find the grasp branch; the arm/base flail). The live target and the pre-close displacement are no longer the failure; the close law is not the failure (W2c close-phase disturbance 0.0033 m; lift worst 0.0679 m; the W2c failure was the stale target + wrist/drill overshoot at t=13.42). The next W2e candidate is an **untuck / pre-grasp** between the base and the live reach (or a multi-seed QP from the tuck) so the QP starts from a reachable local branch while keeping the pre-close displacement <= 0.010 m. Criteria unchanged; gate blocked; batch 0.

**Cleanups:** (i) `hand_open` failure-path flag is now a Python bool via the `True if ... else False` ternary (the sandbox has no `bool` builtin either); the open target remains 0.057 rad in the direct check. (ii) `docs/PROTOCOL.md` `hand_contacts_side` line corrected (it walks the object body tree; `observe()['hand_object_contacts']` is the root-only read).

## 5. Design changes forced by the measurements

1. **Ramp is per-channel.** The measured single-joint curve (0.15/0.15/0.15/
   0.35 s) is valid for single-joint verbs; the 14-joint mink-QP needs its own
   measured ramp (round-0 executed 1.0 s met the gate; 0.35 s did not). Keep
   the single-joint curve for `track_gain`/single-joint transports; use the
   QP ramp + settling hold for `arm_pose_ik`.
2. **Base-lock during the arm ramp.** The QP's base-lock is a scratch-model
   task; the real WBC base drifts ~0.055 m during the arm ramp, which is the
   W2 pad-error binding constraint. The W2 needs a real base-lock (or a longer
   ramp with re-solve against the live base).
3. **Standoff pose must follow the task target, not only the AABB.** The servo
   met its AABB standoff/xy contract but the task-target lateral pose was
   off enough that the right palm missed by 0.073 m. The servo needs a target
   lateral/yaw derived from the task target (palm midpoint), or
   `arm_pose_ik` needs a base-offset correction.
4. **Safety guard scope.** Hand joints resting on their range bounds are
   nominal; the guard now checks the 17 arm/waist joints (the hand joints are
   commanded targets). This is recorded in the validator evidence.
5. **Close-phase contact is not the grasp signal.** `hand_close_contact` is
   restricted to a valid pregrasp; the lift-phase bilateral contact remains the
   reliable grasp check (round-0 proxy).
6. **G10 lift cap.** W2 uses a 0.10 m lift; the round-0 0.15 m lift had a
   0.0406 rad margin. The 0.10 m lift keeps the task rise (>= 0.05 m) with a
   larger margin.

## 6. Graph and gate state

- 33 capabilities, 18 strategies, 9 tasks (unchanged).
- 8 substrate SKILL nodes linked; **8 verified** (5 accept: `base_standoff_servo`,
  `arm_pose_ik`, `object_pose_track`, `safety_guard`, `authority_handover`;
  3 restrict: `hand_close_contact`, `hand_open`, `debug_visual`).
- W1 is CLOSED. The W2b sources (`base_standoff_servo_v2`, `arm_pose_ik_baselock`,
  `w2b_reach_close_lift`) are in `work/ll_v3/substrate/`.
- 05 coverage: 5/19 capabilities verified (the five substrate capabilities in
  its transitive closure); the rest are discovery/warm-up work. The W2b
  baselock/v2 servo are W2-closure implementations, not new capability nodes.
- **Warm-up gate: BLOCKED.** W2b still fails at the IK (`pad_error`, site
  0.0230 m); the dominant source is the target-frame offset (0.0673 m). No
  batch 1.
- Batch counter 0; round counter 0; no discovery batch started.

## 7. Next step

The manager should decide whether to (a) fix the base-lock/standoff-pose issue
and re-run W2 (a W2-closure job, label W2b/W2c), or (b) accept the measured
binding constraint and keep the gate blocked. The 8 substrate sources and the
validators' evidence are ready; the W2 failure is measured and named.

## 8. W2e — true closed loop, base hold, and the unfold failure (measured)

### 8.1 Structural checks the manager asked for
- **The loop advances the sim.** The rewritten `arm_pose_ik_live` solves with
  `reach_box_qp(apply=False)`, clamps the 31-D delta, applies ONE control step
  (`step_goal`, dur 0.02 s) and re-observes. Measured dt = +0.040 s per
  iteration (control_timestep 0.04 s, 25 Hz), sim time continuous
  9.80 -> 11.40 s in a 40-step untuck; the live palm site is re-read after
  every step.
- **No reset, no NaN.** 0 resets / 0 NaNs in every W2e run; sim time never
  runs backward or jumps > 0.05 s. The env package has no NaN/reset handling
  at all (grep: none). The base does move, but in 0.01-0.03 m steps.
- **W2d's 1.22 m "base jump" explained.** The W2d loop called
  `reach_box_qp(..., dur=0.1)` with the then-default `apply=True`, i.e. it
  ramped 0.1 s PER ITERATION: 50 iterations = 5 s of arm motion. The WBC
  walks ~0.25 m/s during a large arm reconfiguration -> ~1.2 m. The on-disk
  `arm_pose_ik_baselock.py` shows the same loop shape (dur=0.02, no `apply`
  arg) plus a final `step_goal`. Not a reset, not a NaN, not a frame bug.
- **Still-arm probe (2 s, post-standoff).** nav=0: drift 0.0354 m, dyaw
  -2.12 deg, all a one-off settle in the first 0.4 s; k20-k49 stay within
  0.0005 m. Corrected deadband hold (deadband 0.02 m, gain 0.05, cap
  0.03 m/s; yaw setpoint deadband 0.01 rad, gain 0.3, cap 0.05 rad/s):
  drift 0.0353 m, dyaw -1.98 deg, nav vx=0, vy<=0.0009, wz<=0.0068 - dormant,
  no oscillation. The WBC does not walk with a static arm.
- **Yaw fix.** wz is a yaw-setpoint error, never `atan2` of the position
  error (that is sign noise near zero and saturated +/-0.15 every few steps
  in probe2). A high-gain hold (gain 1.2, cap 0.2) oscillated: nav saturated
  on all three axes and the base circled 0.7 m. The gentle law above is the
  validated one.

### 8.2 The unfold displaces the drill (stop condition hit)
Every unfold variant from the tucked pose at the standoff displaced the
drill: direct-to-grasp 0.247 m, dx -0.12 0.544 m, up/out pre-grasp
(z=1.02, +/-0.20, dx -0.14) 0.544 m. In the last run:
- object 1.731,0.030,0.800 -> 2.064,-0.399,0.838; the displacement happens
  k029-k060 (sim 11.00-12.24 s), with the live target jumping 2-6 cm/step.
- the base walks 1.48,0.038 -> 1.01,-0.33 (backward + lateral) over the same
  segment; the object moves in the same direction as the base, i.e. the arms
  are carrying it.
- live arm joints sit at/over their limits during the unfold: path margin
  -0.158 rad at k161; final margins left_shoulder_yaw 0.001,
  left_elbow 0.003, left_hand_middle_1 0.000.
- the re-standoff then fails (`standoff`): aabb 0.876 m, lateral 0.296 m,
  yaw 11.6 deg - it cannot recover a 0.5 m lateral drag with a forward-only
  walk law.

### 8.3 Binding constraint and next structural step
The binding constraint is the **unfold at the standoff pose**: the arm
reconfiguration drives the WBC base 0.4-0.5 m while the pads are next to the
drill, and the pads drag it. It is not a QP branch problem (first-step ik_err
0.0038/0.0016) and not the hold. The next structural step is to **back the
base away ~0.4-0.5 m with the arms still tucked, unfold in free space, then
re-approach with the arms in the pre-grasp** (the standoff servo then only
has to walk forward). Gate unchanged; batch 0.

## 9. Substrate contract addendum — clearance must use the full arm chain

`hand_geoms()` returns only the geoms of bodies named `gripper0*hand*`. It
does NOT include the wrist collision geoms (`robot0_g27_col`,
`robot0_g28_col`, `robot0_g34_col`, `robot0_g35_col`), which are the parts
that dragged the 0.226 m-tall object during the W2f/W2g descents. Every
clearance check (pre-descent gate, approach clearance, predictive guard) MUST
use `arm_geoms()` — the read-only helper added 2026-11-16 that returns all
arm-chain geoms (shoulder/elbow/wrist/hand bodies plus the gripper0 hand
geoms) with world positions. `hand_geoms()` alone is insufficient for any
clearance decision near an object.

## 10. Harness defects found during W2e-W2i (recorded)

- `reach_box_qp(apply=False)` was accepted by the signature but never
  honoured: the function ran `_ramp(target, dur)` regardless, applying its
  UNCLAMPED solved target for one control step. Every "read-only" solve in
  the closed loop therefore double-stepped and bypassed the per-step clamp
  (measured dt 0.040 s/iteration = two 0.02 s control steps; wrist motion
  4-5 cm/iteration). Fixed 2026-11-16: the apply=False branch returns
  without ramping and also reports `joint_margin_joint`, `joint_margins`
  and `wrist_solved` (the four wrist geom positions at the solved q).
- `hand_geoms()` excludes the wrist collision geoms; use `arm_geoms()`
  (section 9).
- The object AABB is a body-tree AABB; a wrist geom CENTRE 4-5 cm outside it
  can still be in contact because the collision geom has a radius
  (~0.047 m). All clearance checks must use centre - radius.
- WBC backward authority: `move_base(x=-0.12)` for 4.5 s moved 0.176 m,
  i.e. ~0.04 m/s, not the commanded 0.12 m/s. Back-off distances must be
  budgeted at ~0.04 m/s.

## 11. Tool-contract evidence (W1-adjacent, 2026-11-16)

- Regression test `work/ll_v3/contract_test.py` PASSES:
  `reach_box_qp(apply=False)` leaves qpos, qvel and sim time byte-identical
  (SHA-256 equal before/after) and returns `upper_target`,
  `joint_margin_joint`, `joint_margins`, `wrist_solved`;
  `reach_box_qp(apply=True, dur=0.06)` changes qpos and advances exactly
  0.06 s.
- AST audit of every `Tools` method: no parameter remains in a signature
  without a Name use in the body (the `apply` no-op was the only one and is
  fixed).

## 12. Geometry result, both grasp approaches (stop point)

Side descent (W2i): pre-grasp wrist surface clear (left 0.148 / right 0.269)
but the QP's FIRST solved q predicts right -0.011 m (contact); guard aborts
at k000; 0.0 object displacement.

Top-down from the side pre-grasp (read-only probe): ik_err 1.62 / 1.61 m,
ik_rot 1.85 / 0.96 rad, margin 0.0 at left_shoulder_pitch.

Top-down in FREE space from the tuck (read-only sweep, base-relative
fwd/dy/z):
  fwd 0.35 z 1.16: left ik_err 0.0395 (rot 0.011), right 0.3606 (rot 0.149)
  fwd 0.35 z 1.10: left 0.0262, right 0.3189
  fwd 0.30 z 1.16: left 0.1379, right 0.0885
  fwd 0.30 z 1.10: left 0.2869, right 0.2916
  fwd 0.25 z 1.16: left 0.1357, right 0.1365
  fwd 0.25 z 1.10: left 0.2579, right 0.2535
  margin 0.0 at left_wrist_yaw / left_shoulder_yaw / left_shoulder_pitch.

Common cause: the mink QP is a single-seed local solve and hits joint limits
on the right arm (and on the left for some targets) when a large wrist
reorientation is required. The round-0 DLS `ik_preview` multi-start found
solutions the QP did not. A QP multi-seed from the DLS solution is the
untried lever; it is now justified because the residual is 0.36 m, not the
earlier 3.8 mm.

## 13. W2k-W2l: multi-seed, pre-grasp grid, horizontal approach

- QP multi-seed: `ik_preview` rows now expose `q_named`; `reach_box_qp` has
  `seed_q`; `arm_pose_ik_live` seed_mode='dls' solves unseeded and seeded
  once at the first step (cached), choosing lexicographically: both-sides
  predicted wrist surface >= 0.02, then smaller worst residual, then larger
  margin. Free space: right ik_err 0.3606 -> 0.0041 at fwd 0.30 z 1.16.
- W2k (grasp target): seeded and unseeded both predict right -0.0106 m, so
  the TARGET branch collides; multi-seed cannot help (0.0 displacement,
  guard abort at k0).
- Pre-grasp grid at z=0.94 from the tucked standoff pose (read-only):
  chosen dx -0.16, dy +/-0.22 (left ik_err 0.0424, right 0.0013, predicted
  surfaces 0.147/0.145). Full table in reports/w1_raw/pre_grasp_sweep.json.
- W2l sequence: tuck -> standoff -> backoff -> unfold to the z=0.94
  pre-grasp in free space -> approach -> reposition to the object-relative
  pre-grasp (stop_err 0.05, branch selection) -> horizontal move to the
  grasp (stop_err 0.012, site gate 0.015) -> close -> lift. Wrist guard
  active on both moves.

W2l note: the free-space unfold to the OBJECT-RELATIVE pre-grasp is partial by
design - the base is backed off ~0.176 m, so the pads cannot reach the
object-relative point (worst 0.230 m). The reposition after the approach is
the completing move. The move guards abort on a predicted wrist surface
< 0.010 m; the pre-descent gate at the servo is 0.02 m.

## 14. W2l: joint-space relocation works; the grasp target is infeasible

- seed_force does NOT pin the mink QP branch: seed_dist (max |solved q -
  seed q|) was 1.58-3.94 rad on every step, site error stayed 0.257-0.267,
  first-step seeded ik_err (0.0987) was worse than unseeded (0.0542). The
  QP is a NEAR-TARGET servo, not a branch pinning solver.
- Joint-space interpolation to the DLS winning q (joint_mode in
  arm_pose_ik_live: seed_vec + the existing clamp + the live-surface guard)
  completed the reposition and the pipeline reached the descent with 0.0
  object displacement.
- The grasp target (dx -0.0838, dy 0.07, z 0.94) remains in collision from
  the post-reposition pose: first-step predicted surface left -0.0329 /
  right -0.0032 (seeded == unseeded).
- Hand-orientation grid at the grasp point (pitch x roll +/-30 deg, 15 deg
  steps, 25 rows, seeded QP, post-reposition pose): 0/25 with both wrist
  surfaces >= 0.02 m. Best p+30 r+15 -> 0.0199/0.0071 (ik 0.104/0.001);
  zero orientation -0.0329/0.0125. Tilting costs left reach (ik 0.10-0.15).
- Three approaches measured: side descent (target branch collides
  -0.011..-0.033), top-down (free-space multi-seed reaches 0.007/0.004 at
  fwd 0.30 z 1.16, but infeasible from the side pre-grasp), orientation
  grid (0/25). STOPPED for a manager decision.

## 15. Preview-vs-live contract failure at the standoff (measurement validity)

`preview_contract_probe` (work/ll_v3/substrate/preview_contract_probe.py) at
the tucked standoff pose, target dx -0.126 dy 0.07 z 1.16 orient=down:
- DLS preview err 0.098 / 0.105.
- The DLS q applied DIRECTLY via step_goal for 140 steps (0.02 clamp,
  nav=0) moved the joints only 2-3% of the command (shoulder_roll 1.6 rad
  commanded -> 0.075 moved; elbow 1.29 -> 0.04) and the live site error
  stayed 0.460 (left) / 0.478 (right) instead of converging to 0.098.
- The same probe at the spawn moved 50-90% of a 0.01 command per step.
So the preview's kinematically valid q is NOT executable by the live WBC at
the standoff; every preview-derived predicted-surface number from that pose
is optimistic. This is the third measurement-contract gap (apply,
clearance, preview): measurement validity is now part of the substrate
contract.

## 16. WBC attenuation grid (live-execution contract)

Gain = mean live joint delta / commanded delta per step (5 steps/cell),
tuck posture, positions spawn / backed-off / standoff (full table in
reports/w1_raw/attenuation_grid.json):

  position  joint            s=0.01  s=0.02  s=0.08
  spawn     shoulder_roll    -0.00   0.12    0.31
            shoulder_pitch    0.14   0.24    0.21
            elbow            -0.01   0.02    0.08
            wrist_pitch      -0.01   0.01    0.16
  backoff   shoulder_roll     0.81   0.36    0.35
            shoulder_pitch    0.28   0.16    0.26
            elbow             0.08   0.15    0.24
            wrist_pitch       0.50   0.53    0.54
  standoff  shoulder_roll     0.07   0.18    0.18
            shoulder_pitch    0.11   0.10    0.17
            elbow            -0.03  -0.00   0.07
            wrist_pitch       0.01   0.02    0.23

WBC authority is position/posture/size dependent, not a global joint
property: the backed-off position tracks best; the standoff refuses the
elbow and wrist. Round-0's per-joint gain is therefore configuration
specific. Consequence: free-space relocation budgets must scale by 1/gain
(and a larger clamp moves more in absolute terms in free space); near the
object the gains collapse to ~0 and the joint-space path must stay slow.
W2j's unfold already ran at the backed-off position; its failure was the
0.005-step budget (~0.9-1.7 rad of 2.845 needed), not the ordering.

## 17. Final geometry iteration (W2j option (a)) and the side-by-side stop

Down pre-grasp dy 0.14, z 1.20, fwd 0.25 (pre_dx -0.19), x3 budget, 0.04
clamp:
- Unfold 401 steps, residual 0.381 (from 0.531), DLS preview 0.045/0.049.
- Object displaced 0.0085 m at the UNFOLD. First robot-object contact k294:
  `bottle_body` <-> `robot0_torso_link` (robot0_g19_col), 0.7 mm
  penetration at [1.629, 0.032, 0.994]; 64 contact steps.
- Cause: the base crept from [1.306, 0.027] to [1.484, 0.037] (+0.178 m
  FORWARD, undoing the backoff) over the 400 steps (~0.45 mm/step). The
  live hold (gain 0.05, cap 0.03, deadband 0.02) cannot arrest the creep
  under a long arm reconfiguration; the torso then reaches the drill.

SIDE BY SIDE: (1) side grasp - the grasp target branch collides at the
wrist (-0.011..-0.033 predicted), 0/25 pitch-roll orientations clear both
wrists; (2) top-down - the down pre-grasp is WBC-refused (interpolation
plateaus 0.25-0.38 m short) and the long unfold lets the base creep into a
torso contact. Shared limits: WBC arm authority is position/posture/size
dependent; the base creeps ~0.45 mm/step under arm motion; preview
solutions are not executable at the standoff. Batch 0; stopped for a
manager-level decision.

## 18. Arm actuator static-friction deadband (control-layer root cause)

The "WBC refusal" is an actuator friction deadband, not a policy choice: the
arm joints run robosuite JointPositionController (kp 300/shoulder-elbow,
800/wrist; kd 8/15; torque compensation on) with frictionloss 0.2 N·m (arm)
and 0.1 N·m (wrist). For small errors the PD share M(q) kp e is below the
friction while the actuator force is dominated by gravity compensation, so
the joint does not move. Measured (torque_probe at the backoff): actuator
forces 0.57-2.91 N·m vs ctrlrange +/-25/+/-10 (no saturation); an elbow
0.006 rad command moved 0.0008 rad.

Deadband compensation (command at least 0.012 rad in the target direction
when the residual is small) closes the plateau: the down pre-grasp site
error went 0.532 -> 0.1207 at k225 (previously plateaued at 0.25-0.55).
Two residual defects remain, both named: the compensation limit-cycles at
small residuals (~0.12 m) and the base creeps ~0.5 mm/step over a long
unfold, producing a torso contact at k318 (0.68 mm, 15.3 mm displacement).

## 19. Final pre-check: the top-down grasp target is joint-limit infeasible

Read-only solve at the current standoff (the back-off moved only 8 mm - the
WBC backward authority is weak): DLS preview err 0.121/0.092, seeded QP
ik_err 0.1052/0.0914, binding joint left_shoulder_yaw_joint at margin 0.0.
Margin map: left_shoulder_yaw 0.0, left_wrist_roll 0.0,
right_shoulder_yaw 0.0, right_wrist_roll 0.0, right_wrist_pitch 0.0;
shoulder_pitch 0.196/0.658, elbow 0.77/1.09, shoulder_roll 1.14/1.08,
wrist_yaw 1.50/1.56. A larger standoff moves the base farther and would
increase the reach, not fix the residual.

Both branches are therefore kinematic/actuator limited for this object:
side - the grasp target collides at the wrist (0/25 orientations clear);
top-down - the pre-grasp is reachable and the seeded descent clears the
wrists, but the grasp target is joint-limit infeasible (five joints at
0.0 margin, 10-12 cm residual). Stopped for a manager-level decision;
batch 0; no chain run.

## 20. Top-down grasp feasibility grid: 0/120 cells

Read-only seeded-QP solve of the grasp target over z 1.00/1.03/1.06/1.10 x
dy 0.06/0.08/0.10 x vertical hand roll 0/+15/-15/+30/-30, at two standoffs:
- A (aabb 0.163): 0/60 feasible, best worst-ik 0.0715 (z1.10 dy0.06 roll-30,
  margin 0.0 at left_elbow).
- B (backed off 0.277 m, aabb ~0.30): 0/60 feasible, best 0.1869.
Feasible = both residuals <= 0.02 and joint_margin_min >= 0.05. Every cell
has a joint at 0.0 margin; the best is 7.15 cm short. The z/dy/roll/standoff
dimensions do not rescue the joint-limit infeasibility.

Final measured constraint: side - target branch collides at the wrist
(0/25 orientations clear); top-down - pre-grasp reachable and seeded descent
clear, but the grasp target is joint-limit infeasible (0/120) and the torso
grazes at the close standoff. Stopped for a manager-level task/gate
decision; batch 0.

## 21. Design lesson: the grasp mode follows the object

A box is a two-hand object; a drill/tool is a one-hand object. Assuming a
fixed two-palm (12-D) grasp for 05 was the wrong premise, not a WBC or G1
geometry limit. The substrate contract now treats the grasp mode as a
property of the object (box -> two palms; drill/tool -> one hand, 7-D palm
pose), and the W2 gate reads bilateral pad contact on the object: for a
one-hand object it means the GRASPING HAND'S TWO PADS; for a two-hand
object it means both palms. Site <= 0.015 on the grasping pad(s), rise
>= 0.05, no guard trip, one named binding constraint. A fixed two-palm
assumption is the anti-pattern that blocked W2.

## 22. One-hand grasp: the feasible cell is a pre-grasp

One-hand grid (162 cells, both hands x dx x dy x z x front/down/inward):
exactly 1 feasible cell (ik <= 0.02, margin >= 0.05, wrist >= 0.02) -
right hand, down, dx -0.12, dy 0, z 0.92: ik 0.0050, margin 0.103
(right_wrist_pitch), wrist surface 0.024. The lowest-residual cells
(ik 0.0002-0.0005 at dx -0.04) all have wrist surface 0.0 (the wrist inside
the AABB).

Pad-surface check at the feasible cell: pad center [1.6153, 0.0302, 0.8323]
is 2.2 cm OUTSIDE the object's -x face (x lo 1.6373), at z 0.832 (lower
band), with pad normal [0.493, 0.869, -0.041] (lateral +y). The close along
the normal pinches in y at x 1.615 - in front of the object. So the cell is
a PRE-GRASP: the pads do not straddle the drill, and the solved hand is not
oriented to pinch the -x face. The close-in cells that do reach the object
have the wrist at 0.0.

The one-hand chain (w2_onehand.py) is implemented and ready; it was not run
because it would descend to a pre-grasp and close on air. The remaining
lever is a hand orientation (R_des) with the wrist not below the palm, or a
contact target the wrist can tolerate.

## 23. Top-down one-hand contact grid: no closable contact cell

192 cells (both hands x dx -0.06/-0.03/0/+0.03 x dy -0.03/0/+0.03 x z
1.02/1.05/1.08/1.12 x roll 0/90), pad-surface = the pad's inner face
(center - normal*half_thick) distance to the AABB. 3 cells pass
(pad_surf <= 0.005, wrist >= 0.02, margin >= 0.05): right hand, z 1.08,
dx +0.03, roll 90, pad_surf 0.0000, wrist 0.033, margin 0.161-0.183,
pad normal -X. But the -X normal makes the close axis X: the pads must
straddle the object's 0.19 m X depth, beyond the hand's ~0.10-0.12 m
opening. Every roll-0 (Y close, 0.10 m width) cell has pad_surf 0.0000 but
wrist 0.0 and margin 0.0 (the wrist inside the AABB). Practical closable
contact cells: 0. Same wrist-below/behind-the-pad geometry as the two-palm
branch, now one-handed.

## 24. Grasp-topology design record (queued; 05 measured, others expected)

| task | object | hands | grasp type | approach axis | expected contacts | basis |
|------|--------|-------|------------|---------------|-------------------|-------|
| 05 drill_lift | drill, AABB 0.19x0.10x0.226 | 1 | top-down, Y close preferred | -Z with -X faces | drill front/back faces at z 1.03-1.10 | measured (grids 20-23) |
| 01 box_lift_floor | box | 2 | side, opposing palms | horizontal | two opposite box faces | expected |
| 03 box_lift | box | 2 | side, opposing palms | horizontal | two opposite box faces | expected |
| 07 box_* | box | 2 | side, opposing palms | horizontal | two opposite box faces | expected |
| 02 press_button | button | contact task | press (no grasp) | normal to button | fingertip/pad press | expected |
| 04 push_* | object | contact task | push (no grasp) | horizontal | palm push face | expected |
| 06 tool | tool | 1 | one hand | top/side | tool body | expected |
| 08 tool | tool | 1 | one hand | top/side | tool body | expected |
| 09 tool | tool | 1 | one hand | top/side | tool body | expected |

Anti-pattern note: a fixed two-palm 12-D target for 05 blocked W2 and made
the failures look like tuning. The grasp mode is selected from the object.
Per-task AABBs for tasks other than 05 are to be read live when each task is
first loaded (read-only probe), then this table's rows are filled.

## 25. Object model breakthrough: the drill is not a box (geom-level grasp)

The drill's collision geoms (powerdrill_b01/model.xml, confirmed):
- collider_box_f302734d HANDLE: box pos [-0.035,0,0.054], size
  [0.02,0.014,0.08] (4 x 2.8 x 16 cm), quat 0.998/0.061 about y.
- collider_box_8e968ea9 MOTOR: capsule r 0.035, half-len 0.05 at
  [-0.02,0,0.159].
- collider_cylinder99aba387 CHUCK/BIT: capsule r 0.023, half-len 0.02 at
  [0.06,0,0.16].
- collider_box_a1f69b3d BATTERY: box size [0.06,0.038,0.02] at
  [-0.0143,0,0.02].

The body-tree AABB used for every earlier wrist/clearance check spans all of
these and is a large over-approximation. The handle is a thin box; a one-hand
grasp on the handle keeps the wrist outside the actual colliders. The
"wrist inside the AABB" verdicts (sections 22-23) are NOT collision
verdicts and must be re-measured at the geom level: clearance = wrist geoms
to each named collider (box/capsule), pad surface to the handle, per-cell
pad normal and contact face.

Topology record update: the drill's grasp part is the HANDLE
(collider_box_f302734d), one hand; grasp type top/side on the handle; the
AABB must not be used as the object model for grasp planning.

## 26. Geom-level handle grid: pads reach, wrist cannot clear

36 cells (both hands x handle frac 0.2/0.5/0.8 x roll 0/90 x palm z-offset
0.05/0.07/0.09), clearance measured to the actual colliders
(bottle_collider_box_f302734d handle, _8e968ea9 motor capsule,
_cylinder99aba387 bit, _a1f69b3d battery):
- ALL 36 cells: pad_surface = 0.0000 (the pads reach the handle).
- ALL 36 cells: wrist_min NEGATIVE, -0.008..-0.035 m - the wrist geom center
  is inside a collider. Best margins are the left roll-90 cells
  (0.19-0.24, ik 0.001) with wrist -0.008..-0.03; right-hand cells have
  margin 0.0 and wrist -0.009..-0.02.
The wrist radius (~0.047 m) sits 5-7 cm behind the pad; with the pad on the
4 x 2.8 cm handle the wrist center lands inside it. The wrist-to-pad
geometry, not the object model, is the binding constraint. Zero feasible
one-hand cells; the final measured constraint for 05 across both grasp modes
and both object representations. Stopped for the manager-level decision.

## 27. One-hand handle chain: upstream blockers

Approach-axis grid (48 cells: front/side_p/side_m/diag x frac x roll):
3 feasible cells - front approach, handle frac 0.7, roll 90/0, both hands:
pad_surf 0.0000, wrist 0.027-0.028 to the battery, margins 0.64-0.78,
ik ~1 mm. Best: left front frac 0.7 roll 90 (margin 0.782).

Chain run (left cell, front pre-grasp dx -0.205 z 0.954): unfold residual
0.355 at the 251-step cap (stop_err 0.12 not reached), approach standoff
aabb 0.167 but lateral_error 0.07 vs the 0.04 tolerance with base yaw
14 deg after 60 bursts; the chain stops at the approach. Object
displacement 0.0. The grasp cell is feasible read-only; the blockers are
the front pre-grasp reachability and the base lateral/yaw after the unfold.

## 28. Two-stage one-hand chain: yaw fixed, two upstream stalls

Stage 1 (down pre-grasp, 0.119 proven): clean, no displacement. Stage 2
(joint-space move to the front pre-grasp dx -0.205 z 0.954): stalls at
0.401 after 81 steps (pad_error) - the front posture's own deadband/gain
is untuned. Approach: lateral 0.0108 m and yaw -0.51 deg (the tightened
yaw hold + strong hold fixed the 0.07 / 14 deg), but the forward walk
stalls: the base stays at [1.343, 0.019], aabb 0.298 after 60 bursts
(~0.04 m in 24 s commanded). The forward channel does not move the base
with the arms in the front pre-grasp posture. Chain stops at the approach;
0.0 displacement in every phase; W2 not passed.

Next levers: tune the front-posture joint-space move (its own deadband/
gain curve), and re-approach before the front pre-grasp (down pre-grasp ->
approach -> front pre-grasp near the handle -> descent) so the walk happens
with the tucked arms that do walk.

## 29. QUEUED for the next session: `object_info` L1 tool (manager requirement)

Config entry `object_info` (capability `object_part_inventory`) in
warmup.substrate. Read-only; no state change. Contract:
- Returns: the task object name; a short semantic summary with evidence;
  every body in the object tree with mass and local/world pose; every
  collision and visual geom (name, type, size, local/world pose, mesh asset
  name, parent body); the world AABB explicitly flagged as an
  over-approximation, NOT the object model; a shape-based part
  classification (handle / body / bit / battery / base) with the evidence
  geom and measured dimensions; per part the cross-section vs the hand
  opening (~0.10-0.12 m) and vs the wrist radius (0.047 m) + wrist-pad
  offset (5-7 cm); the candidate grasp parts + approach axes
  (front/side/top); the source model XML path.
- Classification is structural (shape + the model's own body/mesh names),
  not a text classifier.
- Validator: pass on the drill (handle box 0.04x0.028x0.16, motor capsule
  r 0.035, bit capsule r 0.023, battery box 0.06x0.038x0.02) and on a box
  task (03) as the contrast; assert no state change; record the output here.
- The grasp-topology pass must consume it: hand count and grasp part come
  from the tool's inventory, not from the body AABB.

Implementation sketch: aggregate `bodies(match=<object root>)` +
`object_aabb` + the model XML path; classify by geom type/size and the
body/mesh names; emit the per-part cross-section vs the hand/wrist
clearances. The drill model path used here:
humanoidmimicgen/locomanipulation/models/assets/objects/omniverse/locomanip/
powerdrill_b01/model.xml.

Also queued: the two chain levers (front-posture joint-space tuning; the
arm posture that lets the base walk forward, or re-approach before the
front pre-grasp). W2 gate unchanged; batch 0.

## 30. object_info L1 tool: implemented and validated (item 1)

Server tool `object_info` (hmg_tools.Tools.object_info, dispatched in
hmg_server.handle) returns the object name; bodies (mass, local/world
pose); every geom (name, type, size, local/world pose, mesh asset, parent
body, collision flag); the world AABB flagged as an OVER-APPROXIMATION;
a structural part classification; per-part cross-section vs the hand
opening (0.10-0.12) and the wrist radius 0.047 + pad offset (0.05-0.07);
candidate grasp parts and axes; the source model XML path.

Validator (work/ll_v3/object_info_test.py), state unchanged on both:
- 05 drill (bottle_main, 3 bodies, 6 geoms): handle
  bottle_collider_box_f302734d dims [0.028,0.04,0.16] cross [0.028,0.04];
  body capsule [0.07,0.1,0.07]; bit capsule [0.046,0.04,0.046]; battery
  box [0.04,0.076,0.12]. Matches the model (handle 0.04x0.028x0.16, motor
  r0.035, bit r0.023, battery 0.06x0.038x0.02). Grasp candidates: handle,
  bit (front/side/top).
- 03 box (obj_main, 1 body, 10 geoms): four box geoms 0.086x0.115x0.168
  (the two-hand faces) plus five thin panels. Grasp candidates: the four
  box faces. Contrast as intended.
Classification is structural (geom type/size + ratio + cross-section), not
text-based. The 03 panels are labelled 'battery' (a flat-plate class); a
'panel' label is a cosmetic refinement.
