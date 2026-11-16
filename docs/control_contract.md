# ll_v3 control contract (round 0, measured)

Status: **round 0 calibration complete; warm-up and batches NOT started.**
Source of truth for the numbers: `experiments/ll_v3/calibration.json`.
Raw responses: `work/ll_v3/reports/round0_raw/`.

Measured on the live G1 model: 05_drill_lift seed 0 for the joint / ramp /
reach / IK measurements; 05_drill_lift seeds 0-3 for the grasp proxy; all nine
tasks seed 0 for the standoff envelope. The waist stayed HOMIE-owned by default;
the only `ik` use was an authorized handover with a measured-gain check before
and after, and the default owner was restored. No physics, reward, predicate or
model-limit change.

## 1. Authority + measured gain (stored separately)

Authority is structural (`docs/joint_authority.json`); measured gain is dynamic
(`track_gain`, delta 0.2 rad, dur 1.5 s, hold 1.0 s, direction-aware). A high
gain does **not** confer authority.

| group | dof | default final writer | measured gain (homie/default) | commandable through the 31-D upper channel? |
|---|---:|---|---|---|
| legs | 12 | `homie_v2` | null (WBC only) | no — never command hip/knee/ankle |
| waist | 3 | `homie_v2` | yaw 1.142, roll 0.415, pitch 1.189 | **conditional** — only after `waist_owner=ik|blend` |
| arms | 14 | `upper_target` | 0.962–0.997 | yes |
| hands | 14 | `upper_target` | 0.856–0.863 (one-sided) | yes |

30 of the 31 upper joints measured |gain| >= 0.8. The exception is
`waist_roll_joint` under `homie` (0.415); it is **not commandable** because its
authority is `homie`, not because of the gain. Every arm and hand joint is
>= 0.8 and `final_writer=upper_target`.

### Waist handover (authorized, verified, restored)

| joint | homie before | ik after | homie again (restore) |
|---|---:|---:|---:|
| `waist_yaw_joint` | 1.142 | 0.999 | 1.142 |
| `waist_roll_joint` | 0.415 | 1.000 | 0.415 |
| `waist_pitch_joint` | 1.189 | 1.011 | 1.189 |

The handover is a HANDOFF event with the owner recorded; the restore is exact
to the measured precision, and the balance (base tilt) stayed within 8 deg.
`waist_pose_servo` is therefore an L1 with the gate
`waist_owner in {ik, blend}`; under `homie` the waist is `observed_exogenous`.

Per-joint values are in `calibration.json.authority_gain.measured` (the full 43
final writers are in `robot_authority`).

## 2. Ramp-lag contract

Measured as a step command with per-control-step sampling: time to 95 % of the
commanded delta, on `right_elbow_joint` and `left_shoulder_pitch_joint`.

| delta (rad) | measured t95 (s) | recommended `ramp_s(delta)` |
|---:|---|---:|
| 0.1 | 0.14 / 0.14 | 0.15 |
| 0.3 | 0.12 / 0.10 | 0.15 |
| 0.5 | 0.12 / 0.12 | 0.15 |
| 1.0 | 0.12 / 0.32 | 0.35 |

The step response is 0.10–0.32 s, much shorter than the fixed 1.5 s `_ramp`
used by `track_gain`. Use `ramp_s(delta)` above instead of a constant ramp.
The negative direction (-0.5 rad elbow) measured 0.10 s; the curve is not
materially asymmetric. Raw trajectories are in
`round0_raw/ramp_all.json`.

## 3. Reach envelope per arm

`ik_preview` (read-only, DLS) at the 05 minimum reachable pose: base
`[1.488, 0.0145, 0.7537]`, yaw 0.77 deg, tilt 1.2 deg. Grid: x offsets
{-0.04, 0.02}, gaps {0.155, 0.175}, z offsets {0.46, 0.52, 0.58, 0.64, 0.70},
orientation `down`, 150 iterations, 20 rows per side.

| side | rows | reachable <= 15 mm | mean err | min err | max err | best cell |
|---|---:|---:|---:|---:|---:|---|
| left | 20 | 0 | 0.1103 m | 0.017 m | 0.208 m | xo -0.04, gap 0.155, z 0.46, err 0.017 |
| right | 20 | 0 | 0.0909 m | 0.019 m | 0.190 m | xo 0.02, gap 0.155, z 0.46, err 0.019 |

Left−right mean residual difference **+0.0194 m** (max 0.044 m): the left arm
is slightly worse in this grid. The preview is the **DLS** solver; the mink QP
executes far better (0.008–0.015 m, Section 5). Use `ik_preview` only as a
reachability screen; the adopted execution solver is the mink QP.

## 4. Standoff envelope per task

Definition: standoff = distance from the base origin to the task object
body-tree AABB nearest point; the root distance is reported too. The drill
target uses the harness default `gap=0.155, z_off=0.45`; the box/drill targets
use the live AABB-centre height.

**05_drill_lift (measured).** Minimum reachable root standoff **0.233 m**
(AABB **0.136 m**) at base `[1.5008, 0.0124, 0.753]`, with **pad error
0.0059 m**, joint margin **0.1719 rad**, base tilt **3.66 deg**. This re-measures
the prior ~0.25 m result (root distance) and confirms it; it is not copied.

**Other tasks (blocked).** The hand-rolled `move_base` approach is not a
calibrated standoff servo; it tilts 20–125 deg on 01/04/09 and cannot hold the
15 mm gate on the others. Per-task numbers are therefore recorded as measured
failures with their binding constraint, not as adopted standoffs:

| task | status | binding constraint | measured |
|---|---|---|---|
| 01_box_lift_floor | blocked | `base_tilt_guard` | crouch 0.46 m + approach tilted 19.8–29.4 deg; pad error 0.98–1.54 |
| 02_push_button | blocked | `target_invalid` | best pad error 0.156 at AABB 0.192; target must be the button, not the panel centre |
| 03_box_lift | blocked | `target_invalid` | best pad error 0.466 at AABB 0.362; generic target height is above the box |
| 04_push_shelf_forward | blocked | `base_tilt_guard` | shelf approach tilted 20.7–33.0 deg and pushed the 16 kg shelf; push standoff is a contact distance |
| 06_drill_pnp | blocked | `target_invalid` | best pad error 0.091 at AABB 0.157; needs the task-specific grasp point |
| 07_box_table_to_shelf | blocked | `target_invalid` | best pad error 0.440 at AABB 0.306; needs the box grasp point |
| 08_pick_drill_from_holder | blocked | `target_invalid` | best pad error 0.072 at AABB 0.195; the holder blocks the generic target |
| 09_obstacle_aware_pick_drill | blocked | `base_tilt_guard` | obstacle approach tilted 90.9–125.1 deg; no standoff number |

**Design consequence (G0):** the per-task standoff envelope requires the W1
`base_navigate_to_standoff` servo (closed-loop standoff, fingertip/tilt
guards) and task-specific targets. It cannot be completed with the current
`move_base` primitive. Run W1 before the per-task envelope, or defer the
envelope to W1; the 05 number above is the only adopted standoff.

## 5. IK dimension A/B and adoption

Same targets (05 drill, two palm poses at `[1.731, ±(0.155), 1.25]`).

### Read-only model solve (`ik_compare`, mink QP on scratch data)

| condition | dim | worst pad error | joint margin | solve time | base search | notes |
|---|---:|---:|---:|---:|---|---|
| homie | 12 | 0.00013 m | 0.1629 rad | 0.0151 s | — | adopted |
| homie | 14 | 0.00013 m | 0.1629 rad | 0.2447 s | (0, 0) | dominated: same residual, 16× solve |
| ik | 12 | 0.00012 m | 0.0742 rad | 0.0134 s | — | handover control |
| ik | 17 | 0.00001 m | 0.1338 rad | 0.0090 s | — | `waist_authoritative=true`, waist move 0.0229 rad |

### Executed (`qp_reach`, mink QP on the live robot)

| condition | dim | worst palm error | joint margin | waist owner | waist solved/commandable |
|---|---:|---:|---:|---|---|
| homie | 12 | **0.0146 m** | 0.1559 rad | homie | false/false |
| ik (authorized) | 17 | **0.0109 m** | 0.1394 rad | ik | true/true |
| homie (restored) | 12 | 0.0128 m | 0.0938 rad | homie | false/false |

**Adopted: 12-D / `waist_owner=homie`.** It is the smallest candidate that
meets the 15 mm pad and 50 mrad margin gates (executed worst 0.0146 m, margin
0.1559). 14-D is dominated (same read-only residual, 0.2447 s vs 0.0151 s, base
search (0,0)); 17-D under `ik` is slightly better executed (0.0109 m) but
requires the authorized handover and is not the smallest. 17-D is kept as a
measured alternative, never the default. The 14-D candidate is retired with
this measurement (G3). The handover was restored to `homie` and the final tilt
was 3.3 deg.

## 6. Grasp proxy (`grasp_state_tracking` vs the benchmark outcome)

Proxy definition: **lift-phase bilateral hand-object contact** (left > 0 and
right > 0) **and object rise > 0.02 m**. Conditions: `good` (drill target, lift
0.15 m), `low` (drill target, lift 0.05 m), `miss` (target 0.21 m above the
drill, lift 0.15 m), four seeds each.

| proxy \ benchmark task success | true | false |
|---|---:|---:|
| proxy true | 4 | 3 |
| proxy false | 0 | 5 |

The benchmark outcome is the full 05_drill_lift task success; the exact
`IsGrasped` sub-condition is **not exposed** (`success_detail` only has
`task`), so this is a validation against the task outcome, not the sub-predicate.

**Failure boundary.**

- The 3 false positives are the **low-lift** condition (rise 0.045–0.081 m,
  below the task's 0.1 m threshold). The proxy is a grasp indicator, not the
  task predicate; `lift_object`/task programs must add the rise and support
  terms.
- **Close-phase bilateral contact is not sufficient**: good seeds 0 and 3 had
  0/0 close-phase contacts but 3/2 and 2/2 lift-phase contacts, rose
  0.168/0.149 m, and succeeded. A close-phase-only proxy would produce false
  negatives.
- One-sided/swept-object episodes (good seed 1: rise 0.185 m, only right-hand
  contacts; miss seed 2: rise 0.155 m, zero contacts) are correctly not called
  grasps. The bilateral lift-phase requirement is what rejects them.
- The proxy should be reported as raw fields (bilateral contact, aperture,
  relative pose/velocity, rise) and never used to assert the task predicate
  (G1).

## 7. Control contract fields and rules

Every L1 declares, separately:

- `commands`: joints it writes. A joint is listed only if the registry says
  `final_writer=upper_target` and the measured gain is >= 0.8. The legs are
  never listed; the waist is listed only under the `ik|blend` gate.
- `observed_exogenous`: joints read every control step and fed into
  FK/Jacobian (the waist while `homie`).
- `assumes_fixed`: joints/poses assumed constant, with a measured drift bound
  (base pose held by the base-lock task; hand joints held by the pre-shape).
- `min_gain`: 0.8 per commanded joint.
- `binding_constraint`: the single measured reason a candidate fails.

Measured authority for the L1 command verbs:

| L1 | commands | observed exogenous | assumes fixed |
|---|---|---|---|
| `ee_pose_ik` | 14 arm joints | 3 waist joints (`homie`) | base pose (base-lock); 14 hand joints (pre-shape) |
| `ee_position_servo` / `joint_space_transport` / `constrained_servo` / `contact_push` | 14 arm joints | 3 waist joints (`homie`) | 14 hand joints (pre-shape / closed vector) |
| `hand_pre_shape` / `grasp_close_contact` / `grasp_close_force_limited` / `grasp_hold` / `grasp_open_release` / `loaded_release` | 14 hand joints | waist + arms | 14 arm joints (held) |
| `waist_pose_servo` | 3 waist joints **only under `ik|blend`** | arms + hands + base | — |

A skill that hands the waist over also declares `waist_owner`,
`waist_gain_before`, `waist_gain_after`, `handoff_events`, `lower_owner`, and
restores `homie` on exit. Round 0 measured the before/after/restore above.

## 8. Gaps that change the design

| id | gap | measured evidence | design change |
|---|---|---|---|
| G0 | per-task standoff envelope blocked | hand-rolled approach tilts 20–125 deg; only 05 measured | run W1 `base_navigate_to_standoff` before the per-task envelope, or defer it |
| G1 | exact `IsGrasped` not exposed | proxy vs task success 4 TP / 3 FP / 0 FN / 5 TN; FP = low lift | proxy is a grasp indicator; lift L2 adds rise/support; never read the predicate |
| G2 | task-specific targets | 02 button, 03/07 box centre, 08 holder, 04 shelf side | target belongs to the task strategy / press-push-place skill |
| G3 | 14-D dominated; executed path needs base placement | same residual, 0.2447 s vs 0.0151 s | adopt 12-D; retire 14-D |
| G4 | 17-D alternative | executed 0.0109 m vs 0.0146 m; needs handover | keep 12-D as smallest; 17-D is a measured alternative |
| G5 | DLS preview overstates QP residual | DLS 0.017–0.208 m vs QP 0.008–0.015 m | preview is a reachability screen; execute with the QP |
| G6 | ramp curve shorter than the fixed ramp | step t95 0.10–0.32 s vs 1.5 s | use `ramp_s(delta)` 0.15/0.15/0.15/0.35 s |
| G7 | 01 floor crouch unstable | crouch 0.46 m + approach tilt 19.8–29.4 deg | 01 needs `base_height_setpoint` + a stable crouch servo |
| G8 | 04 shelf push is a contact problem | 16 kg shelf pushed; tilt 20.7–33.0 deg | push standoff belongs to `push_object_to_target` |
| G9 | 09 obstacle approach unstable | tilt 90.9–125.1 deg | 09 needs collision-free reach / obstacle-aware standoff |
| G10 | lift-phase joint margin at 0.15 m | good seed 0 lift margin 0.0406 rad (< 0.05) | cap the transport lift or add a posture adjustment; the task's 0.1 m lift is safer |

## 9. Round-0 state

- Batch counter **0**; round counter **0**; no warm-up or batch started.
- `experiments/ll_v3/calibration.json` is written.
- Raw per-measurement responses: `work/ll_v3/reports/round0_raw/`.
- Drivers: `work/ll_v3/round0_gain.py`, `round0_ramp.py`,
  `round0_standoff.py`, `round0_standoff2.py`, `round0_reach_ik.py`,
  `round0_grasp_proxy.py`, `build_calibration.py`.
- The live servers were stopped after the measurements.
