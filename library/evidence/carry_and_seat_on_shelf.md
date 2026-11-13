# carry_and_seat_on_shelf -- proof history

Pinned artefact: source `3b41be06d110aaeec0c75d05de752b029855b85014bcbf76c05f7f1518ed25a9`
(07_box_table_to_shelf/actor_5/pp31_a5.py, 54497 B, 84 signature params) WITH
the 34 kwargs recorded verbatim in the description and in
`work/tracks/07_box_table_to_shelf/actor_5/req_expX_v1.json` /
`req_expW_v1.json`. A default-kwargs run is a different experiment.

## Task outcome
07_box_table_to_shelf engine TRUE on seeds 0-7 in one coordinator batch
(`coord_pp31_full`, t 57.34 / 58.46 / 56.02 / 57.18 / 56.16 / 57.46 / 59.38 /
58.82; episodes 222937-223226). Before this round the track was 0-4 TRUE,
5-7 FALSE with front margins -0.1095 / -0.7411 / -0.1606 m on 5/6/7.

## Gate A (unseen seeds) -- ACCEPT NARROWED
Verifier run on seeds 8-13 at the pinned artefact (`vv_gateA_8_13.json`, with
the builder asserting sha256, byte length and kwarg count before writing).
Coordinator engine bits, matched by t:

| seed | t | proxy negatives | engine |
|---|---|---|---|
| 8 | 58.98 | none | TRUE |
| 9 | 59.30 | back_m -0.0202, x_m -0.0377, yaw 149.12, gap +0.0072 | **FALSE** |
| 10 | 58.30 | none | TRUE |
| 11 | 60.22 | front_m -0.1026 | TRUE |
| 12 | 57.38 | front_m -0.1049 | TRUE |
| 13 | 56.94 | none | TRUE |

5/6 unseen TRUE. Seed 9 is the documented falsifier (rotated, unseated). Seeds
11 and 12 are negative-margin engine TRUEs, the measured false-negative class:
EXP V s7 reads fm -0.1043 and is engine TRUE, EXP Y s1 reads xm -0.0681 and is
engine TRUE. The seed range is unbounded: resets succeed for 18, 19, 20, 50,
100, 1000 and 1000000 with a constant object pose, base z 0.793 and only base y
randomised over ~[-0.055,+0.057] m.

## Gate B (limits, torque, ownership, determinism) -- ACCEPT NARROWED
- Upper body, 17 non-hand DoF: min margins +0.0483 to +0.1334 rad, zero steps
  below -1e-3. Hand DoF: -6e-6 to -4.1e-5 rad, solely joints parked on a 0.0
  range bound.
- Lower body (WBC-owned, 12 DoF): ankle pitch reaches -0.0131 to -0.0201 rad on
  every seed, in 8-17 contiguous segments per episode, first at t~1.5 s (the
  crouch to base z 0.549). Present on engine-TRUE seeds 0 (-0.0146) and 7
  (-0.0128), so it is not introduced by this candidate and does not decide the
  predicate. It is a real model-range breach: do not call the task limit-clean.
- Torque/ctrl, raw per-step counts: lower max fraction 0.7138-0.8880, upper
  non-hand 0.4260-0.4934, zero DoF-steps >= 1.0. Hand only: thumb_1 exceeds on
  s9 (2 steps, 1.1347) and s12 (31 steps, 1.3369); the hand ctrlrange equals
  the joint angle range, so this is a unit artefact, not torque evidence.
- Ownership: source has no lower-body joint name and no direct env/sim/qpos/
  qvel/tau/ctrl access; every interaction is via api.* (step_goal, set_height,
  reach_box, close_grip and reads). Trace: act always (T,31), command overshoot
  <= 2.4e-5 rad; nav (T,3), channel 2 reaching -1.0 unclipped; height within
  the [0.44,0.90] clip. `_send`'s 31-D shape check plus contract validation
  reject any lower-body command.
- Determinism: seed 8 run twice in one request and once fresh -- identical t
  58.98, n_events 292, all maxima and the verdict; all 15 trace arrays
  max|diff| 0.0. Seeds 0 and 7 re-ran at the pinned artefact and reproduced the
  coordinator's t exactly (57.34, 58.82).

## Gate C / merge-or-justify -- DISTINCT SKILL
The candidate claims no cross-task scope, so no cross-task run was required.
Checked against the five nearest catalogued skills: servo_base_to_target_pose
stops on base xy/yaw only (no board-relative seat, no two-palm carry, its own
docs record a 0.052 m height drop); walk_base_to_coordinate is one-axis,
open-loop yaw, <= 8 bursts, no upper-body hold; carry_gripped_object_to_pose
targets a base pose with a gripper grasp and no release; move_held is arm-only,
single-axis, drill/holder tags, no base or release; release_grip_and_settle_in
_place exposes only 3 parameters and no palm approach. None can be
parameterised into two-palm carry + base nav/yaw + crouch + board-relative
seating + release.

## Mechanism (measured by the actor, kept for the contract)
Two independent selections, both inside existing code paths:
- strafe stream (n_str=8, str_yaw_tol=5.0) is the ONLY lever that moves seed 7
  (strafes-only s7 front margin +0.0788 vs pin -0.1606; gate-only -0.1043);
- one-shot base-y latch (pre_lat_y=-0.02, pre_lat_v=-0.05) is the ONLY lever
  that moves seeds 5 and 6 (gate-only +0.0623 / +0.0715 vs pin -0.1095 /
  -0.7411; strafes-only -0.2441 / -0.7375).
The latch's admissible interval is (-0.0320, -0.0100) = 22.0 mm, bracketed on
both sides by measured failure; -0.02 sits 12.0 / 10.0 mm inside the edges.

## Non-evidence and caveats
- The candidate's `knobs` response literal omits nudge_n, yaw_tol and lows_gap.
  The passed kwargs are the effective ones (consumed at pp31_a5.py:759/788/837).
- `lows_gap=-9.0` (default 0.008) disables the early-descent termination and is
  load-bearing.
- Verification pins (source hash + call kwargs). A `{}` run of the same bytes
  selects different branches (this cost a gate-A retraction on 05).

Evidence files: `work/tracks/07_box_table_to_shelf/verifier/` (`vv_gateA_8_13.json`,
`vv_gateA_8_13_resp.json`, `vv_det_same.json`, `vv_det_fresh.json`,
`vv_known_0_7.json`, `vv_seedprobe*.json`), `actor_5/req_expX_v1.json`,
`req_expW_v1.json`, `resp_expV_v1.json`, `resp_expY_v1.json`.


## Re-pin: pp33 (latch on released grip) -- supersedes pp31

Artefact: `0f3d08084a3fc9790ead7a5f35d92e7abe3df6ee25049296e1fbdc268b829b9d`
(pp33_a5.py, 55645 B), 35 kwargs (the pp31 set + `rel_grasp_min=0.5`). Three
hunks vs pp31: the signature, the knobs record, and one latch taken from the
already-recorded released snap (`shep = out['released']['hb']['f'] >=
rel_grasp_min`); when the open ends without a grip the seat row's lateral
component is zeroed so the pads drive straight down instead of shearing a free
box off-centre. No new API read, no new command; AST call-site counts are
identical between the two sources.

### Task outcome
Engine TRUE on seeds 0-19. Coordinator batch 0-13 (t 57.34/58.46/56.02/57.18/
56.16/57.46/59.38/58.82/58.98/59.30/58.30/60.22/57.38/56.94); verifier gate A
on unseen 14-19 (t 57.50/46.98/58.66/57.86/58.94/59.16). Seed 9, FALSE at the
pp31 pin, is TRUE here.

### Gate A (unseen 14-19) -- ACCEPT
Proxies: s16 is the tightest (front -0.1123 m, release gate pass 0), s17 x
margin -0.0046, s15 a 2349-step path; s19 is the only unseen seed where the
latch fires (released hb.f 0.0 N). All six ran to completion and are engine
TRUE; all seat with board_gap 0.0002 and up 1.0.

### Gate B -- ACCEPT NARROWED
Determinism: seed 14 three times (same batch and fresh request) -> all 15
trace arrays bit-identical, identical maxima (act_jump 0.32824, base_drop
0.24614, lim_margin -0.02191, penetration 0.00635, slip 0.0154, tilt 0.27973,
torque 74.25053, torque_frac 1.01616, n_events 276). Limits: lower body -0.0129
to -0.0285 rad (deepest seed 5, right_ankle_pitch, t 37.50), 1813-2202
lower-dof steps below -1e-3; upper non-hand +0.048 to +0.134, zero violations;
hand negatives are fingers parked on the 0.0 bound. Torque: lower peak 0.634-
0.974, upper non-hand 0.399-0.529, zero dof-steps >= 0.995; hand 0.905-1.34 is
the ctrlrange unit quirk. Ownership: 31-D upper contract order only; nav and
height unchanged where the latch acts; lower DoFs never commanded.

### Identity claim verified
Field-by-field: seeds 0-8 and 10-13 differ from the pp31 pin in exactly one
leaf, `/knobs/rel_grasp_min`; seed 9 is the sole exception (160 leaves, all
trace-identifiable as the latch: `/seat_rows/0/cmd_dx` 0.02 -> 0.0,
board_gap 0.0072 -> 0.0002, up 0.9988 -> 1.0). Trace level: 18 of 20 seeds
bit-identical across all 15 arrays. Seed 9 is identical through row 2854
(~t 57.14); seed 19 through row 2873, then upper DoFs 0-16 only change.

### New falsifiers
1. Release-force samples are {0.0, 0.026, 0.158, 9.09, 14.542, 16.475, 26.006}
   N; the 0.158-9.09 N band is unmeasured, so a seed releasing at 1-8 N flips
   the latch with untested outcome.
2. The latch is a hard step with no hysteresis and no per-seed margin report.
3. The source comment's lower bound (0.026 N) is factually wrong; seed 10
   releases at 0.158 N.
4. Proxy/engine divergences carried forward: s16 front -0.1123 with release
   gate pass 0 and s17 x -0.0046 are engine TRUE; s9's back margin -0.0182 is
   proxy-negative and engine TRUE.

Request files: `verifier/vv_gateA_14_19.json`, `vv_det33_fresh.json`,
`vv_pp33_0_13.json`, `vv_pp31_ctrl.json`, `vv_claim3.py` / `vv_claim3_0_13.out`.

## Scope extension: seeds 20-40 (coordinator, 2026-11-11)

Same artefact (`0f3d0808…b9d`), same 35 kwargs, one batch of 21 unseen seeds:
**16/21 engine TRUE**. Failures: s21 (t 55.70), s22 (55.70), s25 (55.66),
s32 (56.74), s38 (57.96). The five failing times sit in the 55.7-58.0 s band
the passing seeds also occupy, so t gives no separation. Request
`work/coordinator/req_coord07_20_40.json` (source 55645 B, 35 kwargs,
sha256 asserted on disk before sending), responses in
the session tmp directory of the 2026-11-11 run.

Consequence: the verified scope stays seeds 0-19 (20/20, unchanged), and the
task is bounded there. Over the whole tested range 0-40 the composition is
36/41. A seed drawn above 19 is a coin flip, not a covered case; do not quote
"07 solved" without the range.
