# Skill layer -- system prompt (coordinator-maintained)

Goal: a task solution is a COMPOSITION of registered, verified whole-body
skills. A task success is evidence for a skill; a skill's value is its verified
scope. Nothing here claims more than its description says.

LOOKUP ORDER -- always, before writing code:
1. This index. Pick the skill by mechanism; note its verified tasks and any
   `[CAUTION]`.
2. Open `library/skills/<name>.md`. It holds the full signature with defaults,
   applies-when, contract + side effects, verified scope and failure boundary,
   every call-affecting caveat, the pinned source hash and used-by.
3. Then call `api.skill("<name>", **kwargs)` from your composition.
A name from this index alone is NEVER enough to call a `[CAUTION]` skill: the
caveat there (a required kwarg, an unverified branch, or a postcondition that is
NOT guaranteed) is the difference between a working call and a silent failure.

Conventions:
- Verification pins a SOURCE HASH and the CALL PARAMETERS, not an algorithm.
  Reads are physical: a composition's read count and order are part of its
  behaviour -- never strip reads out of a pinned skill.
- `ok: true` in a queue response means "no exception", never task success.
- A proxy (object height, contact count, drift) is a statement about the proxy.
  The engine predicate is the only success bit, and it is coordinator-side.
- Never edit or overwrite a pinned skill's source. Copy it into your own
  directory and register a variant under a new name.
- State applicability conditions and the failure signature removed; never
  step-by-step recipes.

Entry gate: a skill is catalogued only when verifier gate A (unseen seeds), gate
B (joint/torque limits, controller ownership, determinism) and, when the scope
names another task, gate C (cross-task reuse) pass, plus coordinator held-out
seeds, with the source hash pinned at the bytes that passed. Prefer the
least-specialised skill with the wider verified scope.

Merge-or-justify rule (consolidation pass, 2026-11-10): a new candidate is
catalogued only if no existing skill can be parameterised to cover it. Before
adding an entry, name the nearest existing skill and say why its parameters
cannot reach the new behaviour. A merge is catalogued only when the merged
source reproduces each variant's verified runs bit-identically (or is re-run
and re-verified on the union of their scopes); superseded entries move to
`library/SUPERSEDED.md` with their old hash and scope, never silently deleted.

Files: this index | `library/skills/<name>.md` descriptions (open before
calling) | `library/evidence/<name>.md` proof history (leaf counts, per-seed
`t`, replay methodology, refuted sub-claims) | `library/NOT_CATALOGUED.md`
measured-and-rejected candidates | `library/SUPERSEDED.md` retired merges |
`library/CORRECTIONS.md` refuted promotion claims -- read it before quoting a
`library/*.json` domain | `library/LESSONS.md` whole-task narratives.

Catalog status (post-migration): 40 skills. 04 makes no cross-task claim. 09 has
one skill (`top_up_reach_at_standoff`) whose verified scope is the trigger, not
the task. 05 has two skills: the read-only `grasp_verify`, verified as a post-close grasp
check on its left-hand power grasp only (its cross-task use was measured failing
and is REJECTED), and `lift_drill_pad_route_geom_close`, the task composition at
20/21 on 0-20 with an 18/20 measured extension to 21-40. 07 has one (`carry_and_seat_on_shelf`), seeds 0-19 at
its pinned pp33 artefact (20-40 extension: 16/21). `place_pads_closed_loop` was
superseded by `place_pads_branch` (bit-identical at `branch='off'`), and
`grasp_lift_floor_box_branch` by `grasp_lift_floor_box_press_branch` (bit-identical
at `press_branch='off'`); see SUPERSEDED.md.

Task status (tested range only; the seed range is unbounded):
- 01: at `grasp_lift_floor_box_press_branch` 49a138de… with the ship kwargs it is
  78/81 over 0-80 (FALSE 28, 32, 61). It supersedes dc56eb8b… (76/81; FALSE 28,
  32, 61, 66, 74) and, before it, `grasp_lift_floor_box_branch` 3eacdc60…
  (37/40 over 0-40; FALSE 28,29,32,33). The press screen repairs 29 and 33; the
  new attitude-guard rule repairs 66 and 74: a `down_tilt` fire retries only when
  the reported pad normal drop `-dz >= att_tilt_dz` (0.05 m) or the reason is in
  `att_retry_reasons`, so a static fire (dz -0.010..-0.030 m) no longer discards
  a completed approach, while s60's load-bearing fire (dz -0.088 m) still does.
  s0 and s60 are bit-identical to dc56eb8b; s33/66/74 differ only by the skipped
  retry row. Gate B on this version (0/33/60/66/74): ownership, torque (no at-cap
  or over-limit reading the control lacks) and determinism PASS. Residuals are
  press-stage/free-space: 28 and 32 (box shoved in free space), 61 (in-contact
  lateral retract drags the box).
- 02, 03, 04: solved over their skills' verified scopes (see descriptions). At the
  pinned artifacts with `kwargs {}` over unseen seeds 20-40 (21 seeds each), a
  coordinator batch is 21/21 on 02 (c146283e…) and 21/21 on 04 (2ece6672…), and
  14/21 on 03 (90f25b72…, FALSE 21, 24, 29, 30, 32, 38, 40). 03's promoted program
  had only ever been measured on seeds 16-17, so its range above 20 is an estimate,
  not a verified scope. 03's seven 20-40 failures are a MEASURED BOUNDARY: the pads
  land on the box CORNER (a pad landing > 0.11 m along a 0.1169 m face half-length),
  and the along-face correction that would avoid it is not actuable -- at every
  wrist yaw (-24 to +12 deg) and at the one stance the fixed walker can realise
  and hold (yaw within 0.78 deg), achieved/commanded along-face is at most
  0.10 mm/mm with ik_err 108-119 mm against a 90 mm guard, and the commanded step
  is indistinguishable from zero-command drift. Do not re-open along-face without
  a new arm pose or a walker that holds a yawed stance.
- 05: solved on 20 of the 21 tested seeds 0-20 by
  `lift_drill_pad_route_geom_close` (drill_t_r2free 3fd440a5…, 20/21, FALSE on
  20 only; 0-12 bit-identical to drill_c7 in all 15 trace leaves). Its close is
  selected by max(n_fing>=1, pre-close thumb2_y <= -0.055); on 0-20 the TRUE
  seeds read thumb2_y in [-0.0415,-0.0029] and the FALSE in [-0.0673,-0.0631].
  Forcing the thumb branch is refuted (s2, -0.0065, flips FALSE). The unseen
  extension 21-40 is 18/20 (FALSE 26, 32) and a coordinator held-out 41-60 is
  18/20 (FALSE 45, 46) -- 36/40 above seed 20, 56/61 over 0-60; the verified
  scope is 0-20 only. Historical chain: drill_c7 afe4899f… was 13/13 on 0-12 with
  a close branched on the finger count, a gate that is blind above 12 (all eight
  unseen seeds read n_fing=0; c7 is 3/8 there); drill_t 86fdd43f… added the 2 mm
  pad-route hull (18/21 on 0-20), then the geom key (20/21), then R2-free widened
  the route entry (38/41). No fixed close plan in 14 measured candidates solved
  s2+s3+s7 together.
- 06: 18 of 21 tested seeds 10-30 (FALSE 16, 18, 28, all ending at t 6.0). The
  three residuals are classed and two are boundaries: 16/28 are a REACH boundary
  -- no base-stance quantity separates them from passers (base x/y/yaw and the
  right shoulder within 3 mm across four seeds), and the separating quantity is
  the object SPAWN (right-shoulder-to-object lateral offset 146-151 mm against
  109-113 mm), which cannot be tuned per seed. s18 terminates at the pick site on
  the first lift row and its end orientation is INSIDE the passing envelope
  (passing seeds end at up to 73 deg tilt). For this task the close-phase
  quantities -- dz across the close, contact count, lift_lost, post-close z_up --
  are REFUTED as predictors of the bit in both directions: two candidates moved
  them the right way, one changed nothing and one lost two passing seeds.
- 07: 0-19 (coordinator 0-13, verifier 14-19) at pp33_a5 0f3d0808… with its 35
  pinned kwargs, catalogued as `carry_and_seat_on_shelf`; 20-40 is 16/21 (FALSE
  21, 22, 25, 32, 38) and a coordinator held-out 41-60 is 15/20 (FALSE 48, 52,
  53, 56, 58), i.e. 51/61 over 0-60. A stall-retry variant (pp38_a6 bed3b297…
  with two extra kwargs, `gait_retry` 0.6 and `gait_retry_mag` 0.012) repairs
  s22 on 0-40 (37/41) but on held-out 41-60 its retry fires on exactly the two
  seeds the pin also fails (48, 56) and repairs neither, so it is a measured
  variant and NOT a supersession -- do not re-derive it.
- 08: at `cand_g24x3cap` 6c39704f… with `kwargs {}` it is 41/41 TRUE over 0-40
  and 18/20 over a coordinator held-out 41-60 (FALSE 41, 48) -- 59/61 over 0-60.
  It supersedes `promoted.py` 035ede59… (36/41 over 0-40) and, on the held-out
  block, `cand_g24x2` fc503928… (58/61; it left s56 FALSE). Mechanism: the
  extraction stage's 8-pull cap is soft while the previous pull row logged
  drill-holder contact, the exit also requires zero contact, and past the 8th
  pull the stage continues while drill x > 0.37 m up to a hard cap of 16. Gate B:
  PASS on the parent (seeds 0/3/10/16/17/36, ownership, torque on forcerange/URDF
  effort, determinism) and PASS on the delta (56/33/10: s33 and s10 bit-identical
  with the same pull and read counts, s56 +2 extract rows = +80 steps with the
  first difference at step 1425 and zero at-cap readings in the added region).
  Remaining failures are boundaries, not tuning targets: s41 is a holder jam (the
  pull target is body-fixed on the object, so the command creeps ~1 mm/pull and
  clearing needs ~85 pulls) and s48 is a grasp slip whose origin is the close
  phase (its close link set equals a passing seed's). The contact-exit mechanism
  is a candidate for a `move_held` variant but is NOT catalogued: the tested
  bytes are a task composition, not a callable skill, and no cross-task scope is
  claimed.
- 09: engine TRUE on 38 of the 41 tested seeds 0-40 at a5_v52_latch 07934f75…
  with {assist:1, assist_roll:180.0, assist_loc:[-0.15,0.0,0.12],
  tilt_gate:0.45}; FALSE on 23, 25, 40. A coordinator held-out 41-60 is 17/20
  (FALSE 41, 49, 59), so 55/61 over 0-60; above the original 0-21
  development range it is 33/39 (22-40 16/19, 41-60 17/20). It supersedes cand_v43 ec699c48…
  {assist:1, assist_roll:180.0, assist_loc:[-0.15,0.0,0.12]}, which was 37/41
  (FALSE 23, 25, 34, 40; 22/22 on 0-21). v52 = v43 plus a latch that fixes the
  object-frame placement once from the entry tilt of the retry block; it differs
  from v43 only on the six retry-firing seeds and recovers 34 (dz +0.0630 ->
  +0.2644, closeN 0 -> 5), losing none. The right-arm assist is NOT catalogued:
  it fires on {12,25,28}, is causally necessary only on seed 12, and is
  bit-neutral on both unseen firing seeds (`library/NOT_CATALOGUED.md`). The six
  failures {23, 25, 40, 41, 49, 59} are ONE class and a MOBILITY BOUNDARY, not a
  reach-tuning problem: hold z 0.838-0.845 m, tilt 73-102 deg, zero hand-object
  contacts. The +x nudge budget is exhausted (base response 0.0003-0.0010 m per
  call after the first), a lateral walk moves the base only 3-5 cm with the feet
  NEVER STEPPING (zero ankle lifts across 41 samples; the stance, not the table,
  is the ceiling), and on s40 the pose is IK-solvable at 2.2 mm yet the WBC
  leaves a 0.163 m palm residual. `move_base` exposes x/y/yaw/dur only, with no
  gait or step request, so no primitive in this composition can reposition the
  base far enough. Do not attempt these six with nudge sizing, retries or
  orientation transport.

# Index

Format: `name(param names only) -- verified task ids [CAUTION]`. Parameter
defaults and meanings are in the description; `...(+N)` marks N further
parameters not listed here.

## 02
- press_until_joint_stop(side, target0, step, max_steps, q_stop, ...(+9)) -- 02 [CAUTION]
- servo_fingertip_to_point(side, target, gain, dur, max_iters, ...(+5)) -- 02 [CAUTION]
- walk_to_reach(target_match, gapx_max, phases, burst_x, burst_y, ...(+4)) -- 02 [CAUTION]

## 04
- hold_commanded_pose(dur) -- 04
- measure_scene_state(leader) -- 04 [CAUTION]
- push_object_to_target_x(target_x, vx, dur, lead, max_bursts, ...(+6)) -- 04 [CAUTION]
- steer_behind_leader(ref_y, ky, kb, eb_soft, cy_max, ...(+3)) -- 04 [CAUTION]
- walk_to_first_contact(app_gap, vx, dur, max_bursts, ref_y, ...(+8)) -- 04

## 03
- align_pads_to_faces(pads, off, iters, dur, x_margin, ...(+11)) -- 03 [CAUTION]
- approach_standoff(box, standoff, closer, max_it, guard, ...(+9)) -- 03
- arm_ramp(side, target, joints, seg, seg_dur, ...(+6)) -- 03,05 [CAUTION]
- limit_safe_close(grip_angle, grip_dur, margin, min_contacts, release_on_fail, ...(+2)) -- 03 [CAUTION]
- reach_opposed_faces(box, gap, x_off, dz, dur, ...(+3)) -- 03 [CAUTION]
- shoulder_lift_hold(commanded, z_start, stand_height1, stand_height2, stand_dur, ...(+8)) -- 03 [CAUTION]

## 06
- carry_gripped_object_to_pose(x, y, yaw_deg, tol, yaw_tol, ...(+7)) -- 06 [CAUTION]
- curl_fingers_onto_handle(angle, dur) -- 06 [CAUTION]
- hoist_gripped_object_in_steps(side, step, max_steps, left_max, right_min, ...(+2)) -- 06 [CAUTION]
- hold_parked_arms_and_measure_drift(hold, right, left) -- 06 [CAUTION]
- level_upright(joints, step, target, min_gain, max_tries, ...(+6)) -- 06 [CAUTION]
- park_hand_clear_of_object(side, other, park, trigger, dz, ...(+2)) -- 06 [CAUTION]
- reach_palm_to_object_standoff(side, back, lateral, z, dur, ...(+1)) -- 06 [CAUTION]
- release_grip_and_settle_in_place(open_angle, open_dur, settle_dur) -- 06 [CAUTION]
- retreat_hands_radially_from_object(right_clear, left_clear, step, max_iter, move_dur, ...(+4)) -- 06 [CAUTION]

## 08
- boost_base_height_holding_command(height, dur, side, obj_tag) -- 08 [CAUTION]
- creep_to_object_standoff(h_off, offset, side, tol, step, ...(+6)) -- 08 [CAUTION]
- move_held(hold, stop, axis, d, side, ...(+10)) -- 08 [CAUTION]
- servo_hand_site_to_point(target, side, tol, step, dur, ...(+2)) -- 08
- walk_base_to_coordinate(axis, target, v, lat, yaw, ...(+2)) -- 08,09 [CAUTION]

## 01
- close_fingers_flip_left(angle, dur, left_finger_flip, rthumb_min, box_ref) -- 01 [CAUTION]
- grasp_lift_floor_box_press_branch(approach_x, h, lift_h, press_ncmd, seated_tol, press_branch, att_tilt_dz, ...(+75)) -- 01 [CAUTION]
- place_pads_branch(tag, stand, xb, zu, ncmd, ...(+27)) -- 01 [CAUTION]
- reach_attitude_guard(records, delta_ztol, level_ztol) -- 01 [CAUTION]
- seat_pads_for_grip(box_ref, press_steps, press_ncmd, press_out0, off, ...(+6)) -- 01 [CAUTION]
- secure_grip_after_close(box_ref, hold, press_records, min_contacts, empty_min, ...(+4)) -- 01 [CAUTION]
- servo_base_to_target_pose(x, y, yaw_deg, upper, tol, ...(+9)) -- 01,06 [CAUTION]
- stand_lift_box(lift_h, lift_dur, sample_every) -- 01 [CAUTION]

## 05
- grasp_verify(match, n_min, gap_min) -- 05 [CAUTION]
- lift_drill_pad_route_geom_close(off) -- 05 [CAUTION]

## 07
- carry_and_seat_on_shelf(con_log, drop, low_dx, low_dy, low_dy_fm, ...(+80)) -- 07 [CAUTION]

## 09
- top_up_reach_at_standoff(target, reach_max, creep, dur, dx, ...(+4)) -- 09 [CAUTION]

