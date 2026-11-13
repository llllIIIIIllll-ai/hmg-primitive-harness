# Primitive lessons (high level, coordinator-maintained)

Format: failure signature -> what worked -> reusable pattern -> verified scope.

---

# Skill catalog

One entry per VERIFIED, reusable whole-body skill. An entry is added only when
the verifier gates (A: unseen seeds; B: joint/torque limits, controller
ownership, determinism; C: cross-task reuse, only when the claimed scope names
another task) and the coordinator held-out seeds have passed, with the source
hash pinned at exactly the source that passed. Prefer the least-specialised
skill with the wider verified scope. Entries state applicability conditions and
the failure signature removed — never step-by-step recipes.

Merge-or-justify rule (consolidation pass, 2026-11-10): a new candidate is
catalogued only if no existing skill can be parameterised to cover it. Before
adding an entry, name the nearest existing skill and say why its parameters
cannot reach the new behaviour. A merge is catalogued only when the merged
source reproduces each variant's verified runs bit-identically (or is re-run
and re-verified on the union of their scopes); superseded entries move to
`library/SUPERSEDED.md` with their old hash and scope, never silently deleted.

Entry format:

```
## <skill_name>(<param>=<default>, ...)
- source hash: <sha256> (<file it names>)
- removes: <the measured failure signature this skill eliminates>
- applies when: <measurable preconditions a caller can test>
- contract: <pre> -> <post>; side effects: <measured>
- verified scope: <tasks / objects / seeds / gates actually passed; known failures; what breaks it>
- used by: <task compositions>
```

Catalog status (2026-11-10, post-migration): 35 entries -- 02 (3), 04 (5),
03 (5), 06 (10: after merging the two uprighting skills into `level_upright`),
08 (5: after merging extract+lift into `move_held`), 01 (6: four from the first
extraction round plus `reach_attitude_guard` and `secure_grip_after_close`),
05 (1: `arm_ramp`, gate A/B/C with a narrowed scope). Superseded entries live
in `library/SUPERSEDED.md` with their old hashes and scopes. Each hash is pinned
at the bytes that passed; each entry's scope line states exactly which gates,
seeds and ablations stand behind it and which do not -- no entry claims more
than its scope line. 04 makes NO cross-task claim (no gate C run; its entries
say so). `library/CORRECTIONS.md` records, per record, the promotion-time
claims that measurement refuted -- read it before quoting a `library/*.json`
domain. 07 and 09 are extraction targets, not catalog sources: nothing from
those tracks is catalogued until its verifier verdict and my held-out seeds
pass.
Every entry below is a SKILL hash, not the monolith hash in `library/*.json`;
its scope is the composition runs it passed, plus the monolith held-out results
it inherits only where the composition reproduces the monolith
element-for-element on those seeds.

## walk_to_reach(target_match='button', gapx_max=0.44, phases=((0.3,0.0,1.2),(0.3,0.05,0.5),(0.25,0.02,0.4)), burst_x=0.22, burst_y=0.02, burst_dur=0.4, max_extra=6, axis=0, joint_index=0)
- source hash: c1c034a8caad49d007aa148980344b578be0503996be8470e87251b686411bd3 (work/tracks/02_push_button/actor_extract/skills/walk_to_reach.py)
- removes: open-loop base walking that stops short of the object and leaves the gap outside arm reach (measured post-walk joint-to-base gaps 0.530/0.247/0.398/0.312/0.305 m on seeds 1,2,3,5,7).
- applies when: the object is observable as a named SceneObject with a readable joint position, the base is holonomically commandable, and the approach is along one base axis (`axis`), with the standoff budget given as `gapx_max`.
- contract: pre: object pose readable at the current base pose -> post: measured object-joint-to-base gap driven at or below `gapx_max` with lateral error bounded by `burst_y`, in a scripted coarse-then-fine burst sequence, or the extra-burst budget spent; returns `{target_pos, target_q, trace}`. Side effects: one object-joint read per burst, base-only nav steps, nav left at zero.
- verified scope: 02_push_button only -- seeds 1,2,3,5,7 (extraction, bit-identical to promoted), 11,12,13 (verifier gate A, 0 differing leaves), 21,31,37 (coordinator held-out, 0 differing leaves). The gap postcondition is NOT met on seed 1 (0.530 m against a 0.44 m gate) and the press still succeeds: the gate is a planning budget, not a precondition the caller may assume. No cross-task (gate C) evidence; breaks when the object has no readable joint or the approach is not axis-aligned.
- used by: 02_push_button composition.

## servo_fingertip_to_point(side='right', target=(0,0,0), gain=0.7, dur=0.45, max_iters=8, tol=0.008, tip_link='index_1', mesh_tag='_vis', watch_match=None, watch_joint=0)
- source hash: 8e04049e134310b26370b9a28c8efcdb6dfecdf00f3f0f2b1bbb0cbfb44f70d1 (work/tracks/02_push_button/actor_extract/skills/servo_fingertip_to_point.py)
- removes: open-loop fingertip placement, which misses by 0.080-0.210 m -- the whole reach error the solution this replaces could not remove.
- applies when: the fingertip site is readable, the target point is expressed in the same frame the site is reported in, and the arm has an IK solution that keeps the palm clear.
- contract: pre: measured fingertip error above `tol` -> post: fingertip within `tol` (default 8 mm) of `target`, or `max_iters` spent; returns `{tip, err, ...}`. Side effects: one fingertip/object read per iteration, one closed-loop reach command per iteration, optional watch-joint read when `watch_match` names an object.
- verified scope: 02_push_button -- same ten seeds as walk_to_reach. Converged in 4-7 reaches on seeds 1,2,3,5,7 but NOT on seed 7 (0.0226 m after 8 iterations, task still succeeds): `max_iters` termination is a normal outcome, so a caller must read the returned residual rather than assume `tol`. No cross-task evidence.
- used by: 02_push_button composition.

## press_until_joint_stop(side='right', target0=(0,0,0), step=(0.003,0,0), max_steps=26, q_stop=-0.0145, object_match='button', joint_index=0, servo_gain=0.7, servo_dur=0.45, servo_iters=2, object_token=..., support_token=..., contact_limit=60, settle_dur=0.6)
- source hash: 4da49591fd7ac96c0069702999dddfdf1800f00ab6e8269b8fa22447831fa2f0 (work/tracks/02_push_button/actor_extract/skills/press_until_joint_stop.py)
- removes: pressing a compliant object for a fixed time or distance -- the button's own slide joint moves from 0 only at the first contact step (measured j=17/17/18/17/17), so a blind press either wastes steps or over-drives into the panel.
- applies when: the target object has a readable joint whose motion measures the press, and there is a fingertip site the caller can servo between steps.
- contract: pre: fingertip at `target0`, object joint at rest -> post: object joint advanced by stepping the fingertip inward `step` at a time, terminated by the joint reaching `q_stop` or by `max_steps`; returns `{button_q, button_pos, ...}`. Side effects: per step a 2-iteration fingertip servo (its own reads) plus object-joint and contact-count reads, then one settle.
- verified scope: 02_push_button -- same ten seeds; the `q_stop` branch is NOT reached at the default -0.0145 (the joint settles at -0.0139..-0.0142 and `max_steps` ends the press) but IS reachable at q_stop=-0.0130 (24 and 22 `press_end` records at t 14.98/14.14, q -0.0137/-0.0136, still in contact). Push depth is compliance-capped: a 34-step press peaks at -0.0143/-0.0144 and then RECEDES to -0.0138, i.e. pressing longer does not press deeper. No cross-task evidence.
- used by: 02_push_button composition.

## measure_scene_state(leader='shelf')
- source hash: e0e9cb205ed606baebabb422afe238ff6bf495e84840e6ad92b3ba67682ca198 (work/tracks/04_push_shelf_forward/actor_extract/skills/measure_scene_state.py)
- removes: a re-read that differs from the promoted program in WHICH engine reads it makes or in their order -- the failure signature is silent, not visible: two copies of one primitive differing only in diagnostic read blocks hold bit-identical commands through step 280 but diverge in physics from the first removed block and can flip the engine outcome at identical command time.
- applies when: a caller needs the scene state and wants the read pattern to stay pinned; it is the single entry point through which a composition should touch the scene.
- contract: pre: none -> post: `scene_objects()`, `geom()`, `observe()`, `base_state()` issued in that exact order, in the promoted count; returns the scene record. Side effects: 3 `mj_forward` forward passes plus one base read; commands nothing, so its only physical effect is the warm-start/derived-state refresh.
- verified scope: 04_push_shelf_forward -- seeds 3,4 (extraction, bit-identical), 5,6 (verifier gate A), 7 (coordinator held-out: 11/11 physical fields identical, t 16.2 = promoted). General beyond the task by construction (nothing in it is cart-specific) but that is an ARGUMENT, not a measurement: no cross-task run yet.
- used by: 04_push_shelf_forward composition (3 calls).

## walk_to_first_contact(app_gap=0.80, vx=0.30, dur=1.0, max_bursts=6, ref_y=None, ky=1.0, kb=0.8, eb_soft=0.0, cy_max=0.08, kyw=1.0, cw_max=0.25, leader='shelf', phase='approach')
- source hash: c042fcd77b392010092c196fa6b8180412467fae06d42cd61184edaf8d35edca (work/tracks/04_push_shelf_forward/actor_extract/skills/walk_to_first_contact.py)
- removes: walking into a push face blind -- the base either stops short and never touches the cart or arrives with yaw/offset error and strikes it off-centre.
- applies when: the leader (cart) and the follower face are both observable, the follower command axis is the base +x, and a standoff `app_gap` can be measured along it.
- contract: pre: base behind the cart face at more than `app_gap` -> post: base advanced in velocity bursts until the measured gap reaches `app_gap`, with lateral/yaw feedback bounded by `cy_max`/`cw_max`; returns `{bursts, trace, contacts, ...}`. Side effects: per burst a `measure_scene_state` read plus one velocity burst, using `steer_behind_leader` for the lateral correction.
- verified scope: 04_push_shelf_forward -- same five seeds. Do not read `bursts` as convergence: the burst COUNT is returned for diagnosis.
- used by: 04_push_shelf_forward composition.

## steer_behind_leader(kwargs: ref_y, ky, kb, eb_soft, cy_max, kyw, cw_max, leader)
- source hash: 59b0fc2c568fe18d96c375274ea8a69889ea5663c7e2ef8282dc0ef08f5cf39a (work/tracks/04_push_shelf_forward/actor_extract/skills/steer_behind_leader.py)
- removes: the cart veering away from the face line -- open-loop bursts accumulate lateral error until the base is beside the cart instead of behind it.
- applies when: a leader object and a follower face are measurable in the base frame, and the caller wants a bounded single-axis correction rather than a full walk.
- contract: pre: leader/follower poses readable -> post: a lateral velocity and yaw rate bounded by `cy_max`/`cw_max` computed from the lateral and heading error, returned as `{y, yaw, ey, eb}`; commands nothing itself. Side effects: reads only.
- verified scope: 04_push_shelf_forward -- exercised only through `walk_to_first_contact`; no direct invocation on any seed, so its contract is measured as a component, not standalone. Do not catalogue a standalone success claim for it.
- used by: walk_to_first_contact and push_object_to_target_x (04) -- both call it once per burst.

## push_object_to_target_x(target_x=None, vx=0.25, dur=1.0, lead=0.55, max_bursts=18, settle_dur=0.0, ref_y=None, ky=1.0, kb=0.8, eb_soft=0.0, cy_max=0.08, ...)
- source hash: 7e095675660dfe9a21cdf8bdab9d3acac5ee642a38fcc95721587104cda1860f (work/tracks/04_push_shelf_forward/actor_extract/skills/push_object_to_target_x.py)
- removes: an open-loop push that keeps rolling past the target or drifts sideways off the face -- contact-free bursts with a bounded lateral lead, plus a settle that exploits the cart's measured coast.
- applies when: a rolling object can be pushed from behind by the base at a face the base can reach, and its x position is measurable each burst.
- contract: pre: base already in contact with the face (call after walk_to_first_contact) -> post: object x driven to `target_x` in bursts of `vx` with lateral/yaw feedback bounded by `cy_max`/`cw_max`, stopping on the measured x or on `max_bursts`; returns `{bursts, trace, contacts, ...}`. Side effects: per burst one scene read plus one velocity burst; optional settle.
- verified scope: 04_push_shelf_forward -- same five seeds; the composed program also calls it for a fine push. Measured: the fine mode ran ZERO bursts on seeds 3,4 but ONE on seed 7 (composition census `{'approach':3,'fast':8,'slow':4,'fine':1}`), so do not claim a verified fine push -- the fine call is mostly one scene read.
- used by: 04_push_shelf_forward composition (3 calls: fast, slow, fine).

## hold_commanded_pose(dur)
- source hash: e7b50be051af0bd80e5890385105cd827a0c71cc4be2e9de75d89a297bd18168 (work/tracks/04_push_shelf_forward/actor_extract/skills/hold_commanded_pose.py)
- removes: a push measurement taken while the base is still coasting -- the cart's own travel after the last burst is part of the placement error.
- applies when: a pose is currently commanded and the caller needs the measured effect of leaving it alone for a known interval.
- contract: pre: a command is in force -> post: `dur` seconds elapse with that command held; returns `{t}`. Side effects: steps the sim with no new command.
- verified scope: 04_push_shelf_forward, same five seeds; no object-specific content, so it is the one 04 skill with no task-shaped parameter at all -- still, no cross-task measurement.
- used by: 04_push_shelf_forward composition (2 calls).

## approach_standoff(box=None, standoff=0.40, closer=0.0, max_it=16, guard=0.20, tilt_limit=25.0, tol_x=0.04, tol_y=0.06, tol_yaw=6.0, stop_x=0.05, vx_gain=0.7, vx_clip=(0.20,0.30), vy_gain=0.7, vy_clip=0.18)
- source hash: 20a6a542e9d4445e3461fcb22b1c1677a473c4bddae46ff91beebea2992c4b28 (work/tracks/03_box_lift/actor_extract/skills/approach_standoff.py)
- removes: grasping from wherever the base happens to stop -- a lateral/yaw offset at the standoff becomes a pad placement error at the faces, and a large pre-grasp reach shortfall drags the box instead of clamping it.
- applies when: the box pose is observable in the base frame, the base is roughly facing it, the ~0.40 m standoff is reachable, and lateral clearance exists.
- contract: pre: base at an arbitrary nearby pose, box at rest -> post: measured box-to-base standoff driven to `standoff` with |lateral| <= tol_y and |yaw| <= tol_yaw, base tilt under tilt_limit, within max_it correction steps; returns a walk log (7 steps on every measured seed). Side effects: one scene read plus one base command per step.
- verified scope: 03_box_lift table-top box, seeds 16,17,18,19,20 (extraction, verifier gate A, coordinator held-out 20 -- composed equals promoted with 0 of 286 compared leaves differing, max delta 0.0, identical t). The `walk_tilt` and `no_object` exits were NOT exercised; `fingertip_guard` cannot be excluded because compose does not abort on it. No cross-task evidence.
- used by: 03_box_lift composition (first step).

## reach_opposed_faces(box=None, gap=0.09, x_off=-0.15, dz=-0.02, dur=2.2, ik_err_max=0.09, via_dur=1.0, retry_dur=1.5)
- source hash: ab501ee38d0f9b8e6e96bcf21832eb45671ddcd8fbdb689ce6e07592f1545639 (work/tracks/03_box_lift/actor_extract/skills/reach_opposed_faces.py)
- removes: reaching both palms to a box with an IK solution that misses the faces -- the error is only visible after the reach, and a big shortfall turns the grasp into a drag.
- applies when: the box is at the standoff with its faces roughly opposed to the palms, and a two-palm IK solution exists within ik_err_max.
- contract: pre: base at the measured standoff -> post: both palms commanded to opposed-face targets with per-side IK error below ik_err_max (measured 0.0609/0.0735 and 0.0615/0.0721 against a 0.09 guard); returns per-side IK errors and a `via_retry` record. Side effects: arm reaches plus the reads that build the targets.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence as above. NARROWED: `via_retry` was empty on every seed and `no_object`/`no_pad` were never observed, so the retry path for a large reach shortfall is UNMEASURED -- do not rely on it.
- used by: 03_box_lift composition.

## align_pads_to_faces(pads, off, iters=4, dur=1.8, x_margin=0.030, y_margin=0.0237, face_off=0.0217, freeze_clr=0.005, near_thr=0.020, capy_far=0.030, capy_near=0.012, cap_x=0.05, cap_z=0.04, done_x=0.030, done_y=0.010, done_z=0.012)
- source hash: 60c0a1fad0da51ff04f82d5cf302bf42e934d7fbdc0a9f1946bf423ce221c42d (work/tracks/03_box_lift/actor_extract/skills/align_pads_to_faces.py)
- removes: a pad that is near a face rather than on it -- a placement-sensitive side clamp slips at the first lift step when the pads are marginal.
- applies when: both pads are visible in the current view and the box has not moved since the reach.
- contract: pre: palms at opposed-face targets -> post: pads corrected onto the faces over `iters` measured iterations (4 on every seed), with per-axis caps and completion thresholds; returns per-iteration corrections. Side effects: per iteration a pad/face measurement plus an arm correction.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence. Requires both pads in view; the `object_lost` exit was never exercised.
- used by: 03_box_lift composition.

## limit_safe_close(grip_angle=1.0, grip_dur=1.2, margin=0.02, min_contacts=2, release_on_fail=True, release_angle=-0.5, release_dur=0.6)
- source hash: 4566cf64936bb4b3c70981926b4f99dd7c16e51e9425451e466f6921d48ecbb8 (work/tracks/03_box_lift/actor_extract/skills/limit_safe_close.py)
- removes: closing the fingers to a fixed angle and assuming a grasp -- a marginal grasp then survives the close and fails at the first lift step.
- applies when: the pads are on the faces and the hand-joint limit magnitude is known (measured 1.0472 against a commanded 1.0, so the command is NOT clamped).
- contract: pre: pads aligned on the faces -> post: fingers closed to grip_angle and verified by a measured hand-object contact count >= min_contacts (measured 4 and 9 at close), with release-on-fail available; returns `closed` and contacts. Side effects: one close command plus contact scans.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence. Contact counts vary 3-9 across seeds, so the min_contacts=2 threshold has little margin evidence; the not-closed recovery path was never taken.
- used by: 03_box_lift composition.

## shoulder_lift_hold(commanded=None, z_start=None, stand_height1=0.82, stand_height2=0.90, stand_dur=1.5, stand_dur2=1.5, settle_dur=0.5, target_rise=0.14, min_rise=0.02, contact_gate_n=3, contact_gate_rise=0.09, iters=8, pitch_step=0.06, ...)
- source hash: e73fda828005ec6c32681c6ac7561f87be8dd0c0a43e3ac4fc461e2f546b4d16 (work/tracks/03_box_lift/actor_extract/skills/shoulder_lift_hold.py)
- removes: a one-shot stand-up that drops the box -- the lift must be ratcheted in gated steps, because contacts transiently break during a rise (HAND_CONTACT_LOSS fired 19/9/20/10 times on seeds 16-19) without the grasp actually failing.
- applies when: the box is clamped with both hands, the arms can start from the captured command, and the target rise is within shoulder travel.
- contract: pre: two-palm clamp held, `commanded` is the grasping posture -> post: object raised in measured samples gated on a contact count (`contact_gate_n`) and a rise gate (target_rise); 5 samples on every seed, final object z 0.957-0.970; returns z series, contacts and the final object pose. Side effects: shoulder/stand commands plus the per-step object and contact reads.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence. The `abort` exit was never exercised. Contacts are transiently lost during the lift (do not read HAND_CONTACT_LOSS as grasp failure).
- used by: 03_box_lift composition.

NOT catalogued for 03: `retract_clear` (c5c200b6...40d2) is CARRIED at its pinned hash but was never invoked on seeds 16-20 -- it is reachable only through compose's `_recover` branch, which never fired (stub-substitution control with five positive firings confirmed the detector). Do not present it as a verified behaviour; it needs a seed that forces a >0.10 m reach shortfall or a failed close.

## servo_base_to_target_pose(x, y, yaw_deg, upper=None, tol=0.05, yaw_tol=6.0, max_iter=10, dur=0.5, vmax=0.26, min_speed=0.22, gain=1.5, yaw_gain=0.02, wz_max=0.4, trace_obj=False)
- source hash: 573c43cb68bcb55637d888d9c819c6833ea510d21ec9d1123d84f3bd28105d54 (work/tracks/06_drill_pnp/actor_extract/skills/servo_base_to_target_pose.py)
- removes: an open-loop base move to a randomized stance -- the error is only visible in the world frame, and the last few cm are below the velocity that moves the base (stiction), so a fixed command both misses and stalls.
- applies when: the base is holonomically commandable and the target is a world pose reachable without dragging a held object.
- contract: pre: live episode, `upper` None or a 31-D posture -> post: xy within tol and yaw within yaw_tol of the target, or max_iter windows spent; nav left zero. Side effects: per window one `base_state()` read and one base-only step.
- verified scope: 06_drill_pnp seeds 0-9 (drill carry) and 01_box_lift_floor seed 3 (cross-task): xy 0.016 m, yaw 0.24 deg, nav zeroed -- so the verified claim is **xy/yaw/nav only**. Breaks/unverified on base HEIGHT (fell 0.052 m in the box scene) and ATTITUDE (4.65 deg tilt); a held object pinning the base makes it plateau. No claim about posture holding with `upper=None`.
- used by: 06_drill_pnp composition, `carry_gripped_object_to_pose`.

## reach_palm_to_object_standoff(side='right', back=0.22, lateral=-0.046, z=0.86, dur=1.2, orientation='keep')
- source hash: a114fcf46afd9d92828b4a5b7ea3d98b5f26bbe9ac9172a650b9ca8b03625c1e (work/tracks/06_drill_pnp/actor_extract/skills/reach_palm_to_object_standoff.py)
- removes: aiming the palm at the object itself -- the hand reaches the grasp column and shoves the object before the fingers close.
- applies when: the object pose is measurable, the approach axis is roughly aligned with the base frame, and the standoff is not occupied.
- contract: pre: base at the measured stance -> post: palm commanded to (obj - back along the approach, lateral, z) with orientation held; returns `{target, residual, obj_after, z_up_after}`. Side effects: one observe plus the arm reach plus two object reads.
- verified scope: 06_drill_pnp seeds 0-9, drill on a table. NARROWED by measurement: the palm actually ends **0.0496-0.0802 m** from its commanded standoff (dominated by a 5-7 cm x shortfall), and compose does not gate on it -- so the postcondition is "reaches toward the standoff", not "arrives at it". Not verified outside an axis-aligned drill handle.
- used by: 06_drill_pnp composition.

## curl_fingers_onto_handle(angle=1.3, dur=1.1)
- source hash: 7df7ca5df6260ff7a4d5c66c3cb374da3be8a53e7b4c56a0f4f37fb3687c7767 (work/tracks/06_drill_pnp/actor_extract/skills/curl_fingers_onto_handle.py)
- removes: trusting an open-loop curl -- a standoff a few mm off closes the fingers on air and every later phase gates on a false success.
- applies when: the grasping palm is at a standoff with the handle between the fingers.
- contract: pre: palm at the standoff, object at rest -> post: fingers at `angle` with the per-side hand-object contact counts MEASURED and returned (3-5 on the grasping side, 0 on the idle side, seeds 0-9). Side effects: one `close_grip` plus two object reads and two contact scans.
- verified scope: 06_drill_pnp, drill handle, seeds 0-9. Breaks when the handle is outside the finger span at the standoff depth -- 08's sibling skill closed around a 20 mm-misaligned handle and the object left the gripper later, so the contact count, not the angle, is the contract.
- used by: 06_drill_pnp composition.

## hoist_gripped_object_in_steps(side='right', step=0.015, max_steps=3, left_max=0, right_min=1, z_up_min=0.85, dur=0.4)
- source hash: 98aae632d26e5e82f937eeca8b606bf440465840c4aac04f8c9b2b6b204a4d00 (work/tracks/06_drill_pnp/actor_extract/skills/hoist_gripped_object_in_steps.py)
- removes: one large lift that yanks the object out of the fingers, or a re-command from a stale posture that silently drops it.
- applies when: the fingers are closed on the object and `api.commanded()` holds that grasp.
- contract: pre: grasp held -> post: object raised up to max_steps*step, re-commanding the finger targets every step and measuring height/uprightness/contacts; aborts on grasp degradation. Side effects: per step one palm read, one reach, one object read, two contact scans.
- verified scope: 06_drill_pnp, drill, seeds 0-9 (full 3-step budget, `lost=False` everywhere). The three abort branches were NEVER exercised -- they are unmeasured code, not verified behaviour.
- used by: 06_drill_pnp composition.

## carry_gripped_object_to_pose(x=0.15, y=0.55, yaw_deg=90.0, tol=0.06, yaw_tol=8.0, max_iter=10, upper=None, recover=2, recover_iter=6, recover_dz=0.02, recover_yaw_tol=8.0, trace_obj=True)
- source hash: 7679c6d0ff85eab19a68e8680f7f88e6b45632b894ae5d03020aaaa6b21fd645 (work/tracks/06_drill_pnp/actor_extract/skills/carry_gripped_object_to_pose.py)
- removes: carrying at the height inherited from a lift -- the held object's lowest collider drags a table edge, the base velocity saturates and heading plateaus short of the goal.
- applies when: a grasp is held and the carry posture is the one captured after the clearance raise.
- contract: pre: grasp held -> post: base near the target pose (8-10 correction windows measured, yaw error 1.89-7.75 deg against 8) with the object pose measured; raises `recover_dz` and re-walks if short. Side effects: the servo's per-window reads plus object/contact tracing.
- verified scope: 06_drill_pnp, drill, seeds 0-9. The stall-recovery branch NEVER fired (every heading was already inside 8 deg) -- do not rely on it. Base tilt reaches 5.03 deg during the carry.
- used by: 06_drill_pnp composition.

## level_upright(joints=(14,15), step=0.08, target=0.996, min_gain=0.001, max_tries=4, dur=0.35, reverse=2.0, sign_reversal=True, read_mode='geom', target_stop='ge', out='two')
- source hash: 869195fcbf42c7f12d416a77fd3c04080c1f9627e82eced8953ece04844ed238 (work/tracks/06_drill_pnp/actor_extract/skills/level_upright.py; the identical bytes also sit at `merge_round/level_upright.py`, where the verification evidence was produced)
- removes: the union of its superseded variants -- a wrist rotation that makes the object LESS upright (keep-only-if-it-gained, rejected probe undone) and the residual tilt about a second wrist axis (coordinate ascent with a one-shot direction reversal).
- applies when: a grasp is held and `nudge_wrist_joint` is registered; one or two wrist axes dominate the tilt. `read_mode` and `target_stop` are PHYSICAL parameters, not diagnostics: 'nudge' uses the nudge's own z_up and adds no read, 'geom' reads explicitly after every probe, opposite probe and restore; 'gt' probes again at exact target equality, 'ge' stops. CALLERS MUST MIGRATE KWARG NAMES: `joint`->`joints`, `delta`->`step`; `**kw` swallows the old names, so an unmigrated call silently runs the TWO-AXIS DEFAULTS.
- contract: pre: measured uprightness below target -> post: one read, then per joint a signed step; each probe is one `nudge_wrist_joint`; accepted if z > z0 + min_gain; on failure either undo (sign_reversal False) or opposite probe then restore; final read. Returns `{log, zup_after}` ('two') or `{zup_before, zup_after}` ('single'). Side effects: reads as per read_mode; the only commands are the nudge calls.
- verified scope: 06_drill_pnp, drill, seeds 0-9 in BOTH modes (stage replay: 0 differing leaves, identical geom/command counts, max delta 0, including the single-axis undo branch on 1,7,9 and the two-axis reversal branch) and seeds 0-9 at the whole-composition level: the canonical `actor_extract/compose.py` b765a328…7236 (old bytes preserved at `compose_superseded.py` bd13f59a…4029) vs the pinned promoted program 455a237a…358f, **0 differing leaves on every seed**, identical t (31.36/23.70/31.46/32.32/31.64/34.66/33.80/29.12/32.04/32.64). Inherited narrowings: it never guarantees reaching target. In the REAL composition the two-axis call on seeds 1 and 9 returns an EMPTY accepted log with z_up BELOW its entry (0.9756 -> 0.9657; 0.9808 -> 0.9649); an empty log means no ACCEPTED probe, not no probe -- probes are issued and reverted, which is why the exit can sit below entry. Per-seed accepted entries are 2,0,3,4,2,3,1,3,5,0 for seeds 0-9. The per-skill stage replay does NOT show this shape because it enters at a different state (z_up 0.9084); the whole-program equality is the binding evidence.
- used by: 06_drill_pnp `merge_round/compose_merged.py` (two calls with the mapped parameters).

## release_grip_and_settle_in_place(open_angle=0.02, open_dur=0.6, settle_dur=1.0)
- source hash: c8eada3fe735ac932e4bd133a412363597acc0d17b954c234c8c2beb3320c15e (work/tracks/06_drill_pnp/actor_extract/skills/release_grip_and_settle_in_place.py)
- removes: opening the fingers while still moving the palm -- the freshly seated object is pushed and rotated off vertical.
- applies when: the object is seated and still gripped with the seated posture commanded.
- contract: pre: object bearing on the support -> post: fingers at open_angle, then the whole posture held still for settle_dur with no palm motion; settled contacts and pose measured. Side effects: one close command plus one hold step plus contact/pose reads.
- verified scope: 06_drill_pnp, drill, seeds 0-9. Detaches on 7 of 8 measured seeds (table 4, hand 0, z_up 1.0, object z 0.7995); on seed 1 it does NOT detach (hand 3, table 0, z_up 0.9337) and the object only lands 0.025 m later during retreat+hold. Breaks when the fingers are wedged so opening necessarily drags the object.
- used by: 06_drill_pnp composition.

## retreat_hands_radially_from_object(right_clear=0.26, left_clear=0.22, step=0.015, max_iter=14, move_dur=0.45, hold_dur=0.35, abort_z_up=0.99, abort_dz=0.004, abort_dxy=0.05)
- source hash: ed8fb9dd4fd773626d4dbcc2ab2413161f58e0edbbc874fafe027dc2f506ef04 (work/tracks/06_drill_pnp/actor_extract/skills/retreat_hands_radially_from_object.py)
- removes: retracting an open hand toward the body or the object's axis -- it brushes the seated object and nudges it (a measured 12 deg tilt after one such park).
- applies when: the object is released and both hands are open and near it.
- contract: pre: object seated -> post: no hand touching and each palm at least its clearance away, or budget/abort; the object is re-measured after each push and the retreat STOPS if it moves. Side effects: one object read per iteration plus the push pair.
- verified scope: 06_drill_pnp, drill, seeds 0-9: final hand contacts 0, table 4 on 8/8. Caveat: on seed 1 its precondition (a released object) was violated because the release failed, and it returned without signalling that -- it does not verify its own precondition.
- used by: 06_drill_pnp composition.

## park_hand_clear_of_object(side='left', other='right', park=0.09, trigger=0.45, dz=0.02, dur=0.9, orientation='keep')
- source hash: 544c8cc87974b1f69a5a9faa4e06ea8c77101ad6deca266eb9644f0b49e525a6 (work/tracks/06_drill_pnp/actor_extract/skills/park_hand_clear_of_object.py)
- removes: an idle hand inside the object's contact shell being swept into the object during grasp/carry, pinning or tipping it.
- applies when: the object is observable, the `side` hand is not grasping, and the palm is within `trigger` of it.
- contract: pre: idle hand near the object -> post: palm moved ~`park` further out along the measured object->palm direction and `dz` up, fingers unchanged; no-op beyond trigger. Side effects: 5 reads in the moving branch, 3 in the no-op branch.
- verified scope: 06_drill_pnp, drill, seeds 0-9 (moved True, idle-side contacts 0, residual 1.8-2.0 cm, palm z +0.024). NARROWED BY CROSS-TASK MEASUREMENT: in a box-on-floor scene the default trigger is a NO-OP at 0.84 m, and when forced it leaves a 0.089 m residual with the palm ending 0.019-0.023 m LOWER despite dz=+0.02. Do not treat it as scene-general.
- used by: 06_drill_pnp composition.

## hold_parked_arms_and_measure_drift(hold=2.0, right='right', left='left')
- source hash: 65e8d576fdeac7049d08177f61f7b2758294fafe97a386ea3fdf8de06021116a (work/tracks/06_drill_pnp/actor_extract/skills/hold_parked_arms_and_measure_drift.py)
- removes: declaring success at the instant of release, when the object is still settling or still touched by a retracting hand.
- applies when: the arms are parked clear and the caller wants the environment's own settling measured.
- contract: pre: arms clear -> post: `hold` seconds of a frozen posture elapsed, object measured twice, drift reported. Side effects: one hold step plus contact/base reads.
- verified scope: 06_drill_pnp, drill, seeds 0-9 (drift 0.0 on 8/9, 0.02498 m on seed 1 -- the measurement that exposes the incomplete release). MEASUREMENT-ONLY: in a box scene it is behaviourally IDENTICAL to an idle `step_goal` (obj_drift 0.0502 both), so it cannot attribute motion to its own arm; and its `table_contacts` counts by the literal body-name substring 'table', reporting 0 while 8 object-floor contacts existed. Read it as a waiter, never as evidence that the arms were still.
- used by: 06_drill_pnp composition.

NOT catalogued for 06, with reasons: `seat_gripped_object_on_support` (6e4034ce...36b9) never meets its own success criterion on any seed -- it exits by exhausting max_steps with table contacts 1, and the object seats only after release, so "seats the object" is NOT a verified claim; `raise_gripped_until_clear` (6046c2cc...230b) only ever took its immediate-exit path (object already above the clear margin 8/8), so its raise loop and grasp-degradation break are unmeasured; `nudge_wrist_joint` (724ab888...48cd) has no independent measurement of any effect (compose discards its return value); `raise_palm_holding_grip` (88414dd2...0127) was executed ZERO times on any seed.

## walk_base_to_coordinate(axis=0, target=0.05, v=0.18, lat=0.0, yaw=0.0, dur=0.5, max_bursts=8)
- source hash: 66f57455e45d293bb63e6b73f8423059b71e3d7482f07912db5e5e205496cd64 (work/tracks/08_pick_drill_from_holder/actor_extract/skills/walk_base_to_coordinate.py)
- removes: a base that stops short of the standoff it was sent to, leaving the arm out of reach before a grasp.
- applies when: the base coordinate along `axis` is measurable and the path is clear.
- contract: pre: base behind the target coordinate -> post: that coordinate driven to `target` (measured 0.099 against 0.05 in 6 bursts) or max_bursts spent; returns the base-state trace.
- verified scope: 08_pick_drill_from_holder seeds 3,4,5,6 (verifier) and 7 (coordinator held-out: composed equals promoted, engine success true, t 34.9 identical). Breaks if the base is blocked or the burst budget is exhausted first.
- used by: 08_pick_drill_from_holder composition.

## servo_hand_site_to_point(target, side='left', tol=0.006, step=0.022, dur=0.5, iters=220, max_steps=12)
- source hash: e8a38255a2b3db99e521c70b991cbf96994abbb7d5fa3097a4cb2548469605b5 (work/tracks/08_pick_drill_from_holder/actor_extract/skills/servo_hand_site_to_point.py)
- removes: reaching a hand site open-loop -- the final few cm are the difference between a grasp and a shove.
- applies when: the hand site is readable and the target is reachable within max_steps.
- contract: pre: hand site measurable -> post: hand site within tol of the target (converged in 7 of 12 allowed steps) or max_steps spent; returns the step record and final pose.
- verified scope: 08 seeds 3-7 (verifier 3-6, coordinator 7). Breaks when the target is unreachable within max_steps.
- used by: 08_pick_drill_from_holder composition.

## creep_to_object_standoff(h_off=(-0.0266,-0.0003,0.1179), offset=(-0.083,0.047,0.012), side='left', tol=0.004, step=0.005, dur=0.35, iters=220, max_steps=40, move_tol=0.02, obj_tag='drill', holder_tag='holder_main')
- source hash: 9455427fed4ce9a3d83b893e6e5f79238a4642a4bb179f606fa5a5d4927f07cb (work/tracks/08_pick_drill_from_holder/actor_extract/skills/creep_to_object_standoff.py)
- removes: creeping to a grasp standoff while the object is being displaced unnoticed -- its measured value is the DRIFT GUARD it trips.
- applies when: object and holder poses are readable and small steps are affordable.
- contract: pre: hand near the standoff -> post: stepped toward the offset until tol, OR `moved` abort when the object's own displacement exceeds move_tol (20 mm), OR the max_steps cap; returns the row log.
- verified scope: 08 seeds 3-7. IMPORTANT: it NEVER reaches tolerance on any seed -- 4,5,6,7 all end on the 40-step cap with `stopped=None`. The verified behaviour is "abort if the object moves 20 mm", NOT "converge to the standoff". On seed 3 the guard fired at j=16 for a 20.1 mm drift, 0.1 mm over the trip, and the run then failed.
- used by: 08_pick_drill_from_holder composition.

## move_held(hold, stop=None, axis=None, d=None, side='left', dur=0.8, iters=250, max_steps=8, clear=0.34, lift_z=0.82, ik_stop=None, obj_tag='drill', bar_tag='holder', label=None, R_body=identity)
- source hash: a8276b1eb041640abb493baa34d96f638a5548c2c29d36dad9e7f76433ccdbed (work/tracks/08_pick_drill_from_holder/actor_extract/skills/move_held.py; the identical bytes also sit at `merge_round/move_held.py`, where all the verification evidence was produced)
- removes: the union of its two superseded variants -- the interleaved pull+lift that catches the held object's top on the holder lip and stalls the rise, and lifting while the object is still inside the holder.
- applies when: a grip is held (`hold` is the commanded hand vector passed to `hand_hold`); object and holder are contact-visible; the stop mode is chosen by `stop` or by which threshold keyword is present (`lift_z` -> `ge`/lift, else `clear` -> `le`/extract). LIFT MODE REQUIRES `ik_stop=0.003` EXPLICITLY: without it the IK-wall break is absent and every trace differs from the superseded skill. Passing both `clear` and `lift_z` without `stop=` silently selects `ge`.
- contract: pre: grasp held -> post: one capped `reach_box` per live object read, stopped when `obj.pos[axis]` crosses the bound in the chosen direction, or (when `ik_stop` is set) when `|ik_err[side]| > ik_stop`, or at `max_steps`; returns `{steps, trace}`. Side effects: per iteration exactly observe -> reach_box -> observe -> geom -> contact_pairs(limit=32); no read added or removed relative to either variant, and the IK-wall row is recorded but not counted in `steps`.
- verified scope: 08 seeds 3,4,5,6,7 in BOTH modes, bit-identical to the superseded sources (extract mode 3424/3424 leaves, 0 diffs, max |delta| 0.0; lift mode 4112/4112, 0 diffs), and the merged composition f733542b…eaa7 equals the pinned promoted program on 3-7 (4664/4664 leaves, 0 diffs, identical t 28.34/34.10/32.50/34.10/34.90). Inherited semantics: the `clear` criterion is met on NO seed (cap-limited); the lift never reaches `lift_z` and stops at the IK wall; on seed 3 the drill leaves the gripper inside the extraction (z 0.718 -> 0.398) and the run fails, reproduced identically. Verified only for a grasp that is on the handle.
- used by: 08_pick_drill_from_holder `merge_round/compose_merged.py` (both calls; the only change vs the pinned composition is the two skill names).

## boost_base_height_holding_command(height=0.87, dur=1.6, side='left', obj_tag='drill')
- source hash: 5f4a8f8f403c3a56069f17333a6200e64baa4e63dd42a511784cb0532e3caa80 (work/tracks/08_pick_drill_from_holder/actor_extract/skills/boost_base_height_holding_command.py)
- removes: a held object that the arm cannot raise any further -- raise the BASE while the closed-hand command is frozen, so the object rises without a re-reach.
- applies when: a grasp is held and `api.commanded()` is not None (otherwise it returns raised False).
- contract: pre: grasp held with a captured command -> post: base height set to `height` with the 31-D command unchanged; returns `{raised, obj, z_up, base}`.
- verified scope: 08 seeds 4,5,6,7. This is what produces the final height (drill z 0.8126-0.8247) and success depends on it. Unverified without a non-None command.
- used by: 08_pick_drill_from_holder composition.

NOT catalogued for 08: `ramp_close_fingers` (a7ccef8a...1e73) executes its four stages (0.3/0.6/0.9/1.2 rad) on every seed, but it is NOT verified to produce a holding grasp -- on seed 3 it closed around a 20 mm-misaligned handle and the object left the hand during the extraction. Do not cite it as a grasp primitive; pair any grasp with a measured contact check (see `curl_fingers_onto_handle`).

## arm_ramp(side='left', target=None, joints=None, seg=None, seg_dur=0.30, max_step=0.25, hold=0.0, n_read=0, dt=0.20, tol=0.06, phase='arm_ramp')
- source hash: ab2746e89e7d10856173d8aaaeacc9bc6e1800ba5d4dddc5f383fc3aae9dd1ae (work/tracks/05_drill_lift/actor_4/skills/arm_ramp.py)
- removes: an arm stuck in the WRONG IK BRANCH of the same palm-pad site -- on 05 seed 1 the second pinch settles with j5 +0.4555 against +2.619 on the passing seeds (~2.16 rad) and the grip closes on air; transporting the joint vector to the passing branch fixes it, and the same seeds flip to failure with the transport disabled.
- applies when: the caller supplies the target joint sub-vector (the skill does NOT plan it), the arm joints are free to command, and a prior command is in force. It does not check balance, does not reject a target outside the joint range, and `n_read >= 4` raises AttributeError at this hash (a documented parameter is unusable; use `n_read=0`).
- contract: pre: target vector in hand, arm free -> post: the 31-D upper goal driven to `target` through `api._ramp`, split into `seg` segments or `ceil(max|delta|)/max_step`; returns `{achieved, err, max_err, tracked}`. Side effects: `upper`/`commanded` reads, the `_ramp` substeps, optional dwell reads and `hold`.
- verified scope: 05 seeds 0-3 (engine true for the composition) and the two unseen ik_seed-0 seeds 9,11 (engine true WITH the ramp, false without). Cross-task on 03_box_lift (gate C): residual 0.0453 rad (2.6 deg), tilt no worse than an idle hold, box displacement 0.0001, zero hand-object contacts. NOT verified as a target planner; NOT verified to respect joint limits (its own `br_arm` j5 2.6191 exceeds the 2.618 limit, so the successful configuration runs at the joint stop); and the claimed SEGMENTATION benefit is REFUTED -- seg=1 gives max_err 0.0339 `tracked 1`, seg=10 gives 0.0636 `tracked 0`. It does not fix unseen ik_seed-2 seeds.
- used by: 05_drill_lift `drill_lift_v18.py` branch mode.
- merge-or-justify: no existing entry takes a JOINT-VECTOR target (`move_held`, `servo_*` and `reach_*` plan a Cartesian pose, `nudge_wrist_joint` moves one joint), so this is not a parameterisation of an existing skill.

NOT catalogued for 05, with reasons: `pad_hold_audit` (4e807d3b...285f) is an AUDIT -- it reports a settled drift (measured 31.4 mm, not the claimed 6.3 mm) and corrects nothing; `base_place` (d3c5f6bf...287b), `pad_place` (0d0545b5...a7ba) and `pad_servo` (9e4e6c71...bd7c) are measured NON-REPAIRS (base moves ~1 cm per 9 cm commanded; pad_place never converges; pad_servo's settled error grew at gain 1.0 and oscillates at 1.8).

## place_pads_closed_loop(tag='place', stand=0.0, xb=0.0, zu=0.0, ncmd=2, free=True, hold=None, out0=None, dur=0.40, lo=None, hi=None, box_ref=None, x_in=0.03, z_frac=0.94, pad_half=0.0217, off=(0.076,-0.018,-0.030), box_move_tol=0.02, box_move_hard=0.25)
- source hash: dd05958a86e3f27dcf548f42cdb326c59101a4e5977b3bf06480dbf5f3461b82 (work/tracks/01_box_lift_floor/actor_4/skills/place_pads_closed_loop.py)
- removes: a one-sided grip from an unseated pad -- the pad centre sits at the commanded IK site plus a fixed world offset, so an open-loop site leaves the pad 1-6 cm clear of its face; and a CUMULATIVE displacement gate aborts every later pad correction once one command has swept the box.
- applies when: box at rest with a `box_ref` settled after the crouch (never taken while the box is airborne), both palms have a two-palm IK solution, and the caller supplies the box AABB. The caller owns the base and the fingers.
- contract: pre: settled box_ref as above -> post: up to `ncmd` measured corrections of both palm sites, each gated on the displacement THAT command caused; returns `{ok, record, log, contacts, sides}`. Side effects: per command one `reach_box` (steps `dur` s), one `geom_detail`, two `observe`, one `hand_contacts_side`.
- verified scope: 01_box_lift_floor, floor box. Seated bilaterally on seed 5 (verifier, independent pn4) and re-scored from saved responses on 3,4,7,8; NOT seated on 0,2,6,9,10,11,12. It does NOT recover a gross miss: on unseen 10/11/12 the press aborts at 3/2/2 commands with one pad 4.5-6.6 cm clear (and those episodes still lift). On coordinator held-out seed 13 one retry recovers the reach but the close is c2 L1/R1 and the box is shoved (AABB top 0.276). Composition traces are bit-identical to the pre-extraction candidate `cand_v8` (05b76db4...662e) on seeds 5,10,11,12. No cross-task evidence.
- used by: 01_box_lift_floor composition (free-space reach and the press's inner loop).

## seat_pads_for_grip(box_ref=None, press_steps=None, press_ncmd=4, press_out0=0.004, off=(0.076,-0.018,-0.030), x_in=0.03, z_frac=0.94, pad_half=0.0217, box_move_tol=0.02, box_move_hard=0.25, place_dur=0.40)
- source hash: f0d78ba66919b105b2190f8e877aeb7b256c50659f04de7507f57a2833ad3b5a (work/tracks/01_box_lift_floor/actor_4/skills/seat_pads_for_grip.py)
- removes: a grip that closes with the right pad only -- with 1-3 correction commands per press step, seed 5 closes c3 L0/R3.
- applies when: the pads were brought to the face pair by `place_pads_closed_loop`, `box_ref` is settled, and the box is at rest.
- contract: pre: pads at the face pair -> post: for each press step (default [0.012,0.003,-0.006] m of inboard stand offset) the pads are commanded normal to the faces with `press_ncmd` measured corrections each; stops at the first step whose per-command displacement gate trips; returns `{ok, failed_at, records, contacts, sides, commands}`. Commands arms only and adds NO trailing observation, so the caller can close immediately without changing the read sequence. Side effects: one `geom_detail` per press step plus, per command, the `place_pads_closed_loop` read block.
- verified scope: 01_box_lift_floor, floor box. `press_ncmd=4` seats BOTH pads on seed 5 only (press2 ends c4 L2/R2, clear L -0.0008/R -0.0022; close c5 L1/R4; dz +0.316; engine success true). On unseen 10/11/12 it aborts after 3/2/2 commands and leaves one pad 4.5-6.6 cm clear -- and the task STILL passes. pn2/pn3 on seed 5 are proxy failures but engine successes (t 18.82/20.42) and all 8 settings of the T2 grid on seeds 3,4,5 at pn2 pass, so "fewer than four press commands fails" is FALSE as a task claim. No cross-task evidence.
- used by: 01_box_lift_floor composition.

## close_fingers_flip_left(angle=0.9, dur=0.8, left_finger_flip=1, rthumb_min=0.30, box_ref=None)
- source hash: 306ed649c99f9d370dc8e638e562cbcb87127669c61980bf1b32a042bd106024 (work/tracks/01_box_lift_floor/actor_4/skills/close_fingers_flip_left.py)
- removes: an OPEN hand at lift time -- measured, not the flip: a no-close control on seed 10 leaves the box at dz 0.0000 and engine success false at t 16.42, while every closed variant lifts it.
- applies when: both pads are on the opposed faces and a valid upper-body command is in force; the caller owns the lift.
- contract: pre: pads seated -> post: one smoothstep ramp (`dur` s at 0.02 s steps) of the hand joints to +-`angle` clipped to each joint range, left index/middle negated when `left_finger_flip` is truthy; returns `{target, finger_cmd, contacts, sides, rthumb_cmd, moved_l1_m, record}`. Commands hands only. Side effects: `upper` and `joint_ranges` queries, the ramp, then observe -> hand_contacts_side -> hand_contacts_side -> observe; this exact read order is part of the verified behaviour.
- verified scope: 01_box_lift_floor, floor box, seeds 3,4,5,10,11,12. `left_finger_flip` is INERT for pass/fail (flip 0 vs 1: identical categorical set on every seed; seeds 11,12 engine t identical at 17.28/17.62; only seed 10 moves, 18.02 -> 19.96). It does NOT remove a right-side-only grip: at pn2 seed 5 the close is c3 L0/R3 with the flip ON as well as off. It shifts post-squeeze side counts and drag. A one-sided 1-2 contact close is sufficient for the task. No cross-task evidence.
- used by: 01_box_lift_floor composition.

## stand_lift_box(lift_h=0.68, lift_dur=2.5, sample_every=20)
- source hash: 156ea4bad3032f8b1ff4733a5b2df6c21d9d5594d698b92790ce8238924942f5 (work/tracks/01_box_lift_floor/actor_4/skills/stand_lift_box.py)
- removes: a base rise that also re-solves the arms -- the measured profile raises only the base height (nav zeroed) with `upper` frozen at `api.commanded()`, and the already-closed box rises with the body (dz +0.319/+0.330/+0.316 on seeds 3/4/5, +0.353/+0.271/+0.261 on 10/11/12, AABB top 0.823/0.834 on held-out 14/15).
- applies when: a grip is closed AND a prior command is in force. It does NOT guard `api.commanded() is None`: called first in a fresh episode it raises TypeError at t 2.5 (seed 10, engine false) and the response wrapper still reports `ok: true` while the episode reports `ok: false`.
- contract: pre: grip closed, last commanded upper pose is the grasp pose -> post: base height smoothstep from the measured height to `lift_h` over `lift_dur` at 0.02 s steps with nav=[0,0,0]; returns `{h_from, h_to, samples, log, commanded}`. Side effects: every 20th step one `observe` + one `geom_detail` + one `hand_contacts_side`; commands height and nav only, never the arms.
- verified scope: 01_box_lift_floor, floor box, ONLY as the terminal step after a close (seeds 3,4,5 verifier; 10,11,12 gate A; 14,15 coordinator held-out). It is not verified standalone -- the precondition is real and unguarded. The lift is load-bearing: with the close removed the box does not rise at all. No cross-task evidence.
- used by: 01_box_lift_floor composition.

## reach_attitude_guard(records=None, delta_ztol=0.10, level_ztol=0.12)
- source hash: 607e1f195a0eeeadd58d6e7ef179f0fc586ee43ac9350c17045e6a19b4f4bad2 (work/tracks/01_box_lift_floor/actor_5/skills/reach_attitude_guard.py)
- removes: a free-space reach that ends with a palm pad ROLLED AWAY from its face, which the following press then slides up and over instead of seating on it (seed 6: right-pad normal-z 0.028 -> -0.183 over the last free-run step).
- applies when: the caller holds the record list returned by a two-pad free-space reach whose records carry per-side pad normals. It makes NO engine call of its own.
- contract: pre: those records -> post: `{retry, reason, side, dz, z_first, z_last, samples, t0, t1}`; `retry` is true iff some side trips roll_away (normal-z drop > `delta_ztol`) or down_tilt (last value < -`level_ztol`). Side effects: NONE -- the `api` parameter is never used; it is a pure selection over caller data.
- verified scope: 01 box, seeds 0-21. Fires on seed 6 only (`attitude_retries` 1, 0 on every other seed); with it disabled seed 6 is engine FALSE (t 17.68, z_hi 0.3005) and with it enabled TRUE (t 23.22). Non-firing records appear on seeds 0,3,8,13; on seed 6 the stored `reach_attitude` field is None, so count firings from `attitude_retries`, not from that field. Masking is impossible: a firing candidate's dz is strictly more negative than any non-firing one. It triggers a retry; it never moves anything itself.
- used by: 01_box_lift_floor `grasp_lift_floor_box_guarded.py`.

## secure_grip_after_close(box_ref=None, hold=None, press_records=None, min_contacts=4, empty_min=2, seated_tol=None, seated_min=-0.02, stand_deep=-0.013, squeeze_ncmd=2, ...)
- source hash: 387eddc9627b7a5a1293e7dd7db0f5824fbf38752192e3e2b094b6470bc44f0b (work/tracks/01_box_lift_floor/actor_5/skills/secure_grip_after_close.py)
- removes: a squeeze that DESTROYS an already-seated press -- when both pads are seated but the raw contact count is below `min_contacts`, the unconditional squeeze drags the hand deeper and the grip is lost (seed 13: press seated, close c2, squeeze fired, lift contacts all 0, engine false).
- applies when: the caller owns the close-and-squeeze phase and holds the last press record with `clear_m` per side; `hold` is the closed-hand target from the finger close.
- contract: pre: press records available -> post: the squeeze runs only when NOT both_seated (both `clear_m` in [`seated_min`, `seated_tol`]) and contacts < `min_contacts`; the empty-grip recovery runs only when contacts < `empty_min`; returns the log plus `both_seated`/`squeezed`/`recovered`. Side effects: adds no engine read and no command beyond the pinned path -- the guard can only REMOVE commands.
- verified scope: 01 box. With `seated_tol=0.0`: seed 13 flips engine FALSE (t 23.96) -> TRUE (t 21.62, lift contacts [4,4,4,4,5,5]) and seed 0 stays TRUE with a slightly lower z_hi (0.9003 -> 0.8847); 14 of 16 seeds are bit-identical in records AND command streams. Inside the band but unchanged: seeds 1,3,5,7,8 (their contacts were already above `min_contacts`, so no squeeze was ever commanded). With `seated_tol=None` it is byte-identical to the pinned behaviour and seed 13 still fails. The band is a pad-seat proxy calibrated on this box, not a grip predicate. It does NOT fix seed 20 (the press itself failed at step 0 there).
- used by: 01_box_lift_floor `grasp_lift_floor_box_guarded.py`.

NOT catalogued for 01: `grasp_lift_floor_box` (e080f9ef...ee41) and `grasp_lift_floor_box_guarded` (438eb7e8...5957) are task COMPOSITIONS, not primitives; `seat_pads_reach_aware` (f83c8606...e68f) is a NO-OP DUPLICATE of `seat_pads_for_grip` at its shipped defaults (all 15 trace arrays identical on seeds 0,5,7,13) and its enabled thresholds regress seed 7 (z_hi 0.8834 -> 0.838), with no seed improved. Engine status of the guarded composition: TRUE on seeds 0-19 and 21, FALSE on seed 20 -- the task is not solved on unseen seeds.

- Press a slide button: open-loop arm reaching never bottomed the button -> what worked: closed-loop servo of the measured fingertip, stopped by the button's own slide-joint state -> reusable pattern: object-state-terminated press servo (walk to a measured standoff, then step the fingertip inward while watching the object's own joint reading) -> verified scope: 02_push_button, verifier seeds plus coordinator held-out seeds; task-specific, no broader scope claimed.
- Push a wheeled cart: open-loop walking never stops at a sampled floor target and the cart veers and keeps rolling -> what worked: closed-loop burst walking at three speed scales with lateral/yaw feedback bounded so the base stays behind the cart face, plus a settle that exploits the measured coast -> reusable pattern: contact-free whole-body push with a bounded lateral lead and coast compensation -> verified scope: 04_push_shelf_forward, verifier plus held-out seeds; single-task scope (needs a rolling cart and a push face at hand height).
- Table-top box lift (verified on a narrow domain): a placement-sensitive two-palm side clamp; marginal grasps slip at the first lift step, and a large pre-grasp reach shortfall can drag the box -> what worked: live object pose/AABB re-reads before every plan step, per-hand measured-pad placement with the fingers held closed, two-stage stand-up plus settle, gentle shoulder-driven lift with a contact-aware stop, and empty-grasp detect/retract/retry -> reusable pattern: measured-pad closed-loop side grasp plus shoulder lift with contact gates -> verified scope: 03_box_lift seeds 16-20; known failures on seeds 5 and 7; no transfer to 07.
- Drill from a cradle holder (verified): an interleaved pull+lift catches the drill's motor top on the holder lip, tilts it and stalls the rise -> what worked: extract horizontally along the slot bars until the drill clears the holder, only then lift, and finish with a base-height boost that raises the held drill while the closed-hand command is preserved -> reusable pattern: clearance-first extraction plus a reach-independent height boost -> verified scope: 08_pick_drill_from_holder. MEASURED at the pinned source `035ede59…80f9` (len 8069, exactly the bytes the extraction actor ran): seed 3 FAILS -- creep abort `moved` at j=16, drift 20.1 mm > `move_tol` 20 mm, drill leaves the holder and lands on the floor at [0.5827,-0.1432,0.0148], z_up 0.914; seed 4 holds, final_drill [0.2862,-0.0124,0.8247], z_up 0.982. The `library/pick_drill.json` claim of "gate A 3/3 unseen seeds (3,4,5) engine success" is therefore NOT reproducible from the pinned source and must not be repeated; the record's seed-3 discipline claim is a catalog error, not a different-source artefact. Corrected domain: seed 4 measured; 5,6 pending verifier; 7 coordinator. Do not carry the 3/4/5 line into a catalog entry.
- Floor box lift (skill extraction VERIFIED; task 14/16, not solved): a tall box on the floor needs a deep crouch and an extreme-reach two-palm side grasp -> what worked: base approach and high guard, deep crouch with a base-x servo, a settled box_ref taken after the crouch, measured-pad closed-loop placement (x before y, gated per command), a three-step normal press (bilateral seat only on seed 5), a finger close, and a base-only stand with the arm command frozen -> reusable pattern: measured-pad placement inside an extreme crouch, gated per command, plus a command-frozen base rise -> verified: engine success TRUE on seeds 0,1,2,3,4,5,7,8,9,10,11,12,14,15 and FALSE on 6 and 13 = 14/16. MEASUREMENT RULE from this track: the engine predicate is far more permissive than a bilateral-grip proxy -- it passes with a one-sided close (1-2 contacts) on 10,11,12,14,15, so the actor's proxy-only "6/10" understates the task by six seeds and must not be quoted as its success rate. Known failures: seed 6 (the press normal drives the right pad away, clear 0.0534 -> 0.1102, no carry, engine false) and seed 13 (coordinator held-out: retry recovers the reach, close c2 L1/R1, box shoved, AABB top 0.276). Gate B: no violation introduced (composition traces bit-identical to the pre-extraction candidate on 5,10,11,12), but the task itself runs over the control range on 10.8-15.6% of steps and exceeds joint limits on every seed (worst -0.040 rad, right_ankle_pitch); NOT safety-clean.
- Drill pick and place (verified on a broad within-task domain): a light drill tips on first touch, a closed grip relaxes under plain base motion, and a set-down released with only two one-sided support contacts tips flat even when the drill was upright just before release -> what worked: contact-free pre-shaped walk-in, closing a few centimetres short so the fingers wrap the handle, carrying with step_goal(commanded) rather than move_base, and a v6 endgame that levels the wrist in two axes, lowers in small steps while still gripped until the object bears on the support, opens the fingers while in contact, and then parks both arms clear with no palm motion after release -> reusable pattern: shape-then-close grasp, command-preserving carry, contact-confirmed seat-then-release -> verified scope: 06_drill_pnp engine success on seeds 0-9 with the exact pinned source `library/drill_pick_place.json` (gate A 3/3 on unseen seeds 1,3,5; gate B pass; seed-5 repeat bit-identical). The stricter actor claim (four support contacts, uprightness 1.000, zero hand contacts) does not reproduce on unseen seed 1 (2 hand contacts, z_up 0.957, ~16.9 deg tilt) even though the task succeeds; the engine predicate, not the stricter end state, is the verified boundary.
- Observation reads are physical actions (harness-level lesson, not a primitive): two copies of the same primitive that differ only in whether the diagnostic `api.geom()` reads are present diverge although the 31-D commands are bit-identical through step 280; the physical state first differs at step 180, exactly the first removed diagnostic block, at float-noise magnitude, then exceeds 1e-6 rad by step 205-260 -> what worked: treat the exact source as the verified artefact and never strip reads out of a promoted primitive without re-running gate A -> reusable pattern: verification pins a source hash, not an algorithm; an "observability only" edit is a dynamics edit -> verified scope: drill_pick_place seed 5 (success in both, 2.17 mm placement shift, t 34.66 vs 33.88) and seed 1 (success flips true -> false at identical t 24.34).
- Box on a cart to a shelf (07, in progress, not solved): a ~10 g open-top box; long fingers sweep it if approached down/forward, and fingers-forward along the outer walls is the only gentle approach found; a one-sided handle pinch lifts 5-6 cm but tilts 13-16 deg. Two-hand identity-orientation handle-bar straddle pinches work. Measured in the pp16 batch (4 settings x seeds 0-3, 16 episodes, 395 s): every walk burst ends in leg-to-cabinet contact at base y -0.355 (449-794 N in `robot0_*_hip_yaw_link`/`g3_col`/`g12_col`), one seed wedges the pelvis on the table slab, and one stalls contact-free; low-height walking tracks only ~6% of the commanded velocity, and the release itself helps (+0.026 to +0.068 m forward margin) but cannot close the gap (`fm_stop` never binds, so three of four settings are byte-identical). The remaining blocker is that no remaining walk burst moves the box after the approach saturates -> next lever is pushing at walk height plus an early walk-drop stop, not a deeper release gate.
- Left-arm drill lift from the support (05, ACTOR ROUND DONE, not solved): on one seed the fingers close on air and the drill never leaves the support (final z 0.82-0.84 vs a 0.92 pass bar, grip ncon 0) while the same source passes the other three seeds -> what was measured: the second pinch's `reach_box(..., prefer_current)` settles on the *other IK branch* of the same palm-pad site (settled j5 +0.4555 vs +2.619, ~2.16 rad); IK error is small but the settled pad is ~9.3 cm out (3.1 cm only in a single post-reach phase sample of a ~1.2 s, 1-5 cm oscillation) -> what worked: transport the arm joint vector to the passing branch (`arm_ramp`), then audit the pad dwell -> reusable pattern: for a repeated pinch, decide from the SETTLED joint vector and pad pose, never from IK error or an immediate post-reach sample -> verified scope: 05_drill_lift seeds 0-3 engine TRUE (4/4); unseen 4-12 is 2/9 -- the repair is confined to the ik_seed==0 family (unseen 9,11 pass with it and fail without; unseen ik_seed-2 seeds 4,5,6,7,8,10,12 all fail at z 0.8383), and the SEEN ik_seed-2 seeds 0,2,3 pass, so the branch label alone does not decide success. The next mechanism is something else that separates seen from unseen seeds. Rejected and not to be repeated: pad-space servoing (`pad_servo`/`pad_place`/`base_place`, all measured non-repairs), one large unsegmented ramp is NOT worse than a segmented one (that mechanism claim is refuted -- the transported vector is what matters), and a `force` gate that corrects every seed (1/3 on unseen and triples episode time).
- Obstacle-aware drill pick (09, in progress, not solved): an extended arm disturbs the drill before the base is in reach and the final servo cannot recover -> what was tried in the v8 pilot (2 settings x 6 seeds, 12 episodes, 323 s, 26.9 s/episode): pre-grasp base placement (`place0` in the body frame, `place1` replacing an x-only nudge) plus an `assist_final` toggle -> result: the placement edits added no seed; the held set is exactly v5's seeds 1/3/5 (hold ncon 3-4, z 1.02-1.04) and the other three stay at z 0.799 with ncon 0 -> reusable pattern: before any reach-dependent grasp, measure the pre-grasp placement residual and base yaw and treat them as a precondition, not something a later arm servo can repair -> measured boundary: `place0` residual d1 <= 0.028 m -> held, d1 >= 0.084 m -> untouched; the 12 mm object-motion guard always fires on the last executed burst, so success is decided by whether convergence happened before that contact. Root cause measured: `move_base` tracks only ~50% of the commanded displacement (5-7 cm per 11 cm command) and yaw degrades with translation (-2.5 to -47.8 deg over 5 bursts on seed 4). A world-frame offset variant is worse (direct palm strike, tips the drill).
