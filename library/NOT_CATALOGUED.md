# Not catalogued (coordinator, 2026-11-10)

Candidates measured on a track that are NOT catalogued, with the reason. Moved
verbatim out of `PRIMITIVES.md` in the 2026-11-10 compaction so the catalog
stays one screen per skill. Nothing here is a recommended skill: a name in this
file is a warning, not a primitive. Full pre-compaction catalog text:
`library/evidence/PRIMITIVES_precompact_2026-11-10.md`.

NOT catalogued for 03: `retract_clear` (c5c200b6...40d2) is CARRIED at its pinned hash but was never invoked on seeds 16-20 -- it is reachable only through compose's `_recover` branch, which never fired (stub-substitution control with five positive firings confirmed the detector). Do not present it as a verified behaviour; it needs a seed that forces a >0.10 m reach shortfall or a failed close.

NOT catalogued for 06, with reasons: `seat_gripped_object_on_support` (6e4034ce...36b9) never meets its own success criterion on any seed -- it exits by exhausting max_steps with table contacts 1, and the object seats only after release, so "seats the object" is NOT a verified claim; `raise_gripped_until_clear` (6046c2cc...230b) only ever took its immediate-exit path (object already above the clear margin 8/8), so its raise loop and grasp-degradation break are unmeasured; `nudge_wrist_joint` (724ab888...48cd) has no independent measurement of any effect (compose discards its return value); `raise_palm_holding_grip` (88414dd2...0127) was executed ZERO times on any seed.

NOT catalogued for 08: `ramp_close_fingers` (a7ccef8a...1e73) executes its four stages (0.3/0.6/0.9/1.2 rad) on every seed, but it is NOT verified to produce a holding grasp -- on seed 3 it closed around a 20 mm-misaligned handle and the object left the hand during the extraction. Do not cite it as a grasp primitive; pair any grasp with a measured contact check (see `curl_fingers_onto_handle`).

NOT catalogued for 05, with reasons: `pad_hold_audit` (4e807d3b...285f) is an AUDIT -- it reports a settled drift (measured 31.4 mm, not the claimed 6.3 mm) and corrects nothing; `base_place` (d3c5f6bf...287b), `pad_place` (0d0545b5...a7ba) and `pad_servo` (9e4e6c71...bd7c) are measured NON-REPAIRS (base moves ~1 cm per 9 cm commanded; pad_place never converges; pad_servo's settled error grew at gain 1.0 and oscillates at 1.8).

NOT catalogued for 01: `grasp_lift_floor_box` (e080f9ef...ee41) and `grasp_lift_floor_box_guarded` (438eb7e8...5957) are task COMPOSITIONS, not primitives; `seat_pads_reach_aware` (f83c8606...e68f) is a NO-OP DUPLICATE of `seat_pads_for_grip` at its shipped defaults (all 15 trace arrays identical on seeds 0,5,7,13) and its enabled thresholds regress seed 7 (z_hi 0.8834 -> 0.838), with no seed improved. Engine status of the guarded composition: TRUE on seeds 0-19 and 21, FALSE on seed 20 -- the task is not solved on unseen seeds.

NOT catalogued for 09: `assist_right_roll_press_at_residual` (cc5a6b87...2341) is a measured one-seed corrective, not a verified skill. The gate (creep residual `rem > 0.03` m) opens on exactly seeds {12, 25, 28} of the 41 tested (0-40), rem 0.0686 / 0.0526 / 0.0924 m. Paired runs on the same pinned composition (ec699c48...b7fe) and kwargs, the only change `assist_rem=10.0` forcing `fire=0`: s12 TRUE t 90.14 vs FALSE t 88.94 (the suppressed run is bit-for-bit the v29 baseline time); s28 TRUE t 64.96 vs TRUE t 63.76; s25 FALSE t 64.74 vs FALSE t 63.54. So the reach is causally necessary on ONE seed that was used during development, bit-neutral on both unseen firing seeds, and it has no positive unseen-seed evidence — gate A is not met. Two further falsifiers: (1) the source docstring inverts the arrangement — its own defaults (roll 90, loc [-0.15,0.045,0.135]) are engine FALSE on s12 (coordinator run 00:30, and the actor's live 23:42 probe), while the arrangement it calls rejected (roll 180, loc [-0.15,0.0,0.12]) is the one that works; (2) at the working arrangement the firing path produced ONE right-hand contact (middle_1), not the documented four-contact cross-plate pinch. The bytes stay registered for the 09 composition and are pinned in the 09 track record; they are not a cross-task primitive.

NOT catalogued for 05 (actor_8, 2026-11-11): `a8_clr` (460ddf18...18cf) and
`a8_clr2` (ff1ad164...ef25) are clearance-first pad uplifts before the branch
ramp, in two frame forms (ship `seat_*` rel; measured `pad_rel + [0,0,+0.06]` m).
Both are measured COIN FLIPS, not fixes: the uplift zeroes the ramp's object
displacement on s26 (55.2 -> 0.0 mm; drill z 0.838 -> 0.921 m) and creates it on
s41 (8.2 -> 19.5 mm; 0.928 -> 0.838 m). On the 14-seed label set (5,10,15,20,22,
26,30,32,35,41,45,46,50,55) form B is 9/14, the ship is 9/14, form A 7/14 (it
fires on vetoed seeds and loses s5/s10). The hand servo cannot execute the uplift
at the pre-ramp pose (s26 commanded (+0.060,0,0) m, achieved
(+0.0741,-0.0017,-0.0331) m, residual 0.060 -> 0.119 m), so the block perturbs
the arm configuration more than it raises the pad; 7 of 10 seeds that ran it
achieved <= +0.02 m z. At `clr_on=0` form B is bit-identical to the ship on six
seeds (5,13,20,26,29,32) across final_obj, final_ncon, close_mode and the whole
`t` list, so the negative is about the uplift, not about the hook. Record:
`work/tracks/05_drill_lift/actor_8/DEADEND_clearance_uplift.md`.

NOT catalogued for 07 (actor_6, 2026-11-11): `pp34_a6` (78adb367...dc31) =
pp33_a5 0f3d0808... with `latch_rows=3`, truncating the retreat's lateral
command after its third burst, is REFUTED. It leaves the retreat depth
unchanged (retreat-end base x within 3 mm of the pin on s1/s5/s6) and instead
raises pre-walk base y (s1/s5/s6 +0.022/+0.025/+0.074 m), after which the
diagonal walk stops translating: walk-phase dy goes -0.324/-0.295/-0.302 ->
-0.034/-0.037/-0.002 m and `walk_why` flips `over_board` -> `stalled`. Engine:
8/8 FALSE on seeds 1,5,6,21,22,25,32,38, including the three that pass at the
pin. Side correlate on both arms: walk-start base y <= -0.038 m -> |dy| ~0.30 m;
>= -0.023 m -> |dy| <= 0.03 m (one counterexample, s38). Record:
`work/tracks/07_box_table_to_shelf/actor_6/DEADEND_latch_rows.md`.

NOT catalogued for 08 (actor_2, 2026-11-11): `cand_guard24` (b482049b...1a3d)
latches the creep guard's first drill-drift crossing and ignores it only before
creep index 24. It is a real trajectory change (14 seeds bit-identical to the
pin, 7 identical to a relaxed control after stripping one log row) but NOT a
repair of the signature it claims: at the engine level 3, 16 and 17 still fail,
the same three the pin fails, so the creep depth and the end-state manifold the
actor measured are necessary but not sufficient. Over 0-40 it is 37/41 vs the
pin's 36/41: it makes seed 22 reach the outcome and loses nothing, its only
engine-level effect. The pin's 21-40 map, measured at the same time: 18/20,
FALSE 22, 36. Gate B PASSES at these bytes (verifier_gateB_2/GATE_B_EVIDENCE.md):
ownership PASS, 43/43 actuators within forcerange (n_over 0 on every seed/run;
the only at-cap joint is left thumb_1 at 1.4000 Nm in both policies, candidate
at-cap 240-295 steps vs control 70-326), determinism PASS (75/75 trace leaves
and identical logs across two reruns), no API read added or removed. It is not
catalogued because gate A fails on the seeds it targets.

NOT catalogued for 07 (actor_6, 2026-11-11, second round): the pre-walk-y and
x-clearance actuators are ALL refuted on the 8-seed arm (1,5,6,21,22,25,32,38).
y dose -0.045 (`pp35_a6` 5bb071fd..., gate hit 8/8) passes only s1 and loses
s5/s6; y dose -0.080 passes only s25; wz=-0.7 at the PINNED bytes drops walk-start
base x into 0.731-0.766 (up to 5 cm below the pin's movers, e.g. s21 0.784 ->
0.762, s25 0.796 -> 0.765) with heading repaired to -86.7..-90.7 deg, and still
stalls 8/8 (first-burst dy -0.002..+0.001 m, walk_why 'stalled'). So walk-start
x-clearance is not the gate. Measured law worth keeping: the in-place turn adds
base x at r = 0.00203 +/- 0.00017 m per degree of yaw swept at wz=-1.0 (0.00183 at
wz=-0.7), i.e. it orbits a vertical axis ~0.117 m in front of the base centre;
S0x = retreat_x + r*dyaw predicts all 8 pin seeds (turn gain +0.146..+0.184 m).
The retreat saturates after 3 rows (pre_x=0.50 is unreachable). Record:
`work/tracks/07_box_table_to_shelf/actor_6/DEADEND_walk_gate.md`.

NOT catalogued for 03 (actor_2, 2026-11-11): `rz1` (a5f09d09...1477) adds a
body-frame pad correction after close. It repairs four of the seven unreached
seeds (21, 24, 30, 40 reach the end state) but BREAKS seed 17, which the pin
passes (0.9649 -> 0.8030 m), so it violates the keep-passing rule and is not
shippable. `rz2` (357b78d7...617b), 12 iterations, is 0/8: it converges the pad
to <= 4.5 mm on 21/24/30/40 and those then fail at the first lift (nlift 1),
and it does not improve 17/29/38 at all. Mechanism worth keeping: with a face
half-length of 0.1169 m, a pad landing > 0.11 m along the face bears on the box
CORNER, after which both pinch and along-face commands are unachieved (20-50 mm/
iteration commanded, <= 1.5 mm achieved) -- no gain, cap or iteration budget can
close the residual 20-31 mm. Separator at close: left-pad along-face coordinate
0.083-0.105 m on the four repairable seeds vs 0.129-0.161 m on 17/29/38.
Record: `work/tracks/03_box_lift/actor_2/DEADEND_body_frame_pads.md`.

NOT catalogued for 05 (actor_8, 2026-11-11, second round): open-loop
resegmentation of the branch ramp (`br_seg` 8/10/16/20/30/40, total held at
3.0 s, `br_seg_dur` 0.375..0.075) is REFUTED as a fix. Over 61 seeds the ship is
56/61 and seg40 is 52/61: seg40 removes the displacement on s26 only (55.2 -> 5.4
mm, z 0.838 -> 0.934) and CREATES it on five ship-TRUE seeds (11, 16, 19, 47,
60; e.g. s11 5.4 -> 49.6 mm). The 12-seed x 6-level bracket is non-monotone and
seed-idiosyncratic (s26 55.2 at every level below 40, 5.4 at 40; s41 14.9/8.2/
30.1/46.3/6.3/7.9), and the segment traces show the drill first moves while the
pad is 232-331 mm away on every ramp seed at both doses -- so the early motion is
the left arm's own transient, not pad contact, and the topple is a discrete
per-seed outcome that resegmentation re-rolls. Record:
`work/tracks/05_drill_lift/actor_8/DEADEND_ramp_resegmentation.md`.
Measured correlate worth keeping: lift-end drill z > 0.91 m reproduces the engine
partition on all 61 seeds 0-60 at the ship bytes (actor's own 61-seed run,
56/61), so it is a valid end-state proxy (not a pre-command gate).

NOT catalogued for 06 (actor_4, 2026-11-11): `diag_v20` (7f8e072f...18e9), a
pre-close screen plus a y-only palm nudge when the static grasp touches < 4
surfaces, is a measured NON-REPAIR at the engine level: over seeds 10-30 it is
18/21, the SAME three FALSE as the pin (16, 18, 28), despite its proxy showing
s18 passing (t 22.58 s, z_up 0.8942, 4 contacts vs 2 before). So the static
contact count and lift_lost are necessary but not sufficient. Caveat kept: the
scratch screen is read-only in state but NOT behaviour-neutral -- zero-command
seeds shift up to 81.8 mm deterministically (library/LESSONS.md). The actor's
perturbation control (screen, no command) is the right instrument and showed the
nudge, not the screen, changed s18's trajectory.

NOT catalogued for 09 (actor_6, 2026-11-11): `cand_v61_regrasp` (6bc548d8...6084,
85353 B) adds a roll re-grasp after a final-hold check. The DETECTOR is sound:
grip empty (0 contacts) and z < 1.0 m at the final hold separates all 41 tested
seeds 0-40 (TRUE hold z 0.929-1.048 with 2-7 contacts; FALSE 0.838-0.843 with
0), fires on exactly {23,25,40}, and is bit-identical to v52 on all 38 passing
seeds (all result keys and t). The RECOVERY is refuted at the engine level:
coordinator run on 23/25/40 gives engine TRUE 0/3. Mechanism of the failure, as
measured by the actor: after two nudges the shoulder-target distance is
0.372/0.389/0.383 m against reach_max 0.32, the base-target lateral offset is
-0.032/-0.179/-0.217 m against a +x-only burst axis, and the palm ends
0.206/0.170 m short. So the empty-grip detector is a valid post-hold proxy
(41/41) but the retry's reach is outside the skill's own reach_max envelope.
Record: `work/tracks/09_obstacle_aware_pick_drill/actor_6/` (v61 files);
coordinator bits `work/coordinator/verify/v61regrasp_cand_v61_regrasp.py.json`.

NOT catalogued for 03 (actor_2, 2026-11-11, frame question): `rz8d`
(3cbcf0f0...e7f6) is a DIAGNOSTIC INSTRUMENT, not a skill: rz7e plus per-step
`ik_err`/`palm_residual` logging, issuing six 40 mm single-hand steps on a clear
pad and reporting, per step, the engine residual and the achieved displacement.
It guarantees no end state (aborts `abort='diag'`). Its result settles the frame
question: the ~90 deg along-face direction error is NOT an artifact of the
actor's body-frame construction. The engine's own per-side `ik_err` on the
commanded side is 41.3-72.3 mm right and 39.6-84.2 mm left (never near zero),
and the angle between commanded and achieved world displacement is 19.9-22.6
deg (R_out), 21.2-30.9 (R_up) and 86.0-87.6 (R_back) at dur 1.5 s, 19.3-23.0 /
22.3-37.2 / 79.6-87.9 at dur 4.0 s -- an angle is rotation-invariant, so the miss
is real in world coordinates. Mechanism: `ik_err` tracks |palm_residual| (0.0693
vs 0.0691) and never falls below ~40 mm, the same 60-74 mm residual the promoted
pre-grasp reach already sits at (library `reach_opposed_faces`: 0.0609/0.0735
under a 0.09 guard), i.e. the solver's operating point, not an exceptional wall.
OPEN (the decision the actor could not make): whether the along-face axis is
kinematically unreachable at this R_body or every 40 mm step is merely below the
~40-70 mm solver resolution floor. Deciding signal: achieved along-face
displacement (mm, body frame, per step) for single-hand steps of 10, 20, 40 and
80 mm at fixed R_body on 17/29/38; slope ~0 over the four magnitudes means
unreachable, slope clearly >0 with a roughly fixed offset means a resolution
floor. Record: `work/tracks/03_box_lift/actor_2/DEADEND_clear_pad_arm.md`.

NOT catalogued for 01 (actor_8, 2026-11-11): `..._press_a8b` (86484e2f...a679) is
a reason-gated attitude retry -- it suppresses a down-tilt guard fire (no
retract + re-approach) while still retrying on a roll-away. Measured over the
full 0-80 range (81/81 episodes ok, no engine error): end-z failures s28 0.5546,
s32 0.5546, s60 0.2764, s61 0.2764 -> 77/81 against the pin's 76/81. It gains
s66 (0.5546 -> 0.8860) and s74 (0.2764 -> 0.8798) and improves s33 (0.8471 ->
0.8655) but BREAKS s60 (pin 0.8607 -> 0.2764), a keep-passing violation, so it
is not shippable. The load-bearing measurement: the guard's own `dz` (m,
world-frame pad normal-z change over the approach) separates the one
load-bearing down-tilt fire (-0.088, s60) from the three safe ones (-0.012 s33,
-0.030 s66, -0.010 s74); the reason label does not. All four observed fires
separate at a threshold in (0.030, 0.088) -- four samples, so any dz gate is
fitted on a gap, not a distribution. Successor a8c is being built on dz; it must
keep s60 and its scope can be nothing wider than 0-80 until seeds above 80 are
run. Record: `work/tracks/01_box_lift_floor/actor_8/`.

NOT catalogued for 09 (actor_6, 2026-11-11): `cand_v62_probe` (eefe5e0c...04c4,
90066 B, 1779 lines) is a MEASUREMENT variant, not a solution; it lifts nothing.
Four results. (1) The whole catalogue of 09 failures is one class: 23/25/40 and
the held-out 41/49/59 all show fire=1, n0=0, hold z 0.838-0.845 m, tilt
73.0-102.1 deg, z1=z0, 0 engine hand-object contacts, whereas the 38 TRUE seeds
hold at 0.929-1.048 m with 2-7 contacts. (2) The +x-only nudge budget is
EXHAUSTED, not merely small: measured base gains are 0.032/0.021/-0.019 m
(seed 23), 0.018/0.000/0.000 (25), 0.0005/-0.0005/-0.0003 (40) against a
verified 0.031-0.033 m per burst, so 25/40 stop responding after the first call
(0.0003-0.0010 m per call). (3) Palm-orientation transport with the body is
expressible (one `reach_box(..., orientation='keep', R_body=X_now X_start^T,
iters=300)` call) and executes, but lifts nothing: on 23/25 the commanded pose
is not IK-solvable at the post-nudge base (ik_err 0.1551/0.1074 m), and on 40
the pose IS solvable (ik_err 0.0022 m, live distance 0.2956 m) yet the
whole-body controller leaves a 0.163 m palm residual -- a tracking limit, not a
reach limit. Contacts appear on 40 and 25; a grip does not (object z unchanged
through three 0.04 m lifts). (4) Barrier: all 41 seeds re-run at the v62 bytes,
and the 38 non-firing seeds are bit-identical to v61 in `t` and every result
key; the only new key is `X0`, on the three firing seeds. OPEN: whether the
collapsed base response is a balance refusal, a physical obstruction, or the
skill's own creep guard; deciding signal = commanded `move_base` displacement
vs achieved `base_state()` displacement per call plus base-foot contact pairs,
with base yaw. Record: `work/tracks/09_obstacle_aware_pick_drill/actor_6/
DEADEND_a6_v62_probe.md`.

NOT catalogued for 03 (actor_2, 2026-11-11, magnitude sweep): the along-face
axis is NOT actuable at the grasp R_body -- it is a boundary, not a resolution
floor. Instrument `rz9d` (46df0c97...f173), the rz8d steps at 10/20/40/80 mm,
seeds 17/29/38, dur 1.5 s, right pad clear, each step from the previous achieved
pose. Achieved along-face displacement (mm, body frame, per step) is 17:
-7.6/-6.4/-4.2/-2.1, 29: -8.4/-6.2/-4.9/-2.7, 38: -12.2/-8.1/+1.7/-4.4,
while the ZERO-COMMAND control on the same steps is -10.0/-12.0/-15.5 mm --
larger in magnitude than every commanded response. Pooled slope 0.084 mm/mm
(n=12, intercept -8.6 mm); control-subtracted per-step displacement peaks at
~10 mm, inside the +-10-15 mm drift scatter. Mechanism: the solver never
realizes the target -- per-side `ik_err` is 72.3-76.8 mm at 0 mm and rises ~1:1
to 161.0-161.6 mm at 80 mm, tracking |palm_residual| (79.0 -> 162.3) -- and the
motion that does occur is a command-independent bias in -ux (-12.9 to -44.1
mm/step, pinch axis) and +world z (+4 to +16 mm/step, palm z 0.915 -> 0.981 m
over five steps). Contact is excluded: pad pinch clearance is positive and
growing (27.9->120.0 / 28.9->138.3 / 34.1->164.5 mm) and box drag <= 9.2 mm
except 17.7 mm on s38's 40 mm step. Control in the opposite direction (rz8d
+uy 40 mm) is -0.9/-0.6/-2.2 mm with ik_err 43.1/42.6/41.3 mm, so the effect is
direction-specific. OPEN (and the last cheap route): whether the boundary is
intrinsic to the arm at this arm pose or specific to this R_body; deciding
signal = achieved/commanded (mm/mm, 40 mm step, zero-command drift subtracted)
at ~3 R_body yaw offsets around the current grasp, same seeds; any offset at
>= 0.5 mm/mm with ik_err < 90 mm reopens it, all remaining ~0 closes it for this
arm pose. Record: `work/tracks/03_box_lift/actor_2/DEADEND_along_face.md`.

NOT catalogued for 05 (actor_8, 2026-11-11, third round): approach-waypoint and
settle-gate are REFUTED, and the failure signature is localised to the branch
ramp. Mechanism, measured on six seeds at two ramp doses (0.30 s x 10 and
0.075 s x 40, ship knobs): the left arm is ALREADY in or near contact with the
drill when the ramp starts -- literal hand-drill contacts on s26 (palm overlap),
s47 (index_0, seg0), s16 (index_1), s60 (index_0), and <= 23.5 mm tight
otherwise -- so the ramp drags an already-engaged object and an approach
waypoint cannot help (the arm is already inside the corridor). A dry control at
identical wall time still moves the object 4.7 mm (s11), 9.1 mm (s26) and 4.0 mm
(s60) before the ramp and 0.0 mm on s16/s19/s47, so part of the first-motion
trigger is pre-ramp settling. The settle-gate candidate (`a8_c1.py` 10eb4da3...)
added a 0.10 s pause: its probe read moved = 0.0 mm on all six, the gate never
extended, and s16 fell 0.923 -> 0.838 m from the pause alone (others within
+-4 mm, s26 stayed 0.838). A 3.0 s hold control swaps rather than gains
(s16 0.838, s26 0.926), which the keep-passing rule forbids. Instrument
`a8_tr4.py` 8648998d... (110813 B); `br_dry`/`br_trace`/`br_settle` are inert at
0. OPEN: whether the pre-ramp tip is created by the ramp or already present at
its first step; deciding signal = drill tilt from world +z at the ramp-start
instant (collider AABB vertical/horizontal extent ratio), failing vs passing
seeds, threshold 5 deg -- > 5 deg on the failures means no ramp-side reaction
can distinguish them and the fix must be upstream hand placement; all < 2 deg
makes an abort-on-first-tilt ramp (same segment count, no added sim time) the
one admissible candidate. SCOPE GAP noted by the coordinator: the six-seed set
is {11,16,19,26,47,60}, so four of the five target failures (20, 32, 45, 46)
are NOT characterised; the same ramp-start measurement must be repeated on them
before any class claim. Records: `DEADEND_approach_waypoint.md`,
`DEADEND_settle_gate.md`.

NOT catalogued for 07 (actor_6, 2026-11-11): `pp37_a6.py`
(f77a6671...0376, 57312 B) is a y-only gait-entry retry -- after the first walk
burst, if the base did not translate in y, issue a zero-nav `step_goal` hold at
`walk_h` for `gait_retry` s, then continue. On the 8-seed failure set it is 4/8
against the pin's 3/8 and rescues s22 with no loss; on a wider 12-seed set
(0,2,3,4,7,8,9,10,11,12,20,23) it is 8/12 against 9/12 and LOSES s4, a control
passer (gap +0.0002 -> -0.0398, box left on the table, 2 bursts, `walk_why`
stalled). Mechanism: the gate tests base y alone, and s4's burst-1 displacement
is dx +37.0 mm / dy -3.0 mm, so a 37 mm step was read as "did not move"; the
0.6 s zero-nav hold then drifted the base y from -0.094 to -0.078 and slid the
box 20 mm backwards relative to the gripper (obj x 0.794 -> 0.774). The
pre-existing stall counter makes the same y-only assumption, which is why s4
reports `stalled`. Refuted over 20 distinct seeds. The direction-agnostic
successor `pp38_a6.py` gates on displacement magnitude |b-b0| with a 0.020 m
threshold, which separates the four jammed seeds (2.8/12.4/13.6/14.8 mm) from
every control passer (minimum 20.4 mm, s20) on the 20 measured seeds -- but the
margin to the nearest passer is 0.4 mm, so the threshold is fitted on a gap, not
a distribution, and it needs the wider seed set before it can be believed.
Records: `DEADEND_gait_retry_yonly.md`.

05 class partition (actor_8, 2026-11-11, coordinator-requested coverage of the
target failures). The five target seeds are NOT one class:
- s20: pre-ramp topple. Drill long-axis vertical extent 31.2 mm against 163.6 mm
  upright, i.e. ~90 deg, ALREADY LYING at ramp start; seg0 pad-drill gap
  155.6 mm, object displacement 0.0 mm across all 10 segments; end z 0.838 m.
- s46: pre-ramp topple during the close. Close fire 0 (mag2), pre-ramp pose
  check rejected (pr_out 0, deviation 113 mm); z 0.801 m at t 11.4 -> 0.816 at
  11.9 -> 0.838 at 12.4; the ramp never executes.
- s45: grasp miss, no ramp. Close fire 0 (fallback mag2, thumb2_y -0.0192
  against the -0.055 threshold), ncon 0, object never moves, end z 0.799 m.
- s32: flip class. seg0 pad-drill gap 20.4 mm (flip set 0.0-23.5 mm), first
  >1 mm motion at segment 5 with 3.0 mm displacement and tight gap 0.0 mm,
  drill z 0.799 -> 0.822, end 0.838.
- s26: flip class (palm overlap, first row 1.0 mm / tight 0.0 mm).
So a ramp-side reaction can separate at most s26 and s32 and tops out at 58/61.
Tilt at ramp start measured from the drill long box (upright 163.6 mm, >= 5 deg
needs z_ext >= 168.3 mm): s11/s16/s19/s47 163.6 (0.0 deg), s26 165.1 (~1.5 deg),
s60 164.0 (~0.3 deg) at seg40 -- all < 5 deg, so the failures are UPRIGHT at
ramp start and tip during the ramp. An abort-on-first-tilt rule is NOT
admissible: s26 (165.1) and s60 (164.0) are 1.1 mm apart, inside the AABB
resolution, so any threshold that catches s26 also catches a passer. OPEN: the
pre-ramp instant (first pad-drill contact of the seat phase) for s20/s46;
deciding rule = upright (< 5 deg) and undisplaced (< 1 mm) at first contact with
tilt appearing afterwards means the seating pose causes the topple and a
pre-ramp pose change is admissible; already tilted/displaced means the state is
set before the hand acts. Record: `work/tracks/05_drill_lift/task_memory.md`.

NOT catalogued for 03 (actor_2, 2026-11-11, R_body yaw sweep): the along-face
axis stays unactuable at every wrist yaw around the grasp, so the boundary is
intrinsic to the arm pose, not to this R_body. Instrument `rz10d` (fba443f9...d197),
the rz9d protocol at four wrist offsets (-24/-12/0/+12 deg), seeds 17/29/38, 40
mm single-hand step, zero-command control at every offset. Control-subtracted
achieved/commanded along-face ratio: -0.196/-0.261/-0.321 at -24, -0.100/-0.121/
-0.184 at -12, -0.002/-0.025/-0.029 at 0, +0.013/-0.003/-0.056 at +12; max
magnitude 0.321 and NEGATIVE at or beyond the grasp yaw, i.e. the commanded step
ended LESS far along the command than its control (anti-actuation). ik_err at
the 40 mm step is 108.5-119.4 mm at every offset, above the 90 mm guard, so the
targets are outside the arm's reachable set; wrist yaw only chooses which
non-along-face channel absorbs the command (palm rise +14..+17 mm at 0/+12, a
world-y drift reversal +14..+22 mm at -24). The -24 row is confounded (left-hand
6.1-17.5 mm, box drag 12.4-19.7 mm) and the decision rests on -12/0/+12. No pad
contact at any offset (pinch clearance positive and growing). OPEN, bounded:
whether an ARM-pose change (base yaw or approach point) rather than a wrist yaw
reaches the along-face axis; deciding signal = same zero-command-subtracted
protocol after the pose change on 17/29/38 (cheap proxy) or 21/24/30/40;
>= 0.5 mm/mm with ik_err < 90 mm reopens it, none closes the along-face route
end-to-end. Record: `work/tracks/03_box_lift/actor_2/DEADEND_rbody_yaw.md`.

05 pre-ramp admissibility (actor_8, 2026-11-11, two authorised episodes;
instrument `a8_pre.py` bf36385b... = pinned a8_fine.py 3fd440a5... + one
read-only knob `br_pre`, reads only, no step/skill). Geometry: drill long box
upright (60.8, 31.6, 163.6) mm, landed (63.7, 164.7, 31.2) = a quarter turn about
world x; tilt read from e_y (e_z never inverted).
- s20: prestance t 10.74 tilt 0.0 deg, dxy 0.0 mm, pad gap 105.1 mm; the FIRST
  pad-drill contact of the seat phase NEVER OCCURS (minimum gap 29.3 mm at
  t 11.54, no hand-drill pair at any tag); at ramp start t 11.54 tilt is 14.4 deg
  about x with dxy 1.0 mm and pad 29.3 mm clear; end lying 0.838, dxy 38.1 mm.
  So the tilt appears between prestance and pinch with the pad 29 mm away --
  the state is set before the hand acts and NO pre-ramp pose change is
  admissible.
- s46: prestance t 10.60 tilt ~0.8 deg, pad 127.2 mm; first pad-drill contact
  t 11.40 with pad gap 0.0 mm, tilt 2.4-3.2 deg, dxy 1.0 mm; no ramp executes
  (gated_out 1, pr_out 0) so t 11.40 is the decision instant; grip t 11.90
  26.0 deg, lift0 t 12.40 lying 0.838, dxy 44.2 mm. The topple completes AFTER
  contact during the close, so by the stated rule the seating/close action is the
  driver and a pre-close hand-pose change is admissible.
- s45 remains a close-phase boundary (fired 0, mag2, ncon 0, object never moves).
OPEN (one seed): whether s46's 2.4-3.2 deg at t 11.40 is the first-contact
response or pre-existed -- there is no sample between 10.60 and 11.40; deciding
signal = the contact-onset instant (first true gripper-drill contact pair) set
against a 10 ms tilt series in that window. seg40 reconciliation: the six-seed
instrument run differs from the refuted 61-seed run only by br_dry=1,
br_trace=1 and the instrument source, so the engine result (52/61 vs 56/61)
STANDS and seg40 is closed.

09 base-response verdict (actor_6, 2026-11-11; measurement only, no recovery).
`cand_v63_nudge.py` 20f83bb9... inlines `top_up_reach_at_standoff`'s loop at
`move_base` and adds read-only `base_state()`/`contact_pairs(limit=150)` at each
boundary; the fire gate and the non-firing path are byte-unchanged, and v62 on
the same seeds gives bit-identical per-call base displacement and nudge count.
Measured per executed `move_base` burst (nominal 0.100 m): achieved 0.0241 ->
0.0011 m (s25), 0.0086 -> 0.0003 m (s40), and 0.0113/0.0192 m BACKWARD on s23
call 3, while foot contact points move <= 0.2 mm as the base moves up to 38.8 mm.
Verdict: (c) the creep guard is EXCLUDED on all three (every executed burst
issued `move_base` at 0.100 m; the only breaks are target-reached exits on s23).
(b) PHYSICAL OBSTRUCTION on 25 and 40: `table_main|robot0_pelvis` forms in the
exact collapsing burst and stays world-pinned through every later burst (13
records each) while 0.4 m more is commanded, and the base creeps backward; the
pelvis meets the table edge at x 1.533/1.558 while the drill sits on that table.
(a) BALANCE/LEG REFUSAL on s23 call 3 only (forward command, net backward
achieved, zero robot-vs-external contact, yaw within 5 deg of +x). Under both:
the feet are planted every burst, so base travel is stance extension capped at
~0.03 m per burst and `move_base` has no stepping parameter. The remaining
lateral offsets (-0.16/-0.23 m on 25/40) were never in an axis=0 nudge's
controllable direction. This is a MOBILITY boundary, not a reach-tuning
problem. Undecided but NOT action-guiding: whether the table blocks the base or
the WBC yields to the contact (no normal force or signed distance is exposed by
`contact_pairs`); both branches say no +x nudge can work, so no force readout is
being added. Structural route under test: a lateral walk to the drill's y offset
(`walk_base_to_coordinate`, catalogued for 08 AND 09) instead of a +x stance
nudge. Record: `DEADEND_a6_v63_nudge.md`.

05 s46 CORRECTION (actor_8, 2026-11-11; no new episode, the authorised episode
re-read at its own 20 ms resolution). First true gripper-drill contact pair is at
t 11.42 s; NONE exists in 10.60-11.40. But the drill tilt is NOT zero before
that: it is exactly 0.000 deg / 33.90 mm e_y from 10.56 to 11.28 (object static
at [1741.39, -24.17, 799.50] mm), then 0.643 deg at 11.30 with object z +0.42 mm
in the same step, rising 1.052/1.537/1.921/2.264/2.561 deg at 11.32/34/36/38/40,
all with hand_n = 0; contact onset 11.42 at 2.497 deg. So the tilt begins 0.1 s
before pinch completion and 0.2 s before the first close command, during the
pre-close approach pinch, and the object moves in the same step the tilt starts.
The binary rule I set ("tilt before contact = pre-ramp boundary; after = close
is the driver") is too coarse and has no middle branch: the physical driver is a
BRUSH during the pre-close approach, which is neither a state set before the
hand acts nor the finger close. Consequence: the admissible fix for s46 is a
PRE-CLOSE APPROACH POSE change (do not let the pad brush the drill on the way
in), not a ramp-side reaction. Capability boundary recorded in DEBUG_TOOLS.md:
the recorder exposes one post-step sample per control step (20 ms) and hand_n is
an instantaneous post-step count, blind to contact confined inside a step's four
5 ms substeps; 10 ms sampling does not exist and splitting control steps would
change the episode. Missing signal: per-substep contact across 11.28->11.30.
Record: `s46_tilt_series.csv`, `log_pre46.txt`, `a8_pre.py` bf36385b....

NOT catalogued for 03 (actor_2, 2026-11-11, arm-pose block): no admissible
arm pose that the CURRENT WALKER can reach and hold actuates the along-face
axis. Instrument `rz11d` (757661f8...2b0a) adds base_yaw/base_along stance
knobs to the rz9d protocol, with the same-pose zero-command control in every
block. Seeds 17/29/38, ratio mm/mm at ik40: A base_yaw -15 deg gives
0.100@64.4 / 1.850@0.4 / 0.620@106.1; B +15 deg 0.033@87.1 / 0.974@81.7 /
0.120@81.3; C base_along -0.050 gives -0.001@114.8 / 0.096@116.4 / -3.825@96.4;
D no-change control -0.002@117.4 / -0.025@116.3 / -0.029@114.2. The rule hits are
all non-actuation: rejected frame (A/29 phi 160.6 deg), pad 245-597 mm off the
nominal approach site, pad 230-325 mm past the face plane, or 24-59 mm box drag.
The ONLY admissible pose (C/29: frame ok, clearance > 0, drag <= 2 mm) is
0.096 mm/mm at ik40 116.4 mm -- below the 0.5 threshold and above the 90 mm
reach guard. CRITICAL CAVEAT, logged not inferred: the commanded stances were
not realised. The +-15 deg walks land 0.15-0.22 m off in world y, every walk
ends on `fingertip_guard`, the yaw is not held through the pipeline (A/17
-17.48 -> -14.34 -> +0.07 -> +6.05 with no logged move_base), and the -50 mm
target lies inside the guard so it stops 34-49 mm short; boxes end spun
(yaw 4.8/6.2/10.9 deg baseline vs -40.2/160.6/14.2 etc.). So the honest claim is
"no stance the unchanged walker can REACH AND HOLD actuates it", not "no arm
pose whatsoever". OPEN, and the last lever: the ratio at a stance whose
realisation is verified in the same episode (base yaw within 2 deg of command at
the diag, standoff within 5 mm, approach within 20 mm of B-30, clearance > 0 at
both steps, drag <= 2 mm) with ik40 < 90 mm. Reaching it needs the instrument's
walker fixed (lateral drift, standoff outside the guard, and the unpatched
`retract_and_walk` at rz11d.py:588 defaulting `yaw_tgt` to 0). Record:
`DEADEND_arm_pose.md`.

NOT catalogued for 06 (actor_4, 2026-11-11): `diag_v31` (05d032c4...e108,
22766 B), a single global change of the pre-close standoff z from 0.86 to
0.854 m (-6 mm), is a KEEP-PASSING VIOLATION at the engine. Coordinator bits over
10-30: TRUE 16/21, FALSE 16, 18, 23, 26, 28 (the pin is 18/21, FALSE 16, 18, 28). s23 and s26 are
TRUE at the pin (455a237a...), so the candidate loses two passing seeds while
repairing none of the three it targeted -- s18 stays FALSE. Its close-phase proxy
said the opposite: on s18 dz 35.8 -> 28.1 mm, post-close z_up 0.8752 -> 0.9276,
hand contacts 2 -> 4, lift_lost True -> False, monotone over the -6..+3 mm grid,
and every previously passing seed still read lift_lost False. So for 06 the
close-phase quantities (dz across the close, contact count, lift_lost, post-close
z_up) are refuted as predictors of the engine outcome in BOTH directions: v20
showed a repaired proxy with an unchanged bit, and v31 shows a "non-loss" proxy
with two lost seeds. The only remaining admissible signal is end-state: the
final object pose error (position mm, orientation deg, world frame) of a
candidate seed against a known-passing seed at the same bytes, which no
close-phase measurement captures. Records: `SHIP_v31_insertion_depth.md`,
`resp_v31_full.json`.

06 end-state and stance measurements (actor_4, 2026-11-11; no candidate). Under
the SHIP bytes s18 terminates AT THE PICK SITE on the first lift row (t 6.00 s,
z 0.7995 -> 0.8353 -> 0.8571 then lost, 2 end contacts, table not reached) and
never carries, so it has no destination pose; its 790.0/867.0 mm x/y offset from
s23 is early termination, not placement error. Its end orientation is INSIDE the
passing envelope: across the 18 passing seeds the final tilt spans 0.00-73.01 deg
(Task-TRUE s10 ends at z_up 0.4660 = 62.23 deg with 1 contact and 1 table, s26 at
z_up 0.2922 = 73.01 deg with 0 contacts and 3 table), so end orientation is not
monotone with the bit. Under v31 s18 does carry (10 rows, hand 5 -> 2, z
0.8745..0.9700) and ends at (0.0197, 0.8918, 0.8792), z_up 0.0344, outside the
16-seed v31-TRUE envelope by 90.0 mm in x and 79.7 mm in z. Base stance: NO stance
quantity separates 16/28 from the passing pair -- at p_pre base x spans 0.5 mm,
base y 3.4 mm, yaw 0.11 deg and right-shoulder x/y/z within 3 mm across all four
seeds; the separating quantity is the SPAWN (object y 53.2/59.4 mm on s16/s28 vs
18.0/15.1 on s23/s20), a right-shoulder-to-object lateral offset of 145.8/150.9
vs 109.0/113.0 mm. That is object spawn and cannot be tuned per seed, so 16/28 is
a REACH BOUNDARY. STRANGE PAIR to resolve: s26 ends at 73.01 deg tilt and is TRUE
at the pin, and ends at the SAME 73.01 deg under v31 and is FALSE, so identical
end position/orientation with different bits; the final probe does not record
final YAW, the one coordinate that could differ. Records:
`DEADEND_endstate_envelope.md`.

09 lateral-walk measurement (actor_6, 2026-11-11; `cand_v64_latwalk.py`
4e283730...8bf5c, measurement only, s25/s40 one episode each). The pre-agreed
build branch was NOT met, so no candidate: the feet NEVER STEP. Both ankle-roll
links keep a `gear_factory` contact pair at all 41 samples (lift 0, land 0, air 0,
n = 41) and move <= 1.3 mm total, while 0.72 m of lateral command (40 sub-bursts
at 0.18 m/s, x commanded 0.0) produces only -0.0497 / -0.0309 m in y and +0.0145 /
+0.0080 m in x (6.9 / 4.3 % of command) before a hard stop after sub-burst 8 (net
y over bursts 9-40 -0.0002 / +0.0004 m). `table_main|robot0_pelvis` never forms on
s25 (0 of 41) yet s25 still freezes at 4.97 cm laterally, and on s40 it appears
only after sub-burst 9 when the response is already <= 0.2 mm. So the STANCE, not
the table, is the ceiling: the base can shift 3-5 cm inside planted double support
and cannot step in any direction. Reach goes 0.4066 -> 0.3660 m (s25) and 0.3833
-> 0.3552 m (s40), both still > 0.32 m, with lateral offset 0.1186 / 0.1971 m.
`move_base`-reported displacement agrees with the `base_state` delta to <= 0.5 mm,
so the base stops moving, not the reporting; base yaw drift <= 3.3 deg and tilt
FALLS (2.88 -> 1.57, 5.64 -> 2.42 deg), i.e. settling, not straining. Layer limit
recorded in the skill: the catalogued `walk_base_to_coordinate` cannot express this
walk (its loop breaks on pos[axis] >= target and every burst issues forward x = v,
so a decreasing/zero-x lateral target is outside its parameterisation). Undecided
and NOT proposed as a harness change: whether the refusal to step is the WBC
choosing double support at these contacts or the command channel itself --
`move_base` exposes x, y, yaw, dur only, with no gait/step-height/step-frequency
request; both readings end this task the same way. Record:
`DEADEND_a6_v64_latwalk.md`.

06 final-yaw probe (actor_4, 2026-11-11; reader-only instruments
diag_v33 a3f14778... = SHIP bytes and diag_v34 d003450b... = v31 bytes, two
hunks each, no new geom call/step/command; all four finals reproduce the
pre-existing reference responses exactly). Final object yaw (ZYX, world) differs
by 9.564 deg on s26 (79.765 vs 70.201) and 7.282 deg on s23 (71.945 vs 64.663)
against a 0.001 deg log resolution. BUT the premise that only yaw differed is
wrong and the actor corrected it: final POSITION also differs (s26 10.3 mm in x
and 61.2 mm in y), and the split is not a carry transient -- carry-end yaw
differs by only 0.09 deg (s23) / 2.24 deg (s26) and the max single-step yaw is
13.46-13.86 deg in all four (commanded base slew), so the 7-10 deg accumulates
AFTER the carry at different landing points. The four episodes separate (y, yaw)
together, not yaw alone. Deciding whether yaw or landing y is causal needs a run
that holds one fixed while varying the other; that behaviour does not exist in
this composition. 06 is closed for this round: 16/28 are a reach boundary (the
separator is spawn, not stance), s18 terminates at the pick site on the first
lift row and its end orientation sits INSIDE the passing envelope, and the
pin/v31 flips co-vary in (y, yaw). Record: `DEADEND_end_yaw.md`.

07 proxy refutation (actor_6, 2026-11-11, reconciliation). The actor's PASS proxy
ANDed four conditions (front, back, x margin >= 0 and board_gap >= 0) and called
8 engine-TRUE seeds FAIL. The numbers were not wrong -- its recomputation
reproduces the engine's own verdict fields to 0.9/1.1/0.7 mm (front/back/x) and
board_gap to < 0.2 mm on 35 of 36 seeds -- the DECISION BOUNDARY was wrong. The 8
seeds it rejected breach exactly the coordinates the engine does not gate:
x_margin negative on 7 (s9 -34.5, s11 -10.8, s12 -15.7, s16 -11.2, s17 -4.6, s23
-12.6, s34 -23.9 mm) and front_margin negative on 4 (s11 -102.6, s12 -104.9,
s16 -112.3, s31 -11.0 mm), while all 8 sit at board_gap +0.2 mm with up 1.0. The
5 engine-FALSE seeds (21, 22, 25, 32, 38) are the only 5 of 36 whose board_gap is
-55.8 mm (obj z 0.685 = table top). So the observed engine discriminator is
BOARD_GAP (box actually on the board), not the margins -- consistent with the
skill's existing CAUTION "never gate on fm/xm/bm". Error size up to 112.3 mm of
front margin treated as disqualifying, always in the strict direction. The
actor's earlier claim that its proxy "agrees with the engine verdict exactly" was
circular (the same rule applied to engine numbers) and is withdrawn. Consequence:
the candidate's effect is pin 31/36 -> 32/36 at engine truth (the single gain
s22, board_gap -55.8 -> +0.2 mm), with zero lost resting on state identity on the
32 unchanged seeds, not on the proxy. Record: `SHIP_gait_retry_mag.md` correction.

03 along-face CLOSED (actor_2, 2026-11-11, realised-stance block). Instrument
`rz12d` (43fb701c...), built from the unchanged rz11d by `patch_rz12.py`
(16a2542c...), instrument-only: promoted.py 90f25b72... never run and unchanged,
proxy seeds 17/29/38 only, no passing 20-40 seed touched. The walker confound is
removed where it can be: one phased controller (rotate while |eyaw| > 6 deg,
translate with yaw=0 and the world intent rotated into the body frame, re-rotate;
exit 5 mm / 10 mm / 1.0 deg) puts the base within 0.01-0.78 deg of the commanded
yaw and 2.3-9.7 mm in x at the diag. At that REALISED yaw-0 stance the
control-subtracted along-face ratio is -0.075 / +0.069 / +0.032 mm/mm at ik40
118.5/119.1/116.1 -- the same null as the accepted baseline -0.025/-0.029/-0.002
at ik40 116.3/114.2/117.4. So the boundary does not depend on an unrealised
stance. At +15 deg the yaw still misses by 3.84-4.08 deg and drags the box
9.2-34.7 mm; at -15 the stance is not realised (seed 17 0.446/0.769 m off, 23.75
deg yaw error; seed 29's diag frame fails to solve). The approach gate is NOT met
at any seed (pad approach 118.4-139.0 mm against the B-30 site 86.9 mm) and it
fails identically in the accepted baseline (154.7-160.0 mm), so that is the
instrument's correction loop, not the walker. Under the fixed rule no realised
stance reached >= 0.5 mm/mm at ik40 < 90 mm, so the along-face route does not
reopen. 03's seven 20-40 failures are a DOCUMENTED BOUNDARY: corner bearing is
the relevant pose, and the along-face direction is outside the right arm's
reachable set at every stance the walker can reach and hold (ik_err 108-119 mm
against a 90 mm guard, commanded step indistinguishable from zero-command drift).
Record: `DEADEND_walker_stance.md`.

NOT catalogued for 05 (actor_8, 2026-11-11): `a8_cand_z1426` (3ea15a09...10ef0,
95059 B), which raises the absolute pre-close pinch-approach height `z_pinch`
from 1.140 to 1.1426 m (+2.6 mm, x and y still object-relative), is a
KEEP-PASSING VIOLATION at the engine. Coordinator run over 0-60 at the ship
15-key `off` dict: TRUE 51/61, FALSE 3, 16, 17, 20, 26, 32, 38, 39, 41, 45. It
GAINS s46 (the target; its proxy 0.838 -> 0.922 m was real) and LOSES 3, 16, 17,
38, 39 and 41. Control: the pinned a8_fine.py 3fd440a5... over exactly those six
seeds at the same call shape is TRUE 6/6, so the losses are the candidate's. The
actor's test order covered only s46 + s11 + s60 and reported "neither control
changes class" -- true for those two, but it extrapolated that to the partition
without running it. MECHANISM TENSION worth keeping: the pre-close pinch height
is load-bearing in BOTH directions. At 11.30 s on s46 the pad still had 2.625 mm
of descent left and its lower edge sat exactly that far below the drill's top +y
corner, so +2.6 mm clears the brush and lets the pad-hull route branch run
(pr_out 0 -> 1); the same +2.6 mm shifts the approach enough on six other seeds
to break them. A seed-independent z_pinch therefore cannot fix s46 without
breaking passers, and the only remaining route is a clearance measurement at the
pre-close instant feeding an adaptive height (closed-loop), which is next-round
work. Ship status unchanged: 56/61, FALSE {20, 26, 32, 45, 46}. Records:
`SHIP_a8_z1426.md` (now needs a rejected banner), `resp_*` under actor_8.

07 pp38 final shape (coordinator, 2026-11-11): the shipped candidate is not the
0.020 m gate described above. `pp38_a6.py` bed3b297...868e1 is called with the
pin's 35 kwargs PLUS two keys the pin passes at all: `gait_retry` 0.6 and
`gait_retry_mag` 0.012 (`kw_retmag12.json`, 37 keys). At that shape and
`{}` otherwise unchanged the engine over seeds 0-40 is TRUE 37/41 (FALSE 21, 25,
32, 38) against the pin pp33_a5 0f3d0808...b9d at 36/41 (FALSE 21, 22, 25, 32,
38): exactly one gain (s22), zero loss, 0-19 unchanged at 20/20, 20-40 16/21 ->
17/21. Recorded as a MEASURED VARIANT, not a supersession, for two reasons the
measurement does not remove: (1) 12 mm is a fitted point: 4.4 mm above the largest jammed
seed's burst-1 motion (s32, 9.4 mm) and 4.2 mm below the smallest pin-passing
seed's (s27, 16.2 mm), i.e. a constant placed between two observations rather
than a threshold derived from a distribution; (2) the retry never fires on
most seeds, so the value is untested where it does not fire, and the one seed it
gains is a single observation. The remaining four failures are the jam class
the retry cannot reach (base does not translate at all, so re-walking 12 mm is
re-walking nothing).

07 pp38 held-out 41-60 (coordinator, 2026-11-11) -- the variant is NOT promoted.
Paired block at the same server and call shape: pin pp33_a5 0f3d0808... TRUE
15/20 (FALSE 48, 52, 53, 56, 58); pp38_a6 bed3b297... TRUE 15/20 with an
IDENTICAL set. The candidate was not inert, which is what makes the result
informative: `gait_retry` rows exist on exactly s48 and s56, the two held-out
seeds the pin also fails, and neither flips (s48 base at the retry
[0.782, 0.006, 0.652]; s56 [0.765, -0.014, 0.651]). So on 0 of 2 unseen seeds
where the 12 mm gate fires the retry repairs nothing, and s22's gain (7.6 mm
burst-1 motion) does not generalise to the 41-60 population. The extended pin
scope is now 0-40 36/41 + 41-60 15/20 = 51/61 (FALSE 21, 22, 25, 32, 38, 48, 52,
53, 56, 58); pp38 would be 52/61. Ship unchanged at `carry_and_seat_on_shelf`
pp33_a5; the variant is kept on record with both scopes so the next round does
not re-derive it.
