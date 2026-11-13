# Debug tools (coordinator-maintained)

Standing rule: at every actor hand-back, name the decision the actor could NOT
make from the feedback it had, and the ONE signal that would decide it
(quantity, unit, frame, and the comparison it changes). Implement that signal
only if it is missing, at the lowest layer that exposes it, and have the same
track re-run the SAME experiment. No dashboards, no dumps, no "all possibly
useful" fields. A shared tool needs the same gap on >=2 tracks or a general
quantity (contact geometry, IK residual, slip, torque, IK-branch state). Remove
a tool once its decision is settled. The success predicate and the reward are
NEVER exposed by anything here.

## Tools

| tool | question it answers | fields | cost | where used |
|---|---|---|---|---|
| `{"tool":"events"}` | which event ended or shaped this episode, and how bad was it | earliest 1-3 events (kind, t, residual, contact dump); maxima (tilt, base drop, slip, act jump, joint-limit margin, torque fraction, penetration, contact counts); counts by kind; trace path | reads the recorded episode; recording is +10% wall, <1 MB per ~1000 steps | all tracks |
| `{"tool":"window",kind,index,pad,fields}` | what exactly happened around that event | the event plus sampled arrays over [t-pad,t+pad]: t, q, qd, act, tau, obj, obj_quat, obj_lo/hi, base_z, tilt, hand_n, contact_n, nav, height | file read, no sim steps | 01, 05, 08 |
| `{"tool":"probe",what}` | what is the precise state right now: `contacts`, `object`, `joints`, `actuators`, `hands`, `geom`, `scene`, `bodies`, `state` | as named; `contacts` = all contacts with geom/body names, normal, distance, penetration, force | one to a few `mj_forward` passes; cheap compute but a PHYSICAL read -- it perturbs the episode | all tracks |
| `{"tool":"trace",fields,stride,t0,t1,format}` | give me the time series to analyse offline | chosen fields at `stride` over [`t0`,`t1`], written as npz/csv | file write; no sim steps | 05, 07 |
| `{"tool":"stills",kind,index,pad,n,lookat,mp4}` | what does the failing frame look like | PNGs / short mp4 rendered on a scratch MjData around the event | offscreen render, seconds | 03, 08 |
| `run_seeds(..., "obs": true)` | what is each seed's event summary in one batch | the event summary attached per seed | recording overhead only | all tracks |
| `api.contact_pairs(limit=32)` | which contacts involve the object RIGHT NOW, including child-body geoms | pairs of body/geom names with normal and distance; the truthful source (the root-body `hand_object_contacts`/`hand_contacts_side` read 0 for drill and some boxes) | one read inside the primitive -- physical, counted in the trace | 08 `move_held`; 07 diagnostic rows |
| `api.observe()` | base pose/tilt, object pose, hand-object contacts, t | as named | one read | all tracks |
| `api.geom_detail(tag)` | collider geometry of an object (AABB, faces) | per-geom size/pose; used to turn a pad offset into a face distance | one geom read | 01, 03 |
| `api.ik_preview(..., targets=, prefer_current=, iters=)` | BEFORE commanding: which multi-start branch will `reach_box` solve this palm target on, and how far will the arm/pad swing | per row: `seed` (0 current pose / 1 mirrored / 2 generic), `dq` (max abs joint travel, rad), `pad_travel` (m from live pad pos), `err`, `rot`, `pad`, `palm`, `idx1/mid1/th2` | 3 seeds x 1 target x iters IK solves on scratch data, no sim step | 01 branch-aware retry; grid mode used by 03, 05, 09 |
| `pad_hold_audit` (05, task-local probe) | is the pad "settled" or is a single read a phase sample of the oscillation | dq sweep, n_read, dt, drift, settled flag, return_err | n_read dwell reads + moves | 05 only |

Known blind spots that are NOT tool bugs (do not "fix" them): `counts.by_kind`
is debounced 0.4 s and undercounts continuous saturation by 4-15x -- use raw
per-step counts from the trace. `obj_lo/obj_hi` in the trace and in the recorder
are NOT a live world AABB: on 05 they have identical dims (0.3302 x 0.3302 x
0.3758) on all 13 seeds and the reported bottom sits 7.1 cm below the measured
table plane, i.e. they behave like nominal model bounds -- do not build a
clearance claim on them. `gv_*` contact list `c[0]` is a table contact for
non-touching seeds, so c[0] indices are not comparable across seeds. `grasp_scratch`'s `seed` is numeric-best over
the 3 starts, WITHOUT reach_box's seed-0 early break, so it can label a target
"seed 2" that reach_box would command on seed 0; use `ik_preview`
(`prefer_current=True`) when the question is which branch will be commanded and
`grasp_scratch` when the question is whether the closed fingers touch. `hand_object_contacts` matches the SceneObject
ROOT body only. `hold_parked_arms_and_measure_drift.table_contacts` counts by the
literal body-name substring 'table'.

## Round reports

### Round 2026-11-10 (07 actor_3 hand-back)
- Invisible decision: during the release rows the box is dragged -8 cm in y and
  spun -23 deg; the actor measured the outcome and the order permutations but
  could not attribute the drag to palm pads, the board edge, or residual
  object motion, and called the final position chaotic at the centimetre scale.
- Signal that decides it: per-row contact set involving the box body during the
  open/settle rows -- geom names, normal direction, tangential force -- in the
  object frame, compared against the box's velocity direction over the same row.
- Missing? No. `api.contact_pairs()` already returns the truthful contact set
  including child-body geoms; the primitive simply did not record it during
  those rows. So NO new tool was added at any layer. Action: the next 07 actor
  records `contact_pairs` per release row and re-runs the same pinned seed-0
  experiment; the re-run result will be reported here.
- Added: nothing. Removed: nothing (no tool's decision is settled this round:
  05's dwell question, 01's press-0 mechanism and 07's drag attribution are all
  still open, so `pad_hold_audit`, `window` and `probe contacts` stay).

### Round 2026-11-10 (01 actor_6 hand-back) -- one signal added
- Invisible decision: before commanding a retry palm target, which of the 3
  multi-start IK branches it will solve to. The actor could measure the
  outcome only AFTER the arm had moved (reach_box reports `ik_seed` post hoc):
  a closed-loop retry target 0.0362 m from the pad solved on branch 2 for the
  left arm (ik_err 0.0003) and swung the pad 0.3554 m (ratio 9.82), shoving the
  box. Its own next-step note names the gap verbatim: "preview the IK branch
  for the `2*dpad - pad` target ... Needs a copy of reach_box/_ik_arm logic (no
  api preview IK is exposed)." Quantities that decide it: winning seed index,
  joint-space travel (max abs, rad, over the 7 arm joints), pad displacement
  (m, world frame, from the live pad position), compared against whether seed 0
  solves at all.
- Missing? YES, and `grasp_scratch` was not a substitute: it takes explicit
  targets and reports a `seed`, but it (a) picks numeric-best over seeds while
  reach_box keeps seed 0 when it is good enough -- so its label can disagree
  with the branch reach_box will command, and (b) reports neither joint travel
  nor pad travel. The existing `ik_preview` had no explicit-target mode and no
  branch fields.
- Added, lowest layer (read-only, no command, live state untouched):
  `hmg_tools.py::ik_preview` gains `targets=` (same site-target semantics as
  `reach_box`), `prefer_current=` (replicates reach_box's seed-0 early break)
  and three per-row fields `seed`, `dq`, `pad_travel`. Same edit fixes a latent
  defect: seed 0 used to be the previous row's solution because rows write
  scratch qpos; it is now the live snapshot. Verified by a coordinator
  read-only smoke test on 01 (grid + explicit-target + +3.6 cm shifted target
  all return the new fields; seeds 2/1 at the reset pose, dq 1.5-2.0 rad).
- Re-run: the 01 track re-runs the SAME seed-20 retry experiment (screen the
  retry target with `ik_preview(targets=..., iters=400)` before commanding);
  result will be reported here. Bar unchanged: 20 TRUE with 0-19, 21 TRUE.
- Removed: nothing. No decision became settled this round (`pad_hold_audit`,
  `window`, `probe contacts`, `trace` all still answer open questions).

### Round 2026-11-10 (07 actor_4 hand-back) -- nothing added
- Invisible decision: whether seed 5's walk stall is reachable-by-phase
  (recoverable) or support-limited (not), since the pin and every variant leave
  seed 5's first-lower within +-0.03 m.
- Signal that decides it: base-y advance per 0.05 s walk sub-step (m) and the
  max contact-force magnitude (N, world frame, per row) against the 8 mm stall
  threshold. Missing? No -- both are already in the `env` field of each walk
  row, and the actor used them (the latch fires on 242.56 N; the 2x8 mm rule is
  phase-locked). So nothing was added at any layer.
- Confirmed cost boundary, now recorded: contact probes are INERT inside the
  walk step loop (seed 0 bit-identical) but NOT in a non-stepping gap (adding
  them to the settle row moved fm 0.1013 -> 0.0926, yaw 90.57 -> 91.26). A read
  cadence is a physical parameter; keep it fixed across compared variants.
- Removed: nothing.

### Round 2026-11-10 (01 actor_7 hand-back) -- nothing added; round-2 tool confirmed
- Decision it could not make: whether "refuse the branch-flipped correction"
  alone would be enough or whether the extra repair pass was needed. That is a
  question about the OUTCOME, not a physical quantity, so no probe can answer
  it -- it is the coordinator's engine bit. I read it: refuse-only is FALSE on
  seed 20; refuse+repair is TRUE on all 22 seeds. Nothing added.
- Round-2 tool confirmed inert where it does not decide: with the previewing
  candidate vs the no-preview control, 21 of 22 seeds are bit-identical
  (including ik_seed, clear_m, contacts_close); the 22nd is seed 20, where the
  screen changes the action by design. `ik_preview` runs on scratch MjData and
  does not perturb an episode in which it does not alter the command.
- Removed: nothing. The 01 branch decision is settled, but it was answered by
  the general `ik_preview`, which 03/05/09 still call; the track-local
  `pad_hold_audit` (05) still answers an open question.

### Round 2026-11-10 (05 actor_5 hand-back) -- nothing added; a SKILL was built instead
- Invisible decision: after the close, is the object actually held? Every
  pre-hoc quantity is a non-separator on 05 (pre-close n_fing 0-1 and gap
  0.1298-0.1360 on every seed, s7 and s9 identical), and the three obvious
  grasp reads are lies (`hand_contacts_side`, `close_grip()['contacts']`,
  `state()['hand_object_contacts']` all read 0 on a good grasp).
- The signal exists and was already reachable: `contact_detail(match=...)`
  filtered on `gripper0` plus `hand_geoms()` gives 3-5 contact rows and
  force 15.578-25.239 on every lifting seed, vs 0 rows / 0.0 / gap 0.0108 on
  the failing one. The actor packaged it as the read-only skill `grasp_verify`
  (e6d66d02) -- a skill, not a harness tool, which is the right layer: it is
  task-level knowledge about this grasp, not a general probe.
- Note for other tracks: this is the only truthful grasp classifier found on
  this robot so far; it is left-hand and bottle-filtered, so 07/09 cannot call
  it as-is, but the mechanism (contact_detail rows whose geom pair contains
  `gripper0`, plus the thumb2-index1 z gap) is general.
- Removed: nothing.


### Round 2026-11-10 (grasp_verify verifier hand-back) -- one signal added
- Invisible decision: whether a re-run is the SAME experiment as a recorded
  one. The verifier re-ran the pinned `drill_lift_v35` (same sha256) and got
  engine FALSE on 05 seed 12 where the record says TRUE; nothing in the
  response or the summary said which call configuration had been used, so it
  narrowed gate A to two seeds on a false premise. The deciding quantity is the
  exact `kwargs` dict (which source branch the call selects), compared between
  the recorded run and the re-run.
- Missing? YES at the coordinator layer: episode summaries carried no call
  configuration, and the response schema intentionally does not echo requests.
- Added, lowest layer: `hmg_server.py::run_episodes` now writes
  `primitive` and `cfg` (the exact kwargs) into the `.coord/episode_*.json`
  summary. Coordinator-only; the actor response schema is untouched, and no
  state, trajectory, reward or predicate value is involved. Takes effect when
  each track server next restarts (07/09 are mid-round and were NOT
  restarted). Measured effect already visible without it: with the actor's
  kwargs the same bytes give engine TRUE t 13.4 and an exact match to the
  recorded state; with `{}` they give FALSE t 16.9.
- Re-run: DONE and confirmed. The verifier re-ran gate A on 05 seeds
  0,1,2,3,4,7,12 with the pinned kwargs; all seven reproduce exactly (6 held
  + the seed-7 close-on-air), and the earlier "not reproduced" claim was
  retracted. Coordinator smoke test of the new field: 05 server restarted
  (registry clean; no actor depends on it), `{"tool":"run_seeds", "name":
  "cfgtest", "kwargs":{"off":{"seat":1}}}` wrote
  episode_05_drill_lift_214401.json with `primitive: cfgtest` and `cfg:
  {"off":{"seat":1}}`. Live on 05 only; 07/09 pick it up at their next
  restart (NOT restarted mid-round).
- Removed: nothing (no decision became settled: 05's dwell question is moot
  now that the track is closed, so `pad_hold_audit` is a candidate for removal
  at the next 05 restart, but no 05 work is planned).


### Round 2026-11-10 (07 actor_5 hand-back) -- nothing added
- Decision it could not make: whether s1's changed trajectory (the base-y latch
  ON under pre_lat_y=-0.02, which moves s1 off the pin's path) is still a
  success, and whether its box x-margin +0.0283 sits on the passing side of the
  boundary the pin's s1 +0.0252 sits on. The first is the engine bit and I read
  it: s1 is engine TRUE at t 58.46, and the whole 0-7 range is TRUE in one
  coordinator batch. The second is a calibrated proxy endpoint, not a missing
  measurement.
- Signal that would decide the proxy half: `verdict.x_margin` at the release
  row -- already returned by the candidate and already compared between the pin
  (+0.0252) and the new run (+0.0283). Missing? No.
- Confirmed this round (do not re-derive): margin proxies are NOT the engine
  predicate on 07. EXP V s7 reads fm -0.1043 and is engine TRUE (t 59.52);
  EXP Y s1 reads xm -0.0681 and is engine TRUE (t 61.3). A threshold fitted on
  fm or xm would have rejected two engine successes.
- Added: nothing. Removed: nothing.

### Round 2026-11-11 (05 actor_6 diverse round) -- nothing added
- Decision it could not make: whether its grip cluster (grip gap 0.0108 ->
  0.0671 m, hand_n 0 -> 4, held 0 -> 1, final z 0.838 -> 0.946, lift stop 3)
  is on the passing side of the predicate on seed 7, and whether s3's final z
  0.893 and s4's +0.5 s still pass. It asked for the engine bits explicitly and
  named the right quantity.
- Signal that would decide it: the engine bit itself -- coordinator-only. The
  bits show the cluster is NOT a predictor on s7: C0 (all cluster values bad)
  and C1 (all good) are both FALSE, at t 14.04 and 13.54. No proxy can be added
  that closes this: the difference the predicate reads is not visible in the
  object z, the pad gap, the contact count or the stop index. Adding another
  proxy would have predicted the wrong bit twice in a row.
- Also confirmed: s3 flips TRUE -> FALSE with the same end time (15.86) under C1,
  so t equality is not a bit either.
- Added: nothing. Removed: nothing.

### Round 2026-11-11 (05 actor_7 above-12 diverse round) -- nothing added
- Decision it could not make: whether H2's pad-route hull COMPOSES with the
  close-branch change -- i.e. whether 16/17/20 fail as per-seed interaction or
  additively -- and whether a routed seed whose pad lands at positive post x in
  the seat frame can pass at all (s20). It named the right quantity both times.
- Signal that would decide it: the engine bit per (seed, source, kwargs) under
  the composed off-dict -- coordinator-only; supplied, and it decided the
  composition half (16 and 17 flip TRUE, 13/15 keep TRUE, s20 FALSE under both
  branches). The s20 half needed no new signal: the response already carries
  seat_pre/post_rel x, and s20's +0.0432 sits outside the passing band
  [-0.044,-0.029] while its base move is 0.029 m for a 0.070 m need.
- Confirmed again this round (do not re-derive): final object z, lift stop index
  and branch label are NOT the predicate. s2 is TRUE with stop null at z 0.914
  and s20 is FALSE with stop null at 0.838; s13/15 flip TRUE with stop 3/4
  unchanged in kind. Five structurally distinct schemes (A, B, C, D, E, G) were
  refuted or rejected before H2; B and D were falsified on their own stated
  premise.
- Added: nothing. Removed: nothing.

### Round 2026-11-11 02:0x (05 selector round) -- nothing added
- Decision it could not make: whether the geometric close key fires on the pass
  side (it saw thumb2_y separated its 21 seeds, but the readings were its own
  proxy and the pass side is what protects 0-12), and whether forcing the thumb
  branch was free on 0-12. Signal: the engine bit per seed for the composed
  kwargs -- coordinator-only. It answered "no" for forced thumb (s2 FALSE at
  13.0 where the gate is TRUE) and "yes" for the geom key (20/21, s2 unchanged).
- Added: nothing. The deciding quantity was already in the actor's own pre-close
  probe log; no new read, no new tool. Removed: nothing.

### Round 2026-11-11 02:14 (05 R2-free round) -- nothing added, one signal declined
- Decision it could not make: whether the >=47 mm object displacement under the
  routed ramp (s20/s26/s32) is a transport defect removable by a different ramp
  or a seed-geometry limit. It named the deciding quantity precisely: minimum
  signed pad-object clearance along the ramp's swept path, metres, seat frame,
  compared with 0. That quantity is not logged (endpoints only; earliest_events
  covers the first 15 events; obs.trace is a path to an npz, not a base64 blob --
  a misreading the actor avoided).
- Signal added: none. Declined on purpose: nothing acts on the answer (the track
  ships at 38/41, retries were measured not to repair, the three residuals are
  documented). If a later round needs it, add it then.
- Removed: nothing.
- Also confirmed this round (do not re-derive): the actor's post-ramp
  displacement statement is a fitted description of 23 routed episodes measured
  AFTER the ramp (TRUE <= 11.4 mm vs 47.2/55.2/53.8 mm); it is not a pre-command
  signal and must not be gated on.

### Round 2026-11-11 03:04-03:46 (seven-track hand-backs) -- nothing added
Each actor named a decision its own feedback could not settle and the one signal
that would; none of the signals needed a new tool (all are already logged or are
the coordinator's engine bit).
- 05 (clearance uplift): whether ramp displacement responds to pad HEIGHT or only
  to the transient any pre-ramp move injects. Signal: pad z relative to the seat
  point at ramp start vs displacement during the ramp, pad placed by a non-servo
  source, 14 seeds. Answered by the same actor's next round: it is the transient
  (drill moves 232-331 mm from the pad), so no height sensor is needed.
- 05 (resegmentation): whether the early non-pad motion is avoidable by an
  approach waypoint or only by reacting. Signal: nearest non-pad body vs the
  drill at first >1 mm object motion, pass vs fail levels -- positions are
  already in the trace's q; no tool.
- 07 (latch_rows): whether pre-walk base y is causal or a correlate, and whether
  any command lowers y without changing x. Signal: one added pre-walk burst
  lowering base y >= 0.03 m with |dx| <= 0.01 m, then walk dy. Available with the
  existing commands.
- 07 (walk gate): whether the stall is foot/lower-leg clearance or WBC
  non-execution. Signal: foot/lower-leg contact force per burst at matched
  walk-start pose (bx/by within 0.01 m, yaw within 2 deg). contacts.jsonl already
  records it; no tool.
- 08 (guard24): whether the proxy repairs pass. Signal: the engine bit -- the
  coordinator answered NO (3/16/17 still fail) after a 41-seed run.
- 09 (v61): whether the rolled grasp is reachable under any +x-only nudge budget.
  Signal: shoulder-to-target distance vs 0.32 m with the target's lateral offset
  vs 0.15 m after 3 nudges -- both are in the primitive's own log.
- 03 (rz1/rz2): whether the unachieved pad motion is contact-blocked or at an
  IK/workspace limit. Signal: commanded-vs-achieved pad error per iteration with
  the pad retracted 5 mm -- needs no new read, only a variant of the existing
  correction loop.
- 01 (press screen): whether to enable press_branch='refuse'. Signal: the engine
  bit over 0-21 + 28/29/32/33; answered YES (24/26, then 59/61 over 0-60) and the
  skill was promoted.
- Added: nothing. Removed: nothing.

2026-11-11 04:20 -- 05 sub-control-step contact. The recorder samples ONE
post-step state per control step (control_timestep 0.02 s) and `hand_n` is an
instantaneous post-step count, so a contact confined inside a step's four 5 ms
substeps is invisible; asking for a 10 ms series does not get it (it would need
control steps split, which changes the episode). Decided signal that is NOT
available without a harness change: per-substep contact across a control step
(used to place a tilt onset on the contact side). Decided by: object z moving
+0.42 mm in the same 20 ms step the tilt begins, which is the proxy used
instead. Not implemented; no tool added.
