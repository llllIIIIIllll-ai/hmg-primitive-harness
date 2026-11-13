# Whole-task lessons (coordinator, moved from PRIMITIVES.md 2026-11-10)

Failure signature -> what worked -> reusable pattern -> verified scope, one
entry per task round. These are narratives about whole compositions, not
skills; the skills they produced live in `PRIMITIVES.md` with their own
evidence files. Verbatim as they stood before the compaction:

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

## A replay is not a repeat: walk_to steps along the CURRENT base yaw (05, v38)
Replaying an approach sequence from a later point in the episode did not
reproduce the approach. `walk_to` walks along the current base yaw, so a replay
that starts at the pinch-time yaw (-3.69 rad) while the original started at 0.34
walks a different path with identical arguments: the measured walk2 and walk3
saturated at their maxp budgets and ended +20.8 cm and +11.8 cm past the target
in x, with a 0.90 rad yaw error, and the subsequent IK degraded from ~0.001 m to
0.095-0.178 m. Consequence: never treat "re-run the same sequence" as a repeat
-- restore base pose AND yaw explicitly first, and check for maxp saturation in
the walk log (n == maxp) instead of assuming a target was reached.

## A pinch close can roll a free-standing object out of its own closing volume (05)
On 05 the drill is never un-graspable: the pre-close geometry is identical on all
13 seeds (gap equal to 4 dp). What separates the failing seed is rotation. The
first pad to touch loads the drill off its vertical axis, and because the base
only rests on its support, that load produces a moment about world +x
(measured axis [1.000,-0.009,-0.012]) which keeps growing while the closing pads
advance: 36.4 deg at a 0.5 s close, 85.7 deg at 1.5 s, ending with the pads at a
NEGATIVE measured gap and zero finger contacts -- they close on air at the pose
the drill has already rolled into. Two consequences: (1) roll magnitude scales
with close DURATION, so slowing a close cannot fix a roll-limited grasp -- it
made a second seed fail and moved two more to the z_stop boundary; (2) the
failure is geometric, not force- or reach-limited, so the fix has to change the
approach line or the closing volume, not the pre-close pose, the base, or the
close timing.


## A source hash is not the experiment: the `run_seeds` kwargs select the branch (05, gate A)
The `grasp_verify` verifier re-ran the pinned `drill_lift_v35` (sha256
`9ec69d48…`) and could not reproduce four of the actor's seeds, then narrowed
gate A to two seeds on that basis. The source was identical. The actor's run
had been submitted with an explicit configuration dict
`off{seat:1, seat_mode:'branch', br_on:1, br_gate:1, ps_on:0, tw:0,
pin1_ramp:0, pre_stance:1, gate:1, servo_on:0}`; the verifier called with
`kwargs {}`, which selects the source's default branches. On seed 12 the same
bytes then produced pinch IK seed 2 instead of 0, seat mode `pad_seat` instead
of `branch`, and the difference between engine FALSE (t 16.9, object never
lifted) and engine TRUE (t 13.4, object at z 0.953, exact match to the actor's
recorded numbers). Consequence: any reproduction or gate-A claim must pin the
SOURCE HASH AND THE CALL PARAMETERS, and a verifier must read the kwargs out of
the original request file (`queue/<task>/req_*.json`) before concluding that a
trajectory does not reproduce. A default-args run of a pinned source is a
different experiment.


## The running process is part of the artefact (09, v39 ladder)
A measurement ladder built against `api.ik_preview(targets=...)` returned
`err -1` on every rung -- not a physical result but `KeyError('pad_travel')`
swallowed into `preview err 1` -- because the track server had been started at
18:37:05 while the patched `hmg_tools.py` on disk is dated 18:53:31. The source
hash of the harness the actor read was not the code the server was executing.
Consequence: a server restart is a change to the experiment, and a tool patch
takes effect only at the next `server_ctl.sh start`; when a tool returns a
sentinel error rather than a number, check the process start time against the
module mtime before treating it as a physics result. The same rule is why an
`ik_preview` run and a `reach_box` run must be compared on the same process.

## `ok: true` is not "the episode ran" (05, drill_c7 build)
A primitive that raises inside the episode still returns a response with
`ok: true` at the request level; the exception lands in the EPISODE's own
`error` field and every result key it should have produced is null. On 05 the
missing builtin (`int()` is not in the exec globals) produced exactly that:
`ok=True`, `err` set, `close_gate`/`gv_preclose`/`final_obj` all null. A run
that only checks the top-level `ok` would have recorded the nulls as physics.
Consequence: read `episodes[i]['error']` before any number from that episode,
and treat "field absent/null" plus `ok:true` as a failed run, not a result.

## The in-process primitive registry is empty after every server restart (09, 2026-11-11)

Measured on the 09 track, but it is a harness property and every track will hit it.

- `hmg_server.py <task>` resolves `api.skill(...)` and any named primitive against
  primitives registered IN THE RUNNING PROCESS. A fresh `./server_ctl.sh start`
  begins with an empty registry.
- A request that names a primitive without carrying its source therefore fails
  INSIDE every episode with `KeyError: no primitive 'top_up_reach_at_standoff'`,
  and the queue still answers `ok: true`: the exception lands in the episode's
  own `error` field and every result key is null. Eight seeds came back in one
  batch looking served.
- Fix: re-register byte-identically in the same request (or in a preceding one on
  the same process) and re-check the pinned sha256s. The hashes that were
  re-registered and matched on 09: walk_base_to_coordinate 66f57455…cd64 (919 B),
  top_up_reach_at_standoff 7a8d8766…f01c (4493 B),
  assist_right_roll_press_at_residual cc5a6b87…2341 (10347 B).
- Related: the earlier v51 request was never served at all (0-byte response after
  the timeout) and needed a server restart. A 0-byte response is "not served",
  not "returned nothing".

Rule for actors: after any restart, the first request on a track must be a
registration request or must embed the sources it names, and `ok: true` with null
result keys is a failed run, not an empty success.

## A read is not free: mj_forward in a probe perturbs the next step (gate C, 2026-11-11)

Measured while adding a pre-close probe to 06 and 08 for a cross-task test, on all
12 seeds tested (06: 10,14,16,17,18,28; 08: 0,3,4,10,16,17).

- The added probe calls observe()/base_state()/hand_geoms()/geom() and commands
  nothing. Full-trace bit-identity does NOT hold: differences of ~1e-7 appear in
  act/obj/q, ~1e-5 in qd/tau (06), and up to 0.65 in act and 0.18 in obj (08).
- Every recorded array is bit-identical for all samples strictly BEFORE the probe
  sample, with the first difference exactly at the probe index (06, all seeds) or
  one sample after (08 s4). So the probe reads the pinned state exactly; only the
  dynamics after it diverge.
- Cause: `qacc_warmstart`, which the harness never resets. `mj_forward` inside the
  probe overwrites the warm-start acceleration, which changes the next `mj_step`'s
  solver path while leaving qpos/qvel untouched.
- On contact-rich episodes the divergence is macro-scale: 08 s16 total t 27.66 ->
  26.86, s17 29.70 -> 26.50, 06 s10 31.86 -> 31.22; other seeds unchanged.
- Consequences: (1) the "reads are physical, never strip them" rule is right for a
  reason beyond cadence -- adding a read changes the episode; (2) an instrumented
  variant may be used to READ a quantity, but its outcome bits must come from the
  pinned program, not from the variant; (3) a quantity that already sits in a
  pinned program's log (as 05's does) is the only kind that can be compared across
  tasks without this confound.

## The arms are not independent, and a single-side reach hides the other hand

Measured on 03 (actor_2, 2026-11-11, seeds 17/29/38/21, `resp_rz7e.json`):
- `api.reach_box(targets={s:...}, sides=[s], R_body=..., dur=...)` returns the pad
  position of the COMMANDED side only, with that side's residual. The uncommanded
  palm still moves: 1.4-4.4 mm when the right hand is commanded, 4.1-40.3 mm when
  the left hand is (independent `api.observe()['object']['hand']` read), dragging
  the box 2-26 mm.
- A single-side probe that reports the uncommanded displacement as exactly 0.0000
  is a `pads_of` fallback artifact, not a measurement: the response carries only
  the commanded side. Use an independent observe for the other hand.
- Magnitude and direction are not guaranteed: with the right pad 82-121 mm outside
  the whole box AABB and box drag <= 4 mm, a 40 mm command is realised with
  9.8-44.4 mm error (achieved/commanded 0.13-1.79), and a body along-face command
  produces 0-2 mm on that axis and 11-22 mm perpendicular (~90 deg direction
  error) on all four seeds.
- Consequence for any correction loop that commands both hands: the residual of
  one pad is arm error PLUS the other pad's coupling, so extra iterations cannot
  converge on it -- and in 03 they destroyed the closed grasp (rz2, 0/8).

## A scratch read is read-only in state but not behaviour-neutral

Measured on 06 (actor_4, 2026-11-11): `grasp_scratch` (a clone used to screen the
pre-close pose without touching the live sim) changes the LIVE episode anyway.
Seeds that take no command at all shift up to 81.8 mm in the largest world
component versus the same program without the screen, deterministically -- most
likely it consumes the episode RNG that the randomised reach draws from.
Consequences:
- A screen is not a free observation. Any candidate that adds one must be
  compared against a no-screen control, not against the pin, before its effect is
  attributed to the action it gates.
- The 06 actor's own perturbation control (`diag_v22`: screen, no command) is the
  right instrument: it showed s18 still failing, so the nudge -- not the screen --
  is what fixes that seed.
- The screen did not flip any of the 18 unaffected seeds on 10-30, but that is an
  outcome statement on 21 seeds, not a claim of neutrality.

## A commanded step is only evidence against a zero-command control
03, 2026-11-11. Four commanded along-face step sizes (10/20/40/80 mm) all
achieved LESS along-face displacement (-2 to -12 mm) than the zero-command
control on the same steps (-10 to -15.5 mm), so the "small residual" that looked
like a resolution floor was drift plus a pinch-axis bias. Before calling a
command too small to move the target, run the same instrument with a zero
command; the solver residual (ik_err 72 -> 161 mm tracking |palm_residual|) and
the drift bias can each be larger than the commanded effect.

## 06: the close-phase proxy is refuted in both directions
06, 2026-11-11. Two candidates moved the same quantities in the "right"
direction and neither tracked the engine. `diag_v20` (pre-close screen plus a
y-only palm nudge) repaired the proxy on s18 (4 contacts, lift_lost false,
completed lift) and the bit stayed FALSE. `diag_v31` lowered the pre-close
standoff 6 mm: on s18 dz across the close 35.8 -> 28.1 mm, z_up 0.8752 ->
0.9276, contacts 2 -> 4, lift_lost True -> False, monotone over the -6..+3 mm
grid, and every passing seed still read lift_lost False -- yet the engine lost
s23 and s26 and repaired nothing. So for this task, contact count, lift_lost,
close-phase dz and post-close z_up are not sufficient evidence in either
direction; a candidate must be judged on end-state error against a passing seed,
and the engine bit is the only acceptance test. Do not build a 06 candidate on a
close-phase proxy again.

## A proxy the engine already reports is not independent evidence
07, 2026-11-11. An actor "verified" its pass rule by recomputing the engine's own
verdict fields and finding they agreed to sub-millimetre precision -- but it was
comparing the engine's numbers against the same AND-of-four-margins rule, so the
agreement was circular. The engine actually gates on board_gap (box on the board)
and tolerates negative x_margin (7 seeds) and front_margin (4 seeds) up to
-112.3 mm. Always separate "reproduces the recorded quantities" from "reproduces
the decision": quote the boundary and the seeds that cross it.
