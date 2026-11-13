# Superseded catalog entries (coordinator, 2026-11-10)

Entries retired from `PRIMITIVES.md` by a verified merge. Nothing in this file
is a live catalog entry: it preserves the provenance (hash, scope, evidence) of
sources that are no longer recommended for reuse. Files on disk are NOT deleted.

## 08: extract_held_horizontally + lift_held_vertically → move_held

Retired: 2026-11-10, after the merge reproduced both originals bit-identically.

### extract_held_horizontally(hold, axis=0, clear=0.34, d=(-0.032,0,0), side='left', dur=0.8, iters=250, max_steps=8, obj_tag='drill', bar_tag='holder')
- source hash: 2ef9b92407aed4a08cc7d342a1eff09f2094cd829a3679ac45d9d8e90c4126fe (work/tracks/08_pick_drill_from_holder/actor_extract/skills/extract_held_horizontally.py)
- removed: the interleaved pull+lift that catches the held object's top on the holder lip, tilts it and stalls the rise -- extract along the slot first.
- verified scope (as catalogued): 08 seeds 4,5,6,7 (verifier 4-6, coordinator 7). The `clear` criterion is met on NO seed (cap-limited), and it does NOT verify its grasp: on seed 3 the object left the gripper at row 4 (z 0.718 -> 0.398). Verified only for a grasp that is on the handle.
- used by: 08_pick_drill_from_holder composition.

### lift_held_vertically(hold, axis=2, lift_z=0.82, d=(-0.004,0,0.024), ik_stop=0.003, side='left', dur=0.8, iters=250, max_steps=8, obj_tag='drill', bar_tag='holder')
- source hash: 83c522496efc0abbb262340dffdce0c070b4abd074467cc18bcb5abb9475089f (work/tracks/08_pick_drill_from_holder/actor_extract/skills/lift_held_vertically.py)
- removed: lifting while the object is still inside the holder, where the lip catches it.
- verified scope (as catalogued): 08 seeds 4,5,6,7. It NEVER reaches lift_z 0.82 -- it stops at the IK wall (ik_err 0.0069-0.0109 against ik_stop 0.003), and the final rise is supplied by `boost_base_height_holding_command`. Verified as a BOUNDED raise, not as a lift to height.
- used by: 08_pick_drill_from_holder composition.

### Replacement
`move_held` sha256 a8276b1eb041640abb493baa34d96f638a5548c2c29d36dad9e7f76433ccdbed
(work/tracks/08_pick_drill_from_holder/merge_round/move_held.py), catalogued in
PRIMITIVES.md. Evidence (merge_round/):
- extract mode vs the old skill: seeds 3,4,5,6,7, 3424/3424 leaves compared,
  0 diffs, max abs delta 0.0.
- lift mode vs the old skill: seeds 3,4,5,6,7, 4112/4112 leaves, 0 diffs,
  max abs delta 0.0.
- the merged composition `compose_merged.py` f733542b…eaa7 vs the pinned
  promoted program 035ede59690c7314b10ac5e65c679d4cf2c131b3b7e860a1f6540e93707180f9:
  seeds 3,4,5,6,7, 4664/4664 leaves, 0 diffs, identical t
  (28.34/34.10/32.50/34.10/34.90). The only source change between the merged
  composition and the pinned `compose.py` is the two skill names.

Merge caveat recorded in the catalog: lift mode must pass `ik_stop=0.003`
explicitly, because `move_held` only breaks on the IK wall when `ik_stop` is
set; without it the traces diverge from the original on every seed.

## 06: level_upright_single_axis + level_upright_two_axis → level_upright

Retired: 2026-11-10, after the merge reproduced both originals bit-identically
in a stage replay and the merged composition reproduced the promoted program
exactly.

### level_upright_single_axis(joint=15, delta=0.2, target=0.97, min_gain=0.002, max_tries=3, dur=0.5)
- source hash: 515e2b04b5330037b5ae80d6d3d7cd6151b9a14e942ac46282318c79a4f1619c (work/tracks/06_drill_pnp/actor_extract/skills/level_upright_single_axis.py)
- removed: pushing a wrist joint a fixed amount "because it helped last time" -- the same direction can end LESS upright and an open-loop rotation cannot know.
- verified scope (as catalogued): 06 seeds 0-9, z_up 0.8789-0.9558 -> 0.9702-0.9941, target 0.97 exceeded on every seed.

### level_upright_two_axis(joints=(14,15), step=0.08, target=0.996, min_gain=0.001, max_tries=4, dur=0.35, reverse=2.0)
- source hash: 37b45993d8cdcc50e39686760ddc00a0ff72eb0d80b46eb9c112783f97d03618 (work/tracks/06_drill_pnp/actor_extract/skills/level_upright_two_axis.py)
- removed: the residual tilt about the second wrist axis that survives a dominant-axis pass.
- verified scope (as catalogued): 06 seeds 0-9, with the narrowing that on seed 1 the returned uprightness 0.9657 is BELOW the 0.9756 entry value. CORRECTED 2026-11-10: the empty log IS present in the real composition on seeds 1 and 9 -- an empty ACCEPTED log (probes issued and reverted), which is why the exit sits below entry. The merge stage replay showed 7 entries only because it entered at a different state (z_up 0.9084). The difference is between contexts, not between the old and merged skills.

### Replacement
`level_upright` sha256 869195fcbf42c7f12d416a77fd3c04080c1f9627e82eced8953ece04844ed238
(work/tracks/06_drill_pnp/merge_round/level_upright.py), catalogued in
PRIMITIVES.md. Evidence (merge_round/):
- composition-context stage replay, single mode and two mode, seeds 0-9:
  0 differing leaves, identical geom/command counts, max abs delta 0.
- merged composition `compose_merged.py` b765a328…7236 vs the pinned promoted
  program 455a237a…358f, seeds 0-7: 0 differing leaves per seed, identical t
  (31.36/23.70/31.46/32.32/31.64/34.66/33.80/29.12).
- Migration hazard: kwarg names changed (`joint`->`joints`, `delta`->`step`)
  and `**kw` swallows the old names, so an unmigrated call silently runs the
  two-axis defaults.

## 01: place_pads_closed_loop → place_pads_branch

Retired: 2026-11-10, after the merge reproduced the incumbent's trace
bit-identically at `branch='off'`.

### place_pads_closed_loop(tag='place', stand=0.0, xb=0.0, zu=0.0, ncmd=2, free=True, hold=None, out0=None, dur=0.40, lo=None, hi=None, box_ref=None, x_in=0.03, z_frac=0.94, pad_half=0.0217, off=(0.076,-0.018,-0.030), box_move_tol=0.02, box_move_hard=0.25)
- source hash: dd05958a86e3f27dcf548f42cdb326c59101a4e5977b3bf06480dbf5f3461b82 (work/tracks/01_box_lift_floor/actor_4/skills/place_pads_closed_loop.py)
- removed: a one-sided grip from an unseated pad; and a CUMULATIVE displacement gate aborting later corrections after one command had swept the box.
- verified scope (as catalogued): 01 floor box. Seated bilaterally on seed 5 (verifier) and re-scored on 3,4,7,8; NOT seated on 0,2,6,9,10,11,12. It does not recover a gross miss (one pad 4.5-6.6 cm clear on unseen 10/11/12) and those episodes still lift; on held-out 13 one retry recovers reach but the close is c2 L1/R1 and the box is shoved (AABB top 0.276). No cross-task evidence.
- used by: 01_box_lift_floor composition (free-space reach and the press's inner loop).

### Replacement
`place_pads_branch` sha256 35cc36386db2c99bb53924b4e2459e8f5b60e873a8576a515a009d9ca524eae7
(work/tracks/01_box_lift_floor/actor_7/skills/place_pads_branch.py), catalogued
in PRIMITIVES.md. Evidence (work/tracks/01_box_lift_floor/verifier_3/):
- branch='off' vs the incumbent, seeds 0/13/20, identical inputs (recorded
  settle box_ref/lo/hi): 15/15 trace arrays equal (np.array_equal, NaN-aware),
  0 differing leaves after ignoring `/jump_guards` (absent vs []) and the
  `/log` phase tag (`m0_closed` vs `m0_branch`). This is a code-path identity
  proof in a wrapper context, not a full-episode identity -- the wrapper replays
  the place call only, so both arms' commands are reach failures.
- the added `branch='refuse'` path is what fixes 01 seed 20; `branch='off'`
  reproduces the incumbent exactly.


## 07: carry_and_seat_on_shelf pp31 -> pp33

Superseded entry (was catalogued 2026-11-10 23:0x, then re-pinned):
- source `3b41be06d110aaeec0c75d05de752b029855b85014bcbf76c05f7f1518ed25a9`
  (07_box_table_to_shelf/actor_5/pp31_a5.py, 54497 B) with 34 call kwargs.
- verified scope then: seeds 0-8 and 10-13 engine TRUE at that artefact; seed 9
  FALSE (board_gap +0.0072, yaw 149.12, box propped on one edge).
Reason for supersession: seed 9's failure was measured to be the seat row
shearing a FREE box (the open ended contact-free, released hb.f 0.026 N) and
propping it on one edge. pp33 adds `rel_grasp_min=0.5` and latches the seat
row's lateral component on the already-recorded released grip force, with no
new read and no new command.
Merge/identity evidence: 18 of 20 seeds are bit-identical in all 15 trace
arrays; every non-seed-9 (and non-seed-19) result differs from pp31 in exactly
one leaf, /knobs/rel_grasp_min. Coordinator: seeds 0-13 engine TRUE; verifier
gate A: unseen 14-19 engine TRUE; gate B deterministic and ownership-clean at
the new hash. Full detail: library/evidence/carry_and_seat_on_shelf.md.

## 01: grasp_lift_floor_box_branch -> grasp_lift_floor_box_press_branch

Superseded entry (was catalogued 2026-11-10, re-pinned at actor_7):
    # grasp_lift_floor_box_branch -- open before calling
    
    Signature: run(api, **kw), 70 keys: approach_x=0.67, h=0.46, lift_h, press_ncmd=4, seated_tol=0.0, branch='off', branch_min_cmd=0, repair_n=0, repair_wide_add=0.0, repair_back_add=0.02, retry_n, retry_back_add, y_wide=0.10, x_back=0.05, ... (see source for the rest)
    - source hash: 3eacdc603b2bc9153717de2d85946775b4b52729bd7ebbc794393bb646a5fe3c (01_box_lift_floor/actor_7/skills/grasp_lift_floor_box_branch.py)
    - removes: the floor-box failure where the retry's closed-loop palm target re-solves onto another IK branch and sweeps the pad through the box before the post-command guard can act.
    - applies when: box at rest on the floor within reach of a standing stance at `approach_x`; one env, one episode.
    - contract: pre: box at rest -> post: pads seated, fingers closed, body standing at `lift_h` holding the box; returns log, box_ref, box_end, box_end_z_hi, contacts_close, sides_at_close/end, reach_ok, reach_failed_at, retries, repairs, branch_mode/refused/refusals and the guard decisions. `branch='off', repair_n=0` is bit-identical to grasp_lift_floor_box_jumpguard.
    - verified scope: 01 floor box. Independent verifier reproduced seeds 0,6,13,20,21 with 0 differing leaves vs the actor log and byte-identical determinism reruns; engine TRUE on all seeds 0-21 (coordinator). SCOPE IS 0-21 ONLY: an extension to unseen seeds 22-40 is 15/19 (FALSE on 28, 29, 32, 33), i.e. 37/40 over the whole tested range -- a seed above 21 is not covered. Gate C: no cross-task evidence -- the same screen has no valid second-task instance on 03.
    - used by: nothing yet (the 01 task solution).
    - CAUTION: peak |tau|/ctrlrange reaches 1.00000 on a shoulder in 3 of the 5 verified seeds, and the composed episode commands wrist-roll targets up to 0.032 rad outside the model range (realized stays inside only via the PD clip); base drop 0.12 m and tilt ~0.47 rad at placement.
    
    Proof history: `library/evidence/grasp_lift_floor_box_branch.md`.

Reason for supersession: the press stage of this composition could send a
closed-loop correction that re-solved onto IK seed 2 and swept a pad through the
box; on seeds 29 and 33 that cost the episode. The successor runs the SAME
pre-command branch screen on the press command stream (`press_branch='refuse'`,
clear criterion only) and repairs 29 and 33, at the same 22/22 on 0-21 and with
no loss anywhere in 0-60.
Merge/identity evidence: at `press_branch='off'` the successor is bit-identical
to this source in all 15 trace leaves at seeds 0, 7 and 13 (coordinator, paired
same-session runs) and at seeds 0 and 20 (actor). Union of scopes re-verified:
0-60 is 59/61 (FALSE 28, 32), held-out 41-60 20/20, second held-out 61-80 17/20.
Full detail: library/evidence/grasp_lift_floor_box_press_branch.md.

## 01: grasp_lift_floor_box_press_branch dc56eb8b -> 49a138de (attitude-guard dz gate)

- superseded source: 01_box_lift_floor/actor_8/skills/grasp_lift_floor_box_branch_press_a8.py,
  sha256 dc56eb8b3568b0e2625a5086583e377bbeb2c1b2ef382a1641a33ec19cc7ba20, 21744 B.
- superseded scope: 76/81 over 0-80 at the ship kwargs (FALSE 28, 32, 61, 66, 74);
  59/61 over 0-60; held-out 41-60 20/20, 61-80 17/20.
- reason: the reach attitude guard retried on any `down_tilt` label, so a fire on
  a COMPLETED approach discarded it (retract + full re-solve from wide_hi) and
  dragged the box even when the pad normal had barely moved (dz -0.010..-0.030 m
  on s33/66/74). The successor gates the retry on the reported drop
  (`-dz >= att_tilt_dz`, default 0.05 m) plus named reasons, keeping the one
  load-bearing fire (s60, dz -0.088 m).
- successor scope: 78/81 over 0-80 (FALSE 28, 32, 61), repairing 66 and 74.
- merge/identity: identity is NOT claimed on the firing seeds -- that is the
  change. Verified identity where the guard does not fire: s0 bit-identical over
  1061 steps (all 15 leaves) and s60 bit-identical over 1081 steps with the same
  retry row (dz -0.088, attempt 0). On s33/66/74 the only difference is the
  presence/absence of that retry row, first differing step 519.
- gate B (successor, seeds 0/33/60/66/74): ownership PASS, torque PASS with
  `new_in_cand = []` (no at-cap or over-limit reading the control lacks; the
  candidate's at-cap/over set is 6/3/1/3/3 against the control's 6/65/1/92/6),
  determinism PASS (15/15 leaves on two reruns).
- CAUTION carried by the successor: the 0.05 m threshold is fitted on four
  observed fires (a gap, not a distribution).
Full detail: library/evidence/grasp_lift_floor_box_press_branch.md.
