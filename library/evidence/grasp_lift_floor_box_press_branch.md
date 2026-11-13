# Evidence -- grasp_lift_floor_box_press_branch (01_box_lift_floor)

All engine bits are coordinator-read from `.coord/episode_01_*.json`; the actor
never read success. One episode per (seed, source, kwargs), same server.

## Pins

| role | source | sha256 | bytes |
|---|---|---|---|
| ship | actor_8/skills/grasp_lift_floor_box_branch_press_a8.py | dc56eb8b3568b0e2625a5086583e377bbeb2c1b2ef382a1641a33ec19cc7ba20 | 21744 |
| superseded | actor_7/skills/grasp_lift_floor_box_branch.py | 3eacdc603b2bc9153717de2d85946775b4b52729bd7ebbc794393bb646a5fe3c | 19116 |

Ship kwargs:
`{"branch":"refuse","branch_min_cmd":1,"repair_n":1,"repair_wide_add":0.0,
"repair_back_add":0.02,"press_ncmd":4,"seated_tol":0.0,"press_branch":"refuse",
"press_ratio":1000000000.0,"press_switch_corr":0,"press_travel":1000000000.0}`

## Gate A (engine predicate)

- Seeds 0-21 at the ship bytes/kwargs: TRUE 22/22 (coordinator, tag pressa8).
- Seeds 28, 29, 32, 33 (the pin's four failures on 22-40): TRUE on 29 and 33,
  FALSE on 28 and 32.
- Unseen 22-27 and 34-40 (tag pressa8ext): TRUE 13/13.
- Coordinator held-out 41-60 (same run): TRUE 20/20.
- Second held-out 61-80 (tag pressa8ho2): TRUE 17/20, FALSE 61, 66, 74 (t
  unknown at build time; every episode error null).
- Total over 0-80: 76/81. Over 0-60: 59/61.
- The superseded skill at the same kwargs is 37/40 over 0-40 (FALSE 28, 29, 32,
  33; the 22-40 extension 15/19 measured earlier).

## Supersession identity

At `press_branch='off'` the new source is bit-identical to the superseded skill
in all 15 trace leaves at seeds 0, 7 and 13 (coordinator runs pin01id and
pressoff, paired in the same session), and the actor measured the same at seeds
0 and 20. So the merge rule is met: the incumbent's verified runs are reproduced
exactly, and the union of the scopes is re-verified above.

## Gate B (verifier, seeds 0/6/20/29/33, candidate vs control)

- Ownership PASS: `act` is (T,31) in contract order, `nav` (T,3), height clipped
  to [0.44,0.90]; free base and object dofs carry max|tau| exactly 0.0; the only
  actuation path is `api.step_goal` -> `_send` -> `WBCGoal(upper,nav,height)`.
- Torque PASS: zero over-limit readings in 55 dofs across all five seeds against
  forcerange (hand servos 1.4 Nm, thumb_0 2.45) or max|ctrlrange| (body motors);
  leg worst fraction 0.691 (s0 left_knee); at-cap counts fall on the firing
  seeds (s29 34 -> 1, s33 100 -> 47) and one new at-cap step appears on s33
  (dof 35 right shoulder_pitch, step 745, 25.0 Nm) after divergence at 676.
- Determinism PASS: two reruns per seed, all 15 leaves byte-identical and the
  result JSON identical.
- Candidate vs control: seed 0 identical on all 15 leaves; seeds 6/20/29/33
  diverge first at steps 819/993/659/676, i.e. inside the press stage.
- API reads: net -8/-2/-14/-24 on seeds 6/20/29/33 (refused commands remove
  their reader/step sequence); seed 0 gains 9 scratch `ik_preview` reads and one
  `branch` log record with an identical trajectory.

## What the change buys, measured

| candidate | 0-40 | 41-60 | 61-80 | note |
|---|---|---|---|---|
| superseded skill | 37/40 | not run | not run | FALSE 28,29,32,33 |
| + press clear-screen | 39/41 (FALSE 28,32) | 20/20 | 17/20 | repairs 29 and 33 |

## Refuted and residual

- The press screen's prediction is miscalibrated (actor's own measurement, kept
  as CAUTION): refusals fired at predicted -0.0435 m with +0.0702 m achieved on
  s29, while the damaging commands on s29/s32 were predicted safe. It is an
  early-stop heuristic.
- An alternative, a widened `move_tol` of 0.06, repaired 3/16/17 in 08's sister
  experiment but is unrelated here; the 01 track rejected nothing else this
  round.
- Residual failures: s28 (box shoved in free space before the press stage),
  s32 (free-space placement), 61/66/74 (same class, unclassified).

## 2026-11-11: attitude-guard dz gate (a8c)

| role | source | sha256 | bytes |
|---|---|---|---|
| ship | actor_8/skills/grasp_lift_floor_box_branch_press_a8c.py | 49a138de785004a4b67aeac56905bc5c7647fa00bc0252b96f154210d2743b8c | 23934 |
| superseded | actor_8/skills/grasp_lift_floor_box_branch_press_a8.py | dc56eb8b3568b0e2625a5086583e377bbeb2c1b2ef382a1641a33ec19cc7ba20 | 21744 |

Ship kwargs unchanged from the section above.

### Gate A (engine predicate, coordinator)
- Seeds 0-80 at the ship bytes: TRUE 78/81, FALSE 28, 32, 61 (tag a8c80, one
  batch, 81 episodes, no errors). The superseded dc56eb8b is 76/81 (FALSE 28,
  32, 61, 66, 74). Gains: s66 0.5546 -> 0.8860, s74 0.2764 -> 0.8798 end z;
  s33 improves 0.8471 -> 0.8655; s60 unchanged at 0.8607 (its retry retained).

### Gate B (verifier, seeds 0/33/60/66/74, candidate vs dc56eb8b)
- Ownership PASS: act (T,31), nav (T,3), height (T,) only; no lower-body token
  in either source; non-actuated dofs (base 0-5, object 49-54) |tau| exactly 0.0.
- Torque PASS on the physical denominator (URDF effort for body, forcerange for
  hands): delta set `new_in_cand = []` on all five seeds, i.e. no at-cap or
  over-limit reading the control lacks. Set sizes candidate 6/3/1/3/3 vs control
  6/65/1/92/6 (at-cap:over on s0/33/60/66/74). The carried over-limit reading is
  left_wrist_pitch (5.35-5.90 Nm against 5.0), identical at the same steps in
  both arms; the control additionally exceeds at left_wrist_yaw and the
  waist/shoulder set.
- Determinism PASS: two reruns per seed, 15/15 leaves byte-identical.
- Delta identity: s0 1061/1061 steps, 15/15 leaves, logs equal, retries 0;
  s60 1081/1081, retries 1, same retry row (down_tilt, dz -0.088, attempt 0);
  s33 764 vs 881, first difference step 519; s66 764 vs 958; s74 764 vs 978.
  The three differing seeds are exactly the down_tilt fires with |dz| < 0.05 m.

### Residuals
s28 and s32 are press-stage/free-space failures (the box is already displaced
before the press stage); s61 fails on an in-contact lateral retract drag. The
threshold is fitted on four fires over 0-80.
