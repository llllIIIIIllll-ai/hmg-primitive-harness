# Evidence -- lift_drill_pad_route_geom_close (05_drill_lift)

All engine bits are coordinator-read from `.coord/episode_05_*.json`; the actor
never read success. Episodes are one per (seed, source, kwargs), same server.

## Pins

| role | source | sha256 | bytes |
|---|---|---|---|
| ship | actor_7/programs/drill_t_r2free.py | 3fd440a55ccbbcdb5c274203617c907f2c7c15af3566ea0fed36b7874a8855e8 | 95057 |
| parent | actor_7/programs/drill_t.py | 86fdd43fe876062a41b953ce12e9695c86598088e7e361214bc5de556d88ebbe | 94006 |
| grandparent | actor_6/programs/drill_c7.py | afe4899f3d54b85abd8684ce08caa87c1e06619ccaf334ab6922434dacef03aa | 86888 |

Ship kwargs: `{"off": {"br_gate":1,"br_on":1,"close_gate_nf":1,"gate":1,
"pin1_ramp":0,"pr_m":0.002,"pr_on":1,"pre_stance":1,"ps_on":0,"seat":1,
"seat_mode":"branch","servo_on":0,"tw":0,"close_key":"geom","geom_thumb2_y":-0.055}}`

## Gate A (engine predicate)

- Seeds 0-20, ship bytes, one episode per seed: TRUE 20/21, FALSE s20 (t 17.04).
  0-12 bit-identical to drill_c7 in all 15 trace leaves (compared seed by seed
  from the on-disk trace.npz; the only differing seeds are 13 and 15 in `t`, 16
  and 17 in `q`, 20 in `t`); 13, 15, 16, 17 recovered.
- Seeds 21-40 (unseen), same bytes/kwargs: TRUE 18/20, FALSE s26 (16.00),
  s32 (18.58). Total 38/41 over 0-40. Verified scope is 0-20; 21-40 is a
  measured extension.
- Coordinator held-out seeds 41-60 at the same bytes/kwargs: TRUE 18/20, FALSE
  s45 (13.00), s46 (13.90). So above seed 20 the rate is 36/40 and the total
  over 0-60 is 56/61. Every episode error null.
- H2 without the R2-free widening was 20/21 on 0-20 and 16/20 on 21-40 (36/41),
  FALSE 26, 29, 32, 35.
- Coordinator bit requests: `req_coord05_H2geom.json` (0-20),
  `req_coord05_H2geom_21_40.json` (21-40), `req_coord05_R2free_0_20.json`
  (0-20 confirmation at the ship bytes).

## What each change buys, measured

| change | 0-20 | 21-40 | note |
|---|---|---|---|
| drill_c7 baseline | 13/13 on 0-12, 3/8 on 13-20 | -- | close gate blind above 12 (n_fing 0 on all eight) |
| H2 pad-route hull (2 mm) | 18/21 | -- | 13, 15 recovered; 0-12 unchanged |
| + forced thumb (close_gate_nf 0) | 20/21 on 0-20 but **s2 FALSE** | -- | rejected: forcing is not free |
| + geom close key (-0.055) | 20/21 (s20 F) | 16/20 | close key never mis-fires on a passing seed |
| + R2-free route entry | 20/21 (s20 F) | 18/20 (26, 32 F) | ship; 29 and 35 recovered |

The geom key's separation over 0-20: TRUE seeds read thumb2_y in
[-0.0415, -0.0029]; the FALSE ones (16, 17, 20) in [-0.0673, -0.0631]. On 21-40
it never fires on a passing seed; the 21-40 failures are upstream (29/32/35 were
blocked by the 2 cm guard) or downstream (26 routes and lands outside).

## Gate B (verifier, seeds 0/7/16/20, parent compared at the same seeds)

- Ownership PASS: act is 31 columns in `ALLOWED_UPPER` order, nav 3 + height 1
  separate (35 total), no lower-body channel, lower-body tau is WBC output only.
- Determinism PASS: 15/15 trace leaves byte-identical across two reruns per seed;
  logs byte-identical.
- Torque PASS on the physical denominator (hand servos are force-limited,
  forcerange 1.4 Nm, thumb_0 2.45; nothing exceeds its limit anywhere). At-cap
  readings are saturation, not overshoot: inherited left thumb_1 s0 66/650 and
  s7 53/677 steps, left index_0 s0 27 steps, five right-hand joints 1 step each
  every seed; introduced at s16 left thumb_1 69/800 steps, left index_0 4,
  left middle_0 3 (parent never at cap there). Right thumb_1 introduced delta
  +0.216/+0.205 Nm at s16/s20 = 0.79 of forcerange.
- Joint-range sub-criterion FAILS on inherited data only: commanded targets up to
  0.0509 rad outside the model range on left_shoulder_yaw, right_shoulder_roll
  0.0055 rad, and measured-q JOINT_LIMIT events at t 4.84-4.96 -- identical in
  the parent at the same seeds.
- The close-key block adds no API read (geom vs nf vs absent are trace-identical
  at s0; the block reads `log['gv_preclose']`).

## Gate C (cross-task reuse, 3 FALSE + 3 TRUE seeds per task)

Quantity: pre-close `gv_preclose['thumb2'][1]` = world-y(thumb_2) - world-y(object
root), left hand in 05.

- 06 (right hand, probe t 4.50 on all seeds): passing 20.4-23.9 mm, failing
  1.3-13.1 mm -- gap 10.8 mm world, 14.7 mm object frame, 11.3 mm base frame;
  6/6 at the best threshold in every frame. So the ORDERING transfers.
- The threshold does not port: all 06/08 values lie in [-0.0004, 0.0339], so
  -0.055 fires on zero seeds. r(object-frame value, object y) = -0.81 on 06, so
  the quantity is partly a restatement of where the drill sits.
- 08 (left hand) is falsified: TRUE [2.6, 20.2] mm overlaps FALSE [12.4, 29.2],
  best 5/6, and s17 is FALSE at 12.4 mm while s0 is TRUE at 20.2 mm -- inverted,
  and opposite in sign to 05/06. The probe instant is not equivalent on 08
  (t 11.26-20.10; s4 never emitted creep_stop, s10 already touching).
- Consequence: no cross-task scope is claimed for this skill.

## Refuted sub-claims kept in the record

- "Force the thumb branch everywhere": s2 flips FALSE at t 13.0 (thumb2_y
  -0.0065), while the n_fing/geom gate keeps it TRUE at the same t.
- "The post-transport x band [-0.044,-0.029] is the capture condition": it holds
  for the 0-20 routed movers only; 13 of 16 TRUE seeds on 21-40 lie outside it.
- "R2-free will recover 29, 32, 35" (pre-registered): 29 and 35 recovered, s32
  did not (route_pad fires, pad ends 37-42 mm lower in object z). One of three
  predictions falsified.
- Scheme B (frozen 31-vector), D (2 mm settle) and the "any tilt-aware re-aim
  helps" reading were each falsified on their own stated discriminator.

## Residual failures (three classes, not one)

- s20: route active, branch authoritative, post-transport pad x +0.0432 m after
  2.9 mm of motion where >=72 mm was needed.
- s26: routed ramp lands 21.2 mm below the worst accepted pad height.
- s32: veto cleared by 0.0025, route fires, pad ends 37-42 mm lower in object z.
