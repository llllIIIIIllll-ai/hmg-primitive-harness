# lift_drill_pad_route_geom_close -- open before calling

Signature: run(api, **kw); the only key is `off`, a dict of 15 knobs. Ship value:
`off = {"br_gate":1,"br_on":1,"close_gate_nf":1,"gate":1,"pin1_ramp":0,
"pr_m":0.002,"pr_on":1,"pre_stance":1,"ps_on":0,"seat":1,"seat_mode":"branch",
"servo_on":0,"tw":0,"close_key":"geom","geom_thumb2_y":-0.055}` (absent keys
default as in the source; `close_key` absent == "nf").
- source hash: 3fd440a55ccbbcdb5c274203617c907f2c7c15af3566ea0fed36b7874a8855e8
  (05_drill_lift/actor_7/programs/drill_t_r2free.py, 95057 B). Its parent, the
  same program without one `_route` insertion, is drill_t
  86fdd43fe876062a41b953ce12e9695c86598088e7e361214bc5de556d88ebbe (94006 B).
- removes: two failure signatures on a floor drill. (1) The `|j5|` stance label
  vetoes the transport the seat stage needs; a 2 mm-expanded hull test on the
  measured post-stance pad, evaluated in the seat frame, overrides the veto.
  (2) The close plan is chosen on a finger-contact count that reads 0 when the
  pad has closed on air; the plan is instead selected by
  `max(n_fing >= 1, thumb2_y <= -0.055)`, where thumb2_y is the pre-close
  thumb_2 world-y offset from the object root, measured by the existing
  `grasp_verify` probe.
- applies when: drill at rest on the floor, reachable from the stance the program
  builds; commanded only through the 31 upper joints + 3 nav + 1 base height.
  The close key is safe when the pre-close thumb2_y is above -0.0415 m; the
  route hull is x [-0.0656, 0.0415], y [0.0454, 0.0862], z [0.2324, 0.2530] in
  the seat frame.
- contract: pre: drill at rest -> post: drill lifted, final object z 0.914-0.953 m
  and stop stage in {2,3,4} on passing seeds. Returns the stage log, `br`
  (route), `close_key`, `seat_pre/post` and the lift rows. No added API read on
  top of drill_c7: the close key reads `log['gv_preclose']`, which the parent
  already records.
- verified scope: 05 at the pinned bytes and kwargs. Engine TRUE on 20 of 21
  seeds 0-20 (FALSE on 20); on 0-12 it is bit-identical to drill_c7 in all 15
  recorded trace leaves (the five seeds the composition changes are 13, 15, 16,
  17 and 20).
  The unseen extension 21-40 at the same bytes/kwargs is 18/20 (FALSE on 26, 32)
  and a coordinator held-out 41-60 is 18/20 (FALSE on 45, 46), so 36/40 above 20
  and 56/61 over 0-60; the verified scope is 0-20 and a seed above 20 is not
  covered. Gate B: ownership PASS, determinism PASS (15/15 trace leaves, two
  reruns), torque PASS on the physical denominator (hand servos are force-limited
  at 1.4 Nm, thumb_0 2.45; nothing exceeds its limit).
- used by: 05 only. Gate C: the close key's ORDERING transfers to 06
  (3 FALSE vs 3 TRUE seeds separated by 10.8-14.7 mm in three frames) but the
  threshold does not port (all 06/08 values lie in 0..34 mm, so -0.055 is
  inert); it is falsified on 08 (ranges overlap, s17 FALSE at 12.4 mm against
  s0 TRUE at 20.2 mm). No cross-task scope is claimed.
- CAUTION: (1) s20 fails with the route active and the branch authoritative
  (post-transport pad x +0.0432 m, 2.9 mm of motion where >=72 mm was needed);
  the [-0.044,-0.029] band that describes the 0-20 movers is NOT general (13 of
  16 TRUE seeds on 21-40 lie outside it). (2) On routed seeds thumb2_y is
  plan-conditional (s20 reads -0.0637 routed, -0.0189 unrouted), so its clearance
  is a property of the plan; the reliable margin is the pass side (s2, -0.0065,
  48.5 mm). (3) The thumb-first branch rides the left-hand force cap at s16
  (69 of 800 steps at 1.400 Nm; the parent is at 0.58 Nm there) and raises
  right-hand thumb_1 from 0.64 to 0.79 of its limit (the parent reads 0.41 of
  its limit at that seed); the sustained left-thumb cap-riding at s0/s7
  (66/650 and 53/677 steps) is inherited byte-identically. (4) Inherited from drill_c7: commanded targets up to
  0.0509 rad outside the model range on left_shoulder_yaw and measured-q limit
  events at t 4.84-4.96; lower-body and waist torques are clean. (5) The
  "outcome is predicted by how far the ramp displaces the object" statement in
  the track is a fitted description of the 23 routed episodes measured AFTER the
  ramp (TRUE displacement <= 11.4 mm vs 47.2/55.2/53.8 mm on the three FALSE;
  pad height TRUE >= 0.2384 m vs FALSE <= 0.2211 m, a 17.3 mm gap), not a
  pre-command signal -- do not gate on it.

Proof history: `library/evidence/lift_drill_pad_route_geom_close.md`.
