# grasp_lift_floor_box_press_branch -- open before calling

Signature: run(api, **kw), 80 keys. The 70 keys of
`grasp_lift_floor_box_branch` (approach_x=0.67, h=0.46, lift_h, press_ncmd=4,
seated_tol=0.0, branch='off', branch_min_cmd=0, repair_n=0, repair_wide_add=0.0,
repair_back_add=0.02, retry_n, retry_back_add, y_wide=0.10, x_back=0.05, ...)
plus ten press-screen keys: press_branch='off', press_ratio, press_clear_tol,
press_travel, press_err_tol, press_dq_tol, press_switch_corr, press_min_cmd,
press_alphas, press_fallback (each defaults to the matching free-space
`branch_*` value).
- source hash: 49a138de785004a4b67aeac56905bc5c7647fa00bc0252b96f154210d2743b8c
  (01_box_lift_floor/actor_8/skills/grasp_lift_floor_box_branch_press_a8c.py,
  23934 B, 80 kw.get keys plus `att_tilt_dz`=0.05 and `att_retry_reasons`=('roll_away',)).
- supersedes, in order: `grasp_lift_floor_box_press_branch` dc56eb8b…ba20 (21744 B,
  the press screen on top of `grasp_lift_floor_box_branch` 3eacdc60…5fe3c,
  19116 B). This version adds ONE decision to dc56eb8b: the reach attitude guard
  retries a completed approach iff the reason is in `att_retry_reasons` (default
  roll_away) OR the reported pad normal drop `-dz >= att_tilt_dz` (default
  0.05 m), instead of retrying on any `down_tilt` label. It is NOT bit-identical
  to dc56eb8b on the down_tilt firing seeds (that is the change); seeds with no
  down_tilt fire are bit-identical (s0 verified over 1061 steps, all 15 leaves),
  and s60 -- the one load-bearing fire, dz -0.088 m -- is bit-identical too
  because its retry still executes.
- removes: (i) the press-stage failure of `grasp_lift_floor_box_branch` where a
  press-side closed-loop correction re-solves onto IK seed 2 and sweeps a pad
  through the box. The screen is the SAME pre-command test `place_pads_branch`
  already applies in free space, run on the press command stream: a command
  whose predicted pad-face clearance is < -0.04 m is refused before it is sent,
  and the stage ends at the last sent pose (`press_failed_at`). (ii) the
  attitude-guard damage where a `down_tilt` fire on a COMPLETED approach
  (retract + full re-solve from wide_hi) drags the box, when the pad normal
  barely moved: measured dz -0.012/-0.030/-0.010 m on s33/66/74. The guard's own
  dz separates those from the load-bearing fire on s60 (dz -0.088 m).
- applies when: box at rest on the floor within reach of a standing stance at
  `approach_x`; one env, one episode. Ship call:
  `{"branch":"refuse","branch_min_cmd":1,"repair_n":1,"repair_wide_add":0.0,
  "repair_back_add":0.02,"press_ncmd":4,"seated_tol":0.0,"press_branch":"refuse",
  "press_ratio":1000000000.0,"press_switch_corr":0,"press_travel":1000000000.0}`
  -- i.e. keep the clearance test, disable the ratio, travel and switch tests.
- contract: pre: box at rest -> post: pads seated, fingers closed, body standing
  to lift_h holding the box; the same return dict as
  `grasp_lift_floor_box_branch` plus `press_failed_at`, `branch_refusals` (and
  the press refusals in the same `branch_refusals` list). Default
  `press_branch='off'` leaves the press path bit-identical to the superseded
  skill.
- verified scope: 01 at the pinned bytes and the ship kwargs above. Engine TRUE
  78/81 over seeds 0-80 -- TRUE on every seed except 28, 32 and 61 (FALSE). That
  includes the earlier pin's failures 29 and 33 (repaired by the press screen),
  66 and 74 (repaired by the dz-gated retry: 0.5546 -> 0.8860 and 0.2764 ->
  0.8798 end z), and 41-60 at 20/20. The three residuals are press-stage or
  free-space failures: s28 (box shoved in free space before the press stage),
  s32 (free-space placement), s61 (in-contact lateral retract drags the box).
- gate B (verifier): on the pin dc56eb8b, seeds 0/6/20/29/33 -- ownership PASS
  (31 upper + 3 nav + 1 height only; free base and object dofs max|tau| 0.0);
  torque PASS against forcerange (zero over-limit in 55 dofs; at-cap counts fall
  on the firing seeds, 34 -> 1 at s29, 100 -> 47 at s33); determinism PASS. On
  this version, seeds 0/33/60/66/74 -- ownership PASS; torque PASS with the
  delta test `new_in_cand = []` on all five seeds, i.e. NO at-cap or over-limit
  reading the control lacks (candidate at-cap/over set sizes 6/3/1/3/3 vs
  control 6/65/1/92/6); determinism PASS (15/15 leaves byte-identical on two
  reruns); delta identity PASS (s0 and s60 bit-identical, s33/66/74 differ only
  by the skipped retry row, at dz -0.012/-0.030/-0.010 m).
- used by: 01 only. No cross-task scope is claimed (gate C not attempted).
- CAUTION: (1) The pre-command prediction is MISCALIBRATED, and the skill is an
  early-stop heuristic, not a penetration detector: on s29 the refusal fired at
  a predicted -0.0435 m where the achieved clearance was +0.0702 m, while the
  commands that actually shoved the box (s29 +0.0395 predicted, -0.2406
  achieved; s32 +0.0262 vs -0.2573) were predicted safe. Do not read a
  `branch_refusals` entry as evidence of contact. (2) On the firing seeds the
  screen REMOVES API reads (net -8/-2/-14/-24 at seeds 6/20/29/33); on a
  non-firing seed it adds scratch `ik_preview` reads (+9 at seed 0) that never
  step the sim, so seed 0 is trajectory-bit-identical but not read-identical.
  (3) One at-cap (25.0 Nm, dof 35 right shoulder_pitch, step 745) appears on s33
  after the two arms have already diverged; nothing is over-limit. (4) s28 is
  out of scope for any press-side fix (the box is already shoved in free space
  before the press stage), and s32 remains a free-space failure. (5) The dz gate
  is fitted on FOUR observed fires over 0-80 (dz -0.010/-0.012/-0.030/-0.088 m),
  so the 0.05 m threshold sits in a gap, not in a distribution: a seed whose
  small-dz down_tilt is load-bearing would lose its retry and fail. Do not
  extrapolate the gate above seed 80 without re-measuring the fires. (6) The
  skill carries over-limit `left_wrist_pitch` readings (max 5.35-5.90 Nm against
  a 5.0 Nm URDF effort, ~13-20 steps per seed); they are NOT introduced here --
  the pin has the identical readings at the same steps and additionally exceeds
  at the wrist yaw and the waist/shoulder set on s33/66/74. Do not write
  "limit-clean"; write "no new at-cap or over-limit reading relative to
  dc56eb8b".

Proof history: `library/evidence/grasp_lift_floor_box_press_branch.md`.
