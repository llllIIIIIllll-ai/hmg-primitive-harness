# Evidence: arm_ramp

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## arm_ramp(side='left', target=None, joints=None, seg=None, seg_dur=0.30, max_step=0.25, hold=0.0, n_read=0, dt=0.20, tol=0.06, phase='arm_ramp')
- source hash: ab2746e89e7d10856173d8aaaeacc9bc6e1800ba5d4dddc5f383fc3aae9dd1ae (work/tracks/05_drill_lift/actor_4/skills/arm_ramp.py)
- removes: an arm stuck in the WRONG IK BRANCH of the same palm-pad site -- on 05 seed 1 the second pinch settles with j5 +0.4555 against +2.619 on the passing seeds (~2.16 rad) and the grip closes on air; transporting the joint vector to the passing branch fixes it, and the same seeds flip to failure with the transport disabled.
- applies when: the caller supplies the target joint sub-vector (the skill does NOT plan it), the arm joints are free to command, and a prior command is in force. It does not check balance, does not reject a target outside the joint range, and `n_read >= 4` raises AttributeError at this hash (a documented parameter is unusable; use `n_read=0`).
- contract: pre: target vector in hand, arm free -> post: the 31-D upper goal driven to `target` through `api._ramp`, split into `seg` segments or `ceil(max|delta|)/max_step`; returns `{achieved, err, max_err, tracked}`. Side effects: `upper`/`commanded` reads, the `_ramp` substeps, optional dwell reads and `hold`.
- verified scope: 05 seeds 0-3 (engine true for the composition) and the two unseen ik_seed-0 seeds 9,11 (engine true WITH the ramp, false without). Cross-task on 03_box_lift (gate C): residual 0.0453 rad (2.6 deg), tilt no worse than an idle hold, box displacement 0.0001, zero hand-object contacts. NOT verified as a target planner; NOT verified to respect joint limits (its own `br_arm` j5 2.6191 exceeds the 2.618 limit, so the successful configuration runs at the joint stop); and the claimed SEGMENTATION benefit is REFUTED -- seg=1 gives max_err 0.0339 `tracked 1`, seg=10 gives 0.0636 `tracked 0`. It does not fix unseen ik_seed-2 seeds.
- used by: 05_drill_lift `drill_lift_v18.py` branch mode.
- merge-or-justify: no existing entry takes a JOINT-VECTOR target (`move_held`, `servo_*` and `reach_*` plan a Cartesian pose, `nudge_wrist_joint` moves one joint), so this is not a parameterisation of an existing skill.

NOT catalogued for 05, with reasons: `pad_hold_audit` (4e807d3b...285f) is an AUDIT -- it reports a settled drift (measured 31.4 mm, not the claimed 6.3 mm) and corrects nothing; `base_place` (d3c5f6bf...287b), `pad_place` (0d0545b5...a7ba) and `pad_servo` (9e4e6c71...bd7c) are measured NON-REPAIRS (base moves ~1 cm per 9 cm commanded; pad_place never converges; pad_servo's settled error grew at gain 1.0 and oscillates at 1.8).
```
