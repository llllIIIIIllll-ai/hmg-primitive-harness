# Evidence: hold_parked_arms_and_measure_drift

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## hold_parked_arms_and_measure_drift(hold=2.0, right='right', left='left')
- source hash: 65e8d576fdeac7049d08177f61f7b2758294fafe97a386ea3fdf8de06021116a (work/tracks/06_drill_pnp/actor_extract/skills/hold_parked_arms_and_measure_drift.py)
- removes: declaring success at the instant of release, when the object is still settling or still touched by a retracting hand.
- applies when: the arms are parked clear and the caller wants the environment's own settling measured.
- contract: pre: arms clear -> post: `hold` seconds of a frozen posture elapsed, object measured twice, drift reported. Side effects: one hold step plus contact/base reads.
- verified scope: 06_drill_pnp, drill, seeds 0-9 (drift 0.0 on 8/9, 0.02498 m on seed 1 -- the measurement that exposes the incomplete release). MEASUREMENT-ONLY: in a box scene it is behaviourally IDENTICAL to an idle `step_goal` (obj_drift 0.0502 both), so it cannot attribute motion to its own arm; and its `table_contacts` counts by the literal body-name substring 'table', reporting 0 while 8 object-floor contacts existed. Read it as a waiter, never as evidence that the arms were still.
- used by: 06_drill_pnp composition.

NOT catalogued for 06, with reasons: `seat_gripped_object_on_support` (6e4034ce...36b9) never meets its own success criterion on any seed -- it exits by exhausting max_steps with table contacts 1, and the object seats only after release, so "seats the object" is NOT a verified claim; `raise_gripped_until_clear` (6046c2cc...230b) only ever took its immediate-exit path (object already above the clear margin 8/8), so its raise loop and grasp-degradation break are unmeasured; `nudge_wrist_joint` (724ab888...48cd) has no independent measurement of any effect (compose discards its return value); `raise_palm_holding_grip` (88414dd2...0127) was executed ZERO times on any seed.
```
