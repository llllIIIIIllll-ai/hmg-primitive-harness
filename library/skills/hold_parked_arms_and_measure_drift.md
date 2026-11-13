# hold_parked_arms_and_measure_drift -- open before calling

Signature: hold_parked_arms_and_measure_drift(hold=2.0, right='right', left='left')
- source hash: 65e8d576fdeac7049d08177f61f7b2758294fafe97a386ea3fdf8de06021116a (06_drill_pnp/actor_extract/skills/hold_parked_arms_and_measure_drift.py)
- removes: declaring success at the instant of release, when the object is still settling or still touched by a retracting hand.
- applies when: the arms are parked clear and the caller wants the environment's own settling measured.
- contract: pre: arms clear -> post: `hold` seconds of a frozen posture elapsed, object measured twice, drift reported. Side effects: one hold step plus contact/base reads.
- verified scope: 06_drill_pnp, drill, seeds 0-9 (drift 0.0 on 8/9, 0.02498 m on seed 1 -- the measurement that exposes the incomplete release). MEASUREMENT-ONLY: in a box scene it is behaviourally IDENTICAL to an idle `step_goal` (obj_drift 0.0502 both), so it cannot attribute motion to its own arm; and its `table_contacts` counts by the literal body-name substring 'table', reporting 0 while 8 object-floor contacts existed. Read it as a waiter, never as evidence that the arms were still.
- used by: 06_drill_pnp composition.
- CAUTION: MEASUREMENT-ONLY -- behaviourally identical to an idle `step_goal`; never cite it as evidence the arms were still.

Proof history: `library/evidence/hold_parked_arms_and_measure_drift.md`.
