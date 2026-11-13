# Evidence: retreat_hands_radially_from_object

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## retreat_hands_radially_from_object(right_clear=0.26, left_clear=0.22, step=0.015, max_iter=14, move_dur=0.45, hold_dur=0.35, abort_z_up=0.99, abort_dz=0.004, abort_dxy=0.05)
- source hash: ed8fb9dd4fd773626d4dbcc2ab2413161f58e0edbbc874fafe027dc2f506ef04 (work/tracks/06_drill_pnp/actor_extract/skills/retreat_hands_radially_from_object.py)
- removes: retracting an open hand toward the body or the object's axis -- it brushes the seated object and nudges it (a measured 12 deg tilt after one such park).
- applies when: the object is released and both hands are open and near it.
- contract: pre: object seated -> post: no hand touching and each palm at least its clearance away, or budget/abort; the object is re-measured after each push and the retreat STOPS if it moves. Side effects: one object read per iteration plus the push pair.
- verified scope: 06_drill_pnp, drill, seeds 0-9: final hand contacts 0, table 4 on 8/8. Caveat: on seed 1 its precondition (a released object) was violated because the release failed, and it returned without signalling that -- it does not verify its own precondition.
- used by: 06_drill_pnp composition.
```
