# Evidence: hold_commanded_pose

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## hold_commanded_pose(dur)
- source hash: e7b50be051af0bd80e5890385105cd827a0c71cc4be2e9de75d89a297bd18168 (work/tracks/04_push_shelf_forward/actor_extract/skills/hold_commanded_pose.py)
- removes: a push measurement taken while the base is still coasting -- the cart's own travel after the last burst is part of the placement error.
- applies when: a pose is currently commanded and the caller needs the measured effect of leaving it alone for a known interval.
- contract: pre: a command is in force -> post: `dur` seconds elapse with that command held; returns `{t}`. Side effects: steps the sim with no new command.
- verified scope: 04_push_shelf_forward, same five seeds; no object-specific content, so it is the one 04 skill with no task-shaped parameter at all -- still, no cross-task measurement.
- used by: 04_push_shelf_forward composition (2 calls).
```
