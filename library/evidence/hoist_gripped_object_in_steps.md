# Evidence: hoist_gripped_object_in_steps

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## hoist_gripped_object_in_steps(side='right', step=0.015, max_steps=3, left_max=0, right_min=1, z_up_min=0.85, dur=0.4)
- source hash: 98aae632d26e5e82f937eeca8b606bf440465840c4aac04f8c9b2b6b204a4d00 (work/tracks/06_drill_pnp/actor_extract/skills/hoist_gripped_object_in_steps.py)
- removes: one large lift that yanks the object out of the fingers, or a re-command from a stale posture that silently drops it.
- applies when: the fingers are closed on the object and `api.commanded()` holds that grasp.
- contract: pre: grasp held -> post: object raised up to max_steps*step, re-commanding the finger targets every step and measuring height/uprightness/contacts; aborts on grasp degradation. Side effects: per step one palm read, one reach, one object read, two contact scans.
- verified scope: 06_drill_pnp, drill, seeds 0-9 (full 3-step budget, `lost=False` everywhere). The three abort branches were NEVER exercised -- they are unmeasured code, not verified behaviour.
- used by: 06_drill_pnp composition.
```
