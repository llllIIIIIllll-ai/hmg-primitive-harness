# Evidence: steer_behind_leader

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## steer_behind_leader(kwargs: ref_y, ky, kb, eb_soft, cy_max, kyw, cw_max, leader)
- source hash: 59b0fc2c568fe18d96c375274ea8a69889ea5663c7e2ef8282dc0ef08f5cf39a (work/tracks/04_push_shelf_forward/actor_extract/skills/steer_behind_leader.py)
- removes: the cart veering away from the face line -- open-loop bursts accumulate lateral error until the base is beside the cart instead of behind it.
- applies when: a leader object and a follower face are measurable in the base frame, and the caller wants a bounded single-axis correction rather than a full walk.
- contract: pre: leader/follower poses readable -> post: a lateral velocity and yaw rate bounded by `cy_max`/`cw_max` computed from the lateral and heading error, returned as `{y, yaw, ey, eb}`; commands nothing itself. Side effects: reads only.
- verified scope: 04_push_shelf_forward -- exercised only through `walk_to_first_contact`; no direct invocation on any seed, so its contract is measured as a component, not standalone. Do not catalogue a standalone success claim for it.
- used by: walk_to_first_contact and push_object_to_target_x (04) -- both call it once per burst.
```
