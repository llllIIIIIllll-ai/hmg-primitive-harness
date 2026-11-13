# Evidence: walk_base_to_coordinate

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## walk_base_to_coordinate(axis=0, target=0.05, v=0.18, lat=0.0, yaw=0.0, dur=0.5, max_bursts=8)
- source hash: 66f57455e45d293bb63e6b73f8423059b71e3d7482f07912db5e5e205496cd64 (work/tracks/08_pick_drill_from_holder/actor_extract/skills/walk_base_to_coordinate.py)
- removes: a base that stops short of the standoff it was sent to, leaving the arm out of reach before a grasp.
- applies when: the base coordinate along `axis` is measurable and the path is clear.
- contract: pre: base behind the target coordinate -> post: that coordinate driven to `target` (measured 0.099 against 0.05 in 6 bursts) or max_bursts spent; returns the base-state trace.
- verified scope: 08_pick_drill_from_holder seeds 3,4,5,6 (verifier) and 7 (coordinator held-out: composed equals promoted, engine success true, t 34.9 identical). Breaks if the base is blocked or the burst budget is exhausted first.
- used by: 08_pick_drill_from_holder composition.
```

## Gate C widening: 09_obstacle_aware_pick_drill (2026-11-10, verifier)
Source 66f57455... (catalogued). In cand_v15 2e59058e... on seeds 4,5,9,10 the
child is called with axis=0, target=measured+0.03, v=0.20, dur=0.5,
max_bursts=2. All 15 trace arrays are bit-identical to v14 37a9994d..., which
issues the raw move_base stream directly; t identical (43.72/82.82/71.52/
73.12). One burst of the two allowed; nread 2 (the child's own trace length);
base overshoots b0+0.046 for dx=0.03 and drifts 0.025-0.034 laterally.
Bit-identity is mechanically expected -- the corridor is clear and the
correction loop never engaged. Scope widened ONLY to this single-burst,
unblocked-base composition; multi-burst convergence and blocked-base behaviour
remain unverified on 09.
