# Evidence: walk_to_reach

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## walk_to_reach(target_match='button', gapx_max=0.44, phases=((0.3,0.0,1.2),(0.3,0.05,0.5),(0.25,0.02,0.4)), burst_x=0.22, burst_y=0.02, burst_dur=0.4, max_extra=6, axis=0, joint_index=0)
- source hash: c1c034a8caad49d007aa148980344b578be0503996be8470e87251b686411bd3 (work/tracks/02_push_button/actor_extract/skills/walk_to_reach.py)
- removes: open-loop base walking that stops short of the object and leaves the gap outside arm reach (measured post-walk joint-to-base gaps 0.530/0.247/0.398/0.312/0.305 m on seeds 1,2,3,5,7).
- applies when: the object is observable as a named SceneObject with a readable joint position, the base is holonomically commandable, and the approach is along one base axis (`axis`), with the standoff budget given as `gapx_max`.
- contract: pre: object pose readable at the current base pose -> post: measured object-joint-to-base gap driven at or below `gapx_max` with lateral error bounded by `burst_y`, in a scripted coarse-then-fine burst sequence, or the extra-burst budget spent; returns `{target_pos, target_q, trace}`. Side effects: one object-joint read per burst, base-only nav steps, nav left at zero.
- verified scope: 02_push_button only -- seeds 1,2,3,5,7 (extraction, bit-identical to promoted), 11,12,13 (verifier gate A, 0 differing leaves), 21,31,37 (coordinator held-out, 0 differing leaves). The gap postcondition is NOT met on seed 1 (0.530 m against a 0.44 m gate) and the press still succeeds: the gate is a planning budget, not a precondition the caller may assume. No cross-task (gate C) evidence; breaks when the object has no readable joint or the approach is not axis-aligned.
- used by: 02_push_button composition.
```
