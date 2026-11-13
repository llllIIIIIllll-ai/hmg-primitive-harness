# Evidence: creep_to_object_standoff

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## creep_to_object_standoff(h_off=(-0.0266,-0.0003,0.1179), offset=(-0.083,0.047,0.012), side='left', tol=0.004, step=0.005, dur=0.35, iters=220, max_steps=40, move_tol=0.02, obj_tag='drill', holder_tag='holder_main')
- source hash: 9455427fed4ce9a3d83b893e6e5f79238a4642a4bb179f606fa5a5d4927f07cb (work/tracks/08_pick_drill_from_holder/actor_extract/skills/creep_to_object_standoff.py)
- removes: creeping to a grasp standoff while the object is being displaced unnoticed -- its measured value is the DRIFT GUARD it trips.
- applies when: object and holder poses are readable and small steps are affordable.
- contract: pre: hand near the standoff -> post: stepped toward the offset until tol, OR `moved` abort when the object's own displacement exceeds move_tol (20 mm), OR the max_steps cap; returns the row log.
- verified scope: 08 seeds 3-7. IMPORTANT: it NEVER reaches tolerance on any seed -- 4,5,6,7 all end on the 40-step cap with `stopped=None`. The verified behaviour is "abort if the object moves 20 mm", NOT "converge to the standoff". On seed 3 the guard fired at j=16 for a 20.1 mm drift, 0.1 mm over the trip, and the run then failed.
- used by: 08_pick_drill_from_holder composition.
```
