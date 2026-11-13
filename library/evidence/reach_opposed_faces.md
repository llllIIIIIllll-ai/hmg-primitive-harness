# Evidence: reach_opposed_faces

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## reach_opposed_faces(box=None, gap=0.09, x_off=-0.15, dz=-0.02, dur=2.2, ik_err_max=0.09, via_dur=1.0, retry_dur=1.5)
- source hash: ab501ee38d0f9b8e6e96bcf21832eb45671ddcd8fbdb689ce6e07592f1545639 (work/tracks/03_box_lift/actor_extract/skills/reach_opposed_faces.py)
- removes: reaching both palms to a box with an IK solution that misses the faces -- the error is only visible after the reach, and a big shortfall turns the grasp into a drag.
- applies when: the box is at the standoff with its faces roughly opposed to the palms, and a two-palm IK solution exists within ik_err_max.
- contract: pre: base at the measured standoff -> post: both palms commanded to opposed-face targets with per-side IK error below ik_err_max (measured 0.0609/0.0735 and 0.0615/0.0721 against a 0.09 guard); returns per-side IK errors and a `via_retry` record. Side effects: arm reaches plus the reads that build the targets.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence as above. NARROWED: `via_retry` was empty on every seed and `no_object`/`no_pad` were never observed, so the retry path for a large reach shortfall is UNMEASURED -- do not rely on it.
- used by: 03_box_lift composition.
```
