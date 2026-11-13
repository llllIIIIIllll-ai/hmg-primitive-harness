# Evidence: limit_safe_close

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## limit_safe_close(grip_angle=1.0, grip_dur=1.2, margin=0.02, min_contacts=2, release_on_fail=True, release_angle=-0.5, release_dur=0.6)
- source hash: 4566cf64936bb4b3c70981926b4f99dd7c16e51e9425451e466f6921d48ecbb8 (work/tracks/03_box_lift/actor_extract/skills/limit_safe_close.py)
- removes: closing the fingers to a fixed angle and assuming a grasp -- a marginal grasp then survives the close and fails at the first lift step.
- applies when: the pads are on the faces and the hand-joint limit magnitude is known (measured 1.0472 against a commanded 1.0, so the command is NOT clamped).
- contract: pre: pads aligned on the faces -> post: fingers closed to grip_angle and verified by a measured hand-object contact count >= min_contacts (measured 4 and 9 at close), with release-on-fail available; returns `closed` and contacts. Side effects: one close command plus contact scans.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence. Contact counts vary 3-9 across seeds, so the min_contacts=2 threshold has little margin evidence; the not-closed recovery path was never taken.
- used by: 03_box_lift composition.
```
