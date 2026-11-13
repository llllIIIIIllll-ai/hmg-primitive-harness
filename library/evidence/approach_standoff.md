# Evidence: approach_standoff

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## approach_standoff(box=None, standoff=0.40, closer=0.0, max_it=16, guard=0.20, tilt_limit=25.0, tol_x=0.04, tol_y=0.06, tol_yaw=6.0, stop_x=0.05, vx_gain=0.7, vx_clip=(0.20,0.30), vy_gain=0.7, vy_clip=0.18)
- source hash: 20a6a542e9d4445e3461fcb22b1c1677a473c4bddae46ff91beebea2992c4b28 (work/tracks/03_box_lift/actor_extract/skills/approach_standoff.py)
- removes: grasping from wherever the base happens to stop -- a lateral/yaw offset at the standoff becomes a pad placement error at the faces, and a large pre-grasp reach shortfall drags the box instead of clamping it.
- applies when: the box pose is observable in the base frame, the base is roughly facing it, the ~0.40 m standoff is reachable, and lateral clearance exists.
- contract: pre: base at an arbitrary nearby pose, box at rest -> post: measured box-to-base standoff driven to `standoff` with |lateral| <= tol_y and |yaw| <= tol_yaw, base tilt under tilt_limit, within max_it correction steps; returns a walk log (7 steps on every measured seed). Side effects: one scene read plus one base command per step.
- verified scope: 03_box_lift table-top box, seeds 16,17,18,19,20 (extraction, verifier gate A, coordinator held-out 20 -- composed equals promoted with 0 of 286 compared leaves differing, max delta 0.0, identical t). The `walk_tilt` and `no_object` exits were NOT exercised; `fingertip_guard` cannot be excluded because compose does not abort on it. No cross-task evidence.
- used by: 03_box_lift composition (first step).
```
