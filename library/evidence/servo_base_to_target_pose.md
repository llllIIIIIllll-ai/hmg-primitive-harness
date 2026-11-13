# Evidence: servo_base_to_target_pose

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## servo_base_to_target_pose(x, y, yaw_deg, upper=None, tol=0.05, yaw_tol=6.0, max_iter=10, dur=0.5, vmax=0.26, min_speed=0.22, gain=1.5, yaw_gain=0.02, wz_max=0.4, trace_obj=False)
- source hash: 573c43cb68bcb55637d888d9c819c6833ea510d21ec9d1123d84f3bd28105d54 (work/tracks/06_drill_pnp/actor_extract/skills/servo_base_to_target_pose.py)
- removes: an open-loop base move to a randomized stance -- the error is only visible in the world frame, and the last few cm are below the velocity that moves the base (stiction), so a fixed command both misses and stalls.
- applies when: the base is holonomically commandable and the target is a world pose reachable without dragging a held object.
- contract: pre: live episode, `upper` None or a 31-D posture -> post: xy within tol and yaw within yaw_tol of the target, or max_iter windows spent; nav left zero. Side effects: per window one `base_state()` read and one base-only step.
- verified scope: 06_drill_pnp seeds 0-9 (drill carry) and 01_box_lift_floor seed 3 (cross-task): xy 0.016 m, yaw 0.24 deg, nav zeroed -- so the verified claim is **xy/yaw/nav only**. Breaks/unverified on base HEIGHT (fell 0.052 m in the box scene) and ATTITUDE (4.65 deg tilt); a held object pinning the base makes it plateau. No claim about posture holding with `upper=None`.
- used by: 06_drill_pnp composition, `carry_gripped_object_to_pose`.
```
