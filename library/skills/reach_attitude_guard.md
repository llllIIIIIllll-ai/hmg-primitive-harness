# reach_attitude_guard -- open before calling

Signature: reach_attitude_guard(records=None, delta_ztol=0.10, level_ztol=0.12)
- source hash: 607e1f195a0eeeadd58d6e7ef179f0fc586ee43ac9350c17045e6a19b4f4bad2 (01_box_lift_floor/actor_5/skills/reach_attitude_guard.py)
- removes: a free-space reach that ends with a palm pad ROLLED AWAY from its face, which the following press then slides up and over instead of seating on it (seed 6: right-pad normal-z 0.028 -> -0.183 over the last free-run step).
- applies when: the caller holds the record list returned by a two-pad free-space reach whose records carry per-side pad normals. It makes NO engine call of its own.
- contract: pre: those records -> post: `{retry, reason, side, dz, z_first, z_last, samples, t0, t1}`; `retry` is true iff some side trips roll_away (normal-z drop > `delta_ztol`) or down_tilt (last value < -`level_ztol`). Side effects: NONE -- the `api` parameter is never used; it is a pure selection over caller data.
- verified scope: 01 box, seeds 0-21. Fires on seed 6 only (`attitude_retries` 1, 0 on every other seed); with it disabled seed 6 is engine FALSE (t 17.68, z_hi 0.3005) and with it enabled TRUE (t 23.22). Non-firing records appear on seeds 0,3,8,13; on seed 6 the stored `reach_attitude` field is None, so count firings from `attitude_retries`, not from that field. Masking is impossible: a firing candidate's dz is strictly more negative than any non-firing one. It triggers a retry; it never moves anything itself.
- used by: 01_box_lift_floor `grasp_lift_floor_box_guarded.py`.
- CAUTION: count firings from `attitude_retries` -- the `reach_attitude` field is None on the seed where it fires; it never moves anything itself.

Proof history: `library/evidence/reach_attitude_guard.md`.
