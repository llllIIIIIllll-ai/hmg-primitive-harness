# walk_base_to_coordinate -- open before calling

Signature: walk_base_to_coordinate(axis=0, target=0.05, v=0.18, lat=0.0, yaw=0.0, dur=0.5, max_bursts=8)
- source hash: 66f57455e45d293bb63e6b73f8423059b71e3d7482f07912db5e5e205496cd64 (08_pick_drill_from_holder/actor_extract/skills/walk_base_to_coordinate.py)
- removes: a base that stops short of the standoff it was sent to, leaving the arm out of reach before a grasp.
- applies when: the base coordinate along `axis` is measurable and the path is clear.
- contract: pre: base behind the target coordinate -> post: that coordinate driven to `target` (measured 0.099 against 0.05 in 6 bursts) or max_bursts spent; returns the base-state trace.
- verified scope: 08_pick_drill_from_holder seeds 3,4,5,6 (verifier) and 7 (coordinator held-out: composed equals promoted, engine success true, t 34.9 identical). On 09_obstacle_aware_pick_drill seeds 4,5,9,10 it is the child of top_up_reach_at_standoff: ONE burst of 2 allowed, target = measured + 0.03, v 0.20, and its 15 trace arrays are bit-identical to the raw move_base it replaced (the correction loop never engaged). Breaks if the base is blocked or the burst budget is exhausted first.
- used by: 08_pick_drill_from_holder composition; top_up_reach_at_standoff (09).
- CAUTION: on 09 the verified bit-identity covers only the single-burst, unblocked-base case; multi-burst convergence and blocked-base behaviour were not exercised there, so do not cite the 09 evidence for those regimes.
- CAUTION: the convergence loop is ONE-DIRECTIONAL. It stops when `pos[axis] >= target` and every burst issues forward `x = v`, so it cannot reach a DECREASING coordinate and cannot express a pure lateral reposition (zero forward component). A task that needs to walk sideways to a smaller y is outside this skill's parameterisation, not a failure of it (09, 2026-11-11).

Proof history: `library/evidence/walk_base_to_coordinate.md`.
