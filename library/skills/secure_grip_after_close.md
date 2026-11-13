# secure_grip_after_close -- open before calling

Signature: secure_grip_after_close(box_ref=None, hold=None, press_records=None, min_contacts=4, empty_min=2, seated_tol=None, seated_min=-0.02, stand_deep=-0.013, squeeze_ncmd=2, ...)
- source hash: 387eddc9627b7a5a1293e7dd7db0f5824fbf38752192e3e2b094b6470bc44f0b (01_box_lift_floor/actor_5/skills/secure_grip_after_close.py)
- removes: a squeeze that DESTROYS an already-seated press -- when both pads are seated but the raw contact count is below `min_contacts`, the unconditional squeeze drags the hand deeper and the grip is lost (seed 13: press seated, close c2, squeeze fired, lift contacts all 0, engine false).
- applies when: the caller owns the close-and-squeeze phase and holds the last press record with `clear_m` per side; `hold` is the closed-hand target from the finger close.
- contract: pre: press records available -> post: the squeeze runs only when NOT both_seated (both `clear_m` in [`seated_min`, `seated_tol`]) and contacts < `min_contacts`; the empty-grip recovery runs only when contacts < `empty_min`; returns the log plus `both_seated`/`squeezed`/`recovered`. Side effects: adds no engine read and no command beyond the pinned path -- the guard can only REMOVE commands.
- verified scope: 01 box. With `seated_tol=0.0`: seed 13 flips engine FALSE (t 23.96) -> TRUE (t 21.62, lift contacts [4,4,4,4,5,5]) and seed 0 stays TRUE with a slightly lower z_hi (0.9003 -> 0.8847); 14 of 16 seeds are bit-identical in records AND command streams. Inside the band but unchanged: seeds 1,3,5,7,8 (their contacts were already above `min_contacts`, so no squeeze was ever commanded). With `seated_tol=None` it is byte-identical to the pinned behaviour and seed 13 still fails. The band is a pad-seat proxy calibrated on this box, not a grip predicate. It does NOT fix seed 20 (the press itself failed at step 0 there).
- used by: 01_box_lift_floor `grasp_lift_floor_box_guarded.py`.
- CAUTION: the guard only works with `seated_tol=0.0`; `None` restores the pinned failing behaviour; the band is a pad-seat proxy, not a grip predicate.

Proof history: `library/evidence/secure_grip_after_close.md`.
