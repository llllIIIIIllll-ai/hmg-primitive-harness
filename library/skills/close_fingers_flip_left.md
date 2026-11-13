# close_fingers_flip_left -- open before calling

Signature: close_fingers_flip_left(angle=0.9, dur=0.8, left_finger_flip=1, rthumb_min=0.30, box_ref=None)
- source hash: 306ed649c99f9d370dc8e638e562cbcb87127669c61980bf1b32a042bd106024 (01_box_lift_floor/actor_4/skills/close_fingers_flip_left.py)
- removes: an OPEN hand at lift time -- measured, not the flip: a no-close control on seed 10 leaves the box at dz 0.0000 and engine success false at t 16.42, while every closed variant lifts it.
- applies when: both pads are on the opposed faces and a valid upper-body command is in force; the caller owns the lift.
- contract: pre: pads seated -> post: one smoothstep ramp (`dur` s at 0.02 s steps) of the hand joints to +-`angle` clipped to each joint range, left index/middle negated when `left_finger_flip` is truthy; returns `{target, finger_cmd, contacts, sides, rthumb_cmd, moved_l1_m, record}`. Commands hands only. Side effects: `upper` and `joint_ranges` queries, the ramp, then observe -> hand_contacts_side -> hand_contacts_side -> observe; this exact read order is part of the verified behaviour.
- verified scope: 01_box_lift_floor, floor box, seeds 3,4,5,10,11,12. `left_finger_flip` is INERT for pass/fail (flip 0 vs 1: identical categorical set on every seed; seeds 11,12 engine t identical at 17.28/17.62; only seed 10 moves, 18.02 -> 19.96). It does NOT remove a right-side-only grip: at pn2 seed 5 the close is c3 L0/R3 with the flip ON as well as off. It shifts post-squeeze side counts and drag. A one-sided 1-2 contact close is sufficient for the task. No cross-task evidence.
- used by: 01_box_lift_floor composition.
- CAUTION: `left_finger_flip` is inert for pass/fail and does not fix a right-only grip.

Proof history: `library/evidence/close_fingers_flip_left.md`.
