# reach_palm_to_object_standoff -- open before calling

Signature: reach_palm_to_object_standoff(side='right', back=0.22, lateral=-0.046, z=0.86, dur=1.2, orientation='keep')
- source hash: a114fcf46afd9d92828b4a5b7ea3d98b5f26bbe9ac9172a650b9ca8b03625c1e (06_drill_pnp/actor_extract/skills/reach_palm_to_object_standoff.py)
- removes: aiming the palm at the object itself -- the hand reaches the grasp column and shoves the object before the fingers close.
- applies when: the object pose is measurable, the approach axis is roughly aligned with the base frame, and the standoff is not occupied.
- contract: pre: base at the measured stance -> post: palm commanded to (obj - back along the approach, lateral, z) with orientation held; returns `{target, residual, obj_after, z_up_after}`. Side effects: one observe plus the arm reach plus two object reads.
- verified scope: 06_drill_pnp seeds 0-9, drill on a table. NARROWED by measurement: the palm actually ends **0.0496-0.0802 m** from its commanded standoff (dominated by a 5-7 cm x shortfall), and compose does not gate on it -- so the postcondition is "reaches toward the standoff", not "arrives at it". Not verified outside an axis-aligned drill handle.
- used by: 06_drill_pnp composition.
- CAUTION: reaches TOWARD the standoff, not to it (ends 0.0496-0.0802 m off and compose does not gate) -- do not gate on arrival.

Proof history: `library/evidence/reach_palm_to_object_standoff.md`.
