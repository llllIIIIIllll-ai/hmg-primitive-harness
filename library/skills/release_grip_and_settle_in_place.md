# release_grip_and_settle_in_place -- open before calling

Signature: release_grip_and_settle_in_place(open_angle=0.02, open_dur=0.6, settle_dur=1.0)
- source hash: c8eada3fe735ac932e4bd133a412363597acc0d17b954c234c8c2beb3320c15e (06_drill_pnp/actor_extract/skills/release_grip_and_settle_in_place.py)
- removes: opening the fingers while still moving the palm -- the freshly seated object is pushed and rotated off vertical.
- applies when: the object is seated and still gripped with the seated posture commanded.
- contract: pre: object bearing on the support -> post: fingers at open_angle, then the whole posture held still for settle_dur with no palm motion; settled contacts and pose measured. Side effects: one close command plus one hold step plus contact/pose reads.
- verified scope: 06_drill_pnp, drill, seeds 0-9. Detaches on 7 of 8 measured seeds (table 4, hand 0, z_up 1.0, object z 0.7995); on seed 1 it does NOT detach (hand 3, table 0, z_up 0.9337) and the object only lands 0.025 m later during retreat+hold. Breaks when the fingers are wedged so opening necessarily drags the object.
- used by: 06_drill_pnp composition.
- CAUTION: on seed 1 it does NOT detach; the object lands 0.025 m later during retreat+hold.

Proof history: `library/evidence/release_grip_and_settle_in_place.md`.
