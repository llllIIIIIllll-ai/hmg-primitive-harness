# boost_base_height_holding_command -- open before calling

Signature: boost_base_height_holding_command(height=0.87, dur=1.6, side='left', obj_tag='drill')
- source hash: 5f4a8f8f403c3a56069f17333a6200e64baa4e63dd42a511784cb0532e3caa80 (08_pick_drill_from_holder/actor_extract/skills/boost_base_height_holding_command.py)
- removes: a held object that the arm cannot raise any further -- raise the BASE while the closed-hand command is frozen, so the object rises without a re-reach.
- applies when: a grasp is held and `api.commanded()` is not None (otherwise it returns raised False).
- contract: pre: grasp held with a captured command -> post: base height set to `height` with the 31-D command unchanged; returns `{raised, obj, z_up, base}`.
- verified scope: 08 seeds 4,5,6,7. This is what produces the final height (drill z 0.8126-0.8247) and success depends on it. Unverified without a non-None command.
- used by: 08_pick_drill_from_holder composition.
- CAUTION: requires a non-None `api.commanded()` (otherwise `raised` is False).

Proof history: `library/evidence/boost_base_height_holding_command.md`.
