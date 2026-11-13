# Evidence: boost_base_height_holding_command

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## boost_base_height_holding_command(height=0.87, dur=1.6, side='left', obj_tag='drill')
- source hash: 5f4a8f8f403c3a56069f17333a6200e64baa4e63dd42a511784cb0532e3caa80 (work/tracks/08_pick_drill_from_holder/actor_extract/skills/boost_base_height_holding_command.py)
- removes: a held object that the arm cannot raise any further -- raise the BASE while the closed-hand command is frozen, so the object rises without a re-reach.
- applies when: a grasp is held and `api.commanded()` is not None (otherwise it returns raised False).
- contract: pre: grasp held with a captured command -> post: base height set to `height` with the 31-D command unchanged; returns `{raised, obj, z_up, base}`.
- verified scope: 08 seeds 4,5,6,7. This is what produces the final height (drill z 0.8126-0.8247) and success depends on it. Unverified without a non-None command.
- used by: 08_pick_drill_from_holder composition.

NOT catalogued for 08: `ramp_close_fingers` (a7ccef8a...1e73) executes its four stages (0.3/0.6/0.9/1.2 rad) on every seed, but it is NOT verified to produce a holding grasp -- on seed 3 it closed around a 20 mm-misaligned handle and the object left the hand during the extraction. Do not cite it as a grasp primitive; pair any grasp with a measured contact check (see `curl_fingers_onto_handle`).
```
