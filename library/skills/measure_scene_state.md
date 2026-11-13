# measure_scene_state -- open before calling

Signature: measure_scene_state(leader='shelf')
- source hash: e0e9cb205ed606baebabb422afe238ff6bf495e84840e6ad92b3ba67682ca198 (04_push_shelf_forward/actor_extract/skills/measure_scene_state.py)
- removes: a re-read that differs from the promoted program in WHICH engine reads it makes or in their order -- the failure signature is silent, not visible: two copies of one primitive differing only in diagnostic read blocks hold bit-identical commands through step 280 but diverge in physics from the first removed block and can flip the engine outcome at identical command time.
- applies when: a caller needs the scene state and wants the read pattern to stay pinned; it is the single entry point through which a composition should touch the scene.
- contract: pre: none -> post: `scene_objects()`, `geom()`, `observe()`, `base_state()` issued in that exact order, in the promoted count; returns the scene record. Side effects: 3 `mj_forward` forward passes plus one base read; commands nothing, so its only physical effect is the warm-start/derived-state refresh.
- verified scope: 04_push_shelf_forward -- seeds 3,4 (extraction, bit-identical), 5,6 (verifier gate A), 7 (coordinator held-out: 11/11 physical fields identical, t 16.2 = promoted). General beyond the task by construction (nothing in it is cart-specific) but that is an ARGUMENT, not a measurement: no cross-task run yet.
- used by: 04_push_shelf_forward composition (3 calls).
- CAUTION: use it as the ONLY scene entry point -- its exact read order and count are physical, and an equivalent-looking re-read can flip the outcome silently.

Proof history: `library/evidence/measure_scene_state.md`.
