# grasp_verify -- open before calling

Signature: grasp_verify(match='bottle', n_min=2, gap_min=0.040, **kw)
- source hash: e6d66d025681099ad1ec18645f298185d47526cafd0418455d87439c5a83cb61 (05_drill_lift/actor_5/skills/grasp_verify.py)
- removes: a post-close grasp check that would need a prediction. On 05 no pre-hoc quantity separates a lift from a close-on-air (pre-close n_fing 0 and gap 0.11-0.16 on every seed).
- applies when: called at the POST-CLOSE instant of a left-hand power grasp held with the v35 wrist orientation; object tag known (`match`); `api.contact_detail`, `api.hand_geoms`, `api.observe` available.
- contract: read-only, no command path. Returns `held` (1 iff >= `n_min` `gripper0` contact rows AND thumb_2-minus-index_1 world-z gap >= `gap_min`), plus n_fing, fmax (max single contact force), n_bc, gap, thumb2/index1 object-relative, c (rows), obj, t. Reads are `mj_forward` only; qpos/qvel and the command stream are untouched.
- verified scope: 05_drill_lift only, left-hand power grasp, post-close. Verifier re-ran 7 seeds at the pinned composition (drill_lift_v35 9ec69d48... WITH its `off{seat_mode:'branch', ...}` kwargs): 0,1,2,3,4,12 -> held=1, n_fing 3-5, fmax 15.578-25.239, gap 0.0683-0.0832; 7 -> held=0, n_fing 0, gap 0.0108 (never lifts). All exact to 4 dp against the actor's run. Seeds 5,6,8,9,10,11 are actor-reported only.
- used by: 05 drill_lift_v35/v39/v40 compositions (read-only gate).
- CAUTION: the `held` decision is geometry-specific and does NOT transfer. Measured on genuinely held objects: 07 right-hand box -> held=0 (gap -0.0266/-0.0263 with all grasp contacts on the right hand); 09 left-hand top grasp -> held=0 (gap -0.0301/-0.0308). Only the `gripper0` contact-row count transfers. Also: pre-close it always reports n_fing=0; `fmax` is a max-single-contact statistic, not a force sum; `n_bc` pools non-hand contacts of the matched body (`match='box'` also matches table-box rows); `limit=40` can truncate in cluttered scenes.

Proof history: `library/evidence/grasp_verify.md`.
