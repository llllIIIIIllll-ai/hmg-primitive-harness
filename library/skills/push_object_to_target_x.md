# push_object_to_target_x -- open before calling

Signature: push_object_to_target_x(target_x=None, vx=0.25, dur=1.0, lead=0.55, max_bursts=18, settle_dur=0.0, ref_y=None, ky=1.0, kb=0.8, eb_soft=0.0, cy_max=0.08, ...)
- source hash: 7e095675660dfe9a21cdf8bdab9d3acac5ee642a38fcc95721587104cda1860f (04_push_shelf_forward/actor_extract/skills/push_object_to_target_x.py)
- removes: an open-loop push that keeps rolling past the target or drifts sideways off the face -- contact-free bursts with a bounded lateral lead, plus a settle that exploits the cart's measured coast.
- applies when: a rolling object can be pushed from behind by the base at a face the base can reach, and its x position is measurable each burst.
- contract: pre: base already in contact with the face (call after walk_to_first_contact) -> post: object x driven to `target_x` in bursts of `vx` with lateral/yaw feedback bounded by `cy_max`/`cw_max`, stopping on the measured x or on `max_bursts`; returns `{bursts, trace, contacts, ...}`. Side effects: per burst one scene read plus one velocity burst; optional settle.
- verified scope: 04_push_shelf_forward -- same five seeds; the composed program also calls it for a fine push. Measured: the fine mode ran ZERO bursts on seeds 3,4 but ONE on seed 7 (composition census `{'approach':3,'fast':8,'slow':4,'fine':1}`), so do not claim a verified fine push -- the fine call is mostly one scene read.
- used by: 04_push_shelf_forward composition (3 calls: fast, slow, fine).
- CAUTION: the FINE mode is NOT verified (0 bursts on seeds 3,4 and 1 on 7).

Proof history: `library/evidence/push_object_to_target_x.md`.
