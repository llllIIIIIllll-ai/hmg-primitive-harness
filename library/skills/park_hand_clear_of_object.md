# park_hand_clear_of_object -- open before calling

Signature: park_hand_clear_of_object(side='left', other='right', park=0.09, trigger=0.45, dz=0.02, dur=0.9, orientation='keep')
- source hash: 544c8cc87974b1f69a5a9faa4e06ea8c77101ad6deca266eb9644f0b49e525a6 (06_drill_pnp/actor_extract/skills/park_hand_clear_of_object.py)
- removes: an idle hand inside the object's contact shell being swept into the object during grasp/carry, pinning or tipping it.
- applies when: the object is observable, the `side` hand is not grasping, and the palm is within `trigger` of it.
- contract: pre: idle hand near the object -> post: palm moved ~`park` further out along the measured object->palm direction and `dz` up, fingers unchanged; no-op beyond trigger. Side effects: 5 reads in the moving branch, 3 in the no-op branch.
- verified scope: 06_drill_pnp, drill, seeds 0-9 (moved True, idle-side contacts 0, residual 1.8-2.0 cm, palm z +0.024). NARROWED BY CROSS-TASK MEASUREMENT: in a box-on-floor scene the default trigger is a NO-OP at 0.84 m, and when forced it leaves a 0.089 m residual with the palm ending 0.019-0.023 m LOWER despite dz=+0.02. Do not treat it as scene-general.
- used by: 06_drill_pnp composition.
- CAUTION: NOT scene-general: in a box-on-floor scene the default trigger is a NO-OP at 0.84 m and forcing it leaves 0.089 m residual.

Proof history: `library/evidence/park_hand_clear_of_object.md`.
