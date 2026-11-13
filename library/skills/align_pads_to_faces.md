# align_pads_to_faces -- open before calling

Signature: align_pads_to_faces(pads, off, iters=4, dur=1.8, x_margin=0.030, y_margin=0.0237, face_off=0.0217, freeze_clr=0.005, near_thr=0.020, capy_far=0.030, capy_near=0.012, cap_x=0.05, cap_z=0.04, done_x=0.030, done_y=0.010, done_z=0.012)
- source hash: 60c0a1fad0da51ff04f82d5cf302bf42e934d7fbdc0a9f1946bf423ce221c42d (03_box_lift/actor_extract/skills/align_pads_to_faces.py)
- removes: a pad that is near a face rather than on it -- a placement-sensitive side clamp slips at the first lift step when the pads are marginal.
- applies when: both pads are visible in the current view and the box has not moved since the reach.
- contract: pre: palms at opposed-face targets -> post: pads corrected onto the faces over `iters` measured iterations (4 on every seed), with per-axis caps and completion thresholds; returns per-iteration corrections. Side effects: per iteration a pad/face measurement plus an arm correction.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence. Requires both pads in view; the `object_lost` exit was never exercised.
- used by: 03_box_lift composition.
- CAUTION: requires BOTH pads in view; the `object_lost` exit is unexercised.

Proof history: `library/evidence/align_pads_to_faces.md`.
