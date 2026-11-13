# shoulder_lift_hold -- open before calling

Signature: shoulder_lift_hold(commanded=None, z_start=None, stand_height1=0.82, stand_height2=0.90, stand_dur=1.5, stand_dur2=1.5, settle_dur=0.5, target_rise=0.14, min_rise=0.02, contact_gate_n=3, contact_gate_rise=0.09, iters=8, pitch_step=0.06, ...)
- source hash: e73fda828005ec6c32681c6ac7561f87be8dd0c0a43e3ac4fc461e2f546b4d16 (03_box_lift/actor_extract/skills/shoulder_lift_hold.py)
- removes: a one-shot stand-up that drops the box -- the lift must be ratcheted in gated steps, because contacts transiently break during a rise (HAND_CONTACT_LOSS fired 19/9/20/10 times on seeds 16-19) without the grasp actually failing.
- applies when: the box is clamped with both hands, the arms can start from the captured command, and the target rise is within shoulder travel.
- contract: pre: two-palm clamp held, `commanded` is the grasping posture -> post: object raised in measured samples gated on a contact count (`contact_gate_n`) and a rise gate (target_rise); 5 samples on every seed, final object z 0.957-0.970; returns z series, contacts and the final object pose. Side effects: shoulder/stand commands plus the per-step object and contact reads.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence. The `abort` exit was never exercised. Contacts are transiently lost during the lift (do not read HAND_CONTACT_LOSS as grasp failure).
- used by: 03_box_lift composition.
- CAUTION: contacts transiently break during the rise -- do NOT read HAND_CONTACT_LOSS as grasp failure; the `abort` exit is unexercised.

Proof history: `library/evidence/shoulder_lift_hold.md`.
