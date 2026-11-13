# Evidence: shoulder_lift_hold

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## shoulder_lift_hold(commanded=None, z_start=None, stand_height1=0.82, stand_height2=0.90, stand_dur=1.5, stand_dur2=1.5, settle_dur=0.5, target_rise=0.14, min_rise=0.02, contact_gate_n=3, contact_gate_rise=0.09, iters=8, pitch_step=0.06, ...)
- source hash: e73fda828005ec6c32681c6ac7561f87be8dd0c0a43e3ac4fc461e2f546b4d16 (work/tracks/03_box_lift/actor_extract/skills/shoulder_lift_hold.py)
- removes: a one-shot stand-up that drops the box -- the lift must be ratcheted in gated steps, because contacts transiently break during a rise (HAND_CONTACT_LOSS fired 19/9/20/10 times on seeds 16-19) without the grasp actually failing.
- applies when: the box is clamped with both hands, the arms can start from the captured command, and the target rise is within shoulder travel.
- contract: pre: two-palm clamp held, `commanded` is the grasping posture -> post: object raised in measured samples gated on a contact count (`contact_gate_n`) and a rise gate (target_rise); 5 samples on every seed, final object z 0.957-0.970; returns z series, contacts and the final object pose. Side effects: shoulder/stand commands plus the per-step object and contact reads.
- verified scope: 03_box_lift, seeds 16-20, same 0-difference equivalence. The `abort` exit was never exercised. Contacts are transiently lost during the lift (do not read HAND_CONTACT_LOSS as grasp failure).
- used by: 03_box_lift composition.

NOT catalogued for 03: `retract_clear` (c5c200b6...40d2) is CARRIED at its pinned hash but was never invoked on seeds 16-20 -- it is reachable only through compose's `_recover` branch, which never fired (stub-substitution control with five positive firings confirmed the detector). Do not present it as a verified behaviour; it needs a seed that forces a >0.10 m reach shortfall or a failed close.
```
