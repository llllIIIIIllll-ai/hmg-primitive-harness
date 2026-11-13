# stand_lift_box -- open before calling

Signature: stand_lift_box(lift_h=0.68, lift_dur=2.5, sample_every=20)
- source hash: 156ea4bad3032f8b1ff4733a5b2df6c21d9d5594d698b92790ce8238924942f5 (01_box_lift_floor/actor_4/skills/stand_lift_box.py)
- removes: a base rise that also re-solves the arms -- the measured profile raises only the base height (nav zeroed) with `upper` frozen at `api.commanded()`, and the already-closed box rises with the body (dz +0.319/+0.330/+0.316 on seeds 3/4/5, +0.353/+0.271/+0.261 on 10/11/12, AABB top 0.823/0.834 on held-out 14/15).
- applies when: a grip is closed AND a prior command is in force. It does NOT guard `api.commanded() is None`: called first in a fresh episode it raises TypeError at t 2.5 (seed 10, engine false) and the response wrapper still reports `ok: true` while the episode reports `ok: false`.
- contract: pre: grip closed, last commanded upper pose is the grasp pose -> post: base height smoothstep from the measured height to `lift_h` over `lift_dur` at 0.02 s steps with nav=[0,0,0]; returns `{h_from, h_to, samples, log, commanded}`. Side effects: every 20th step one `observe` + one `geom_detail` + one `hand_contacts_side`; commands height and nav only, never the arms.
- verified scope: 01_box_lift_floor, floor box, ONLY as the terminal step after a close (seeds 3,4,5 verifier; 10,11,12 gate A; 14,15 coordinator held-out). It is not verified standalone -- the precondition is real and unguarded. The lift is load-bearing: with the close removed the box does not rise at all. No cross-task evidence.
- used by: 01_box_lift_floor composition.
- CAUTION: unguarded precondition: called with `commanded() is None` it raises while the RESPONSE still says `ok: true` (episode says false); use only as the terminal step after a close.

Proof history: `library/evidence/stand_lift_box.md`.
