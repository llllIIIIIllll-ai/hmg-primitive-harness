# Evidence: press_until_joint_stop

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## press_until_joint_stop(side='right', target0=(0,0,0), step=(0.003,0,0), max_steps=26, q_stop=-0.0145, object_match='button', joint_index=0, servo_gain=0.7, servo_dur=0.45, servo_iters=2, object_token=..., support_token=..., contact_limit=60, settle_dur=0.6)
- source hash: 4da49591fd7ac96c0069702999dddfdf1800f00ab6e8269b8fa22447831fa2f0 (work/tracks/02_push_button/actor_extract/skills/press_until_joint_stop.py)
- removes: pressing a compliant object for a fixed time or distance -- the button's own slide joint moves from 0 only at the first contact step (measured j=17/17/18/17/17), so a blind press either wastes steps or over-drives into the panel.
- applies when: the target object has a readable joint whose motion measures the press, and there is a fingertip site the caller can servo between steps.
- contract: pre: fingertip at `target0`, object joint at rest -> post: object joint advanced by stepping the fingertip inward `step` at a time, terminated by the joint reaching `q_stop` or by `max_steps`; returns `{button_q, button_pos, ...}`. Side effects: per step a 2-iteration fingertip servo (its own reads) plus object-joint and contact-count reads, then one settle.
- verified scope: 02_push_button -- same ten seeds; the `q_stop` branch is NOT reached at the default -0.0145 (the joint settles at -0.0139..-0.0142 and `max_steps` ends the press) but IS reachable at q_stop=-0.0130 (24 and 22 `press_end` records at t 14.98/14.14, q -0.0137/-0.0136, still in contact). Push depth is compliance-capped: a 34-step press peaks at -0.0143/-0.0144 and then RECEDES to -0.0138, i.e. pressing longer does not press deeper. No cross-task evidence.
- used by: 02_push_button composition.
```
