# Evidence: walk_to_first_contact

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## walk_to_first_contact(app_gap=0.80, vx=0.30, dur=1.0, max_bursts=6, ref_y=None, ky=1.0, kb=0.8, eb_soft=0.0, cy_max=0.08, kyw=1.0, cw_max=0.25, leader='shelf', phase='approach')
- source hash: c042fcd77b392010092c196fa6b8180412467fae06d42cd61184edaf8d35edca (work/tracks/04_push_shelf_forward/actor_extract/skills/walk_to_first_contact.py)
- removes: walking into a push face blind -- the base either stops short and never touches the cart or arrives with yaw/offset error and strikes it off-centre.
- applies when: the leader (cart) and the follower face are both observable, the follower command axis is the base +x, and a standoff `app_gap` can be measured along it.
- contract: pre: base behind the cart face at more than `app_gap` -> post: base advanced in velocity bursts until the measured gap reaches `app_gap`, with lateral/yaw feedback bounded by `cy_max`/`cw_max`; returns `{bursts, trace, contacts, ...}`. Side effects: per burst a `measure_scene_state` read plus one velocity burst, using `steer_behind_leader` for the lateral correction.
- verified scope: 04_push_shelf_forward -- same five seeds. Do not read `bursts` as convergence: the burst COUNT is returned for diagnosis.
- used by: 04_push_shelf_forward composition.
```
