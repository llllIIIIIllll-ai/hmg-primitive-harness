# servo_fingertip_to_point -- open before calling

Signature: servo_fingertip_to_point(side='right', target=(0,0,0), gain=0.7, dur=0.45, max_iters=8, tol=0.008, tip_link='index_1', mesh_tag='_vis', watch_match=None, watch_joint=0)
- source hash: 8e04049e134310b26370b9a28c8efcdb6dfecdf00f3f0f2b1bbb0cbfb44f70d1 (02_push_button/actor_extract/skills/servo_fingertip_to_point.py)
- removes: open-loop fingertip placement, which misses by 0.080-0.210 m -- the whole reach error the solution this replaces could not remove.
- applies when: the fingertip site is readable, the target point is expressed in the same frame the site is reported in, and the arm has an IK solution that keeps the palm clear.
- contract: pre: measured fingertip error above `tol` -> post: fingertip within `tol` (default 8 mm) of `target`, or `max_iters` spent; returns `{tip, err, ...}`. Side effects: one fingertip/object read per iteration, one closed-loop reach command per iteration, optional watch-joint read when `watch_match` names an object.
- verified scope: 02_push_button -- same ten seeds as walk_to_reach. Converged in 4-7 reaches on seeds 1,2,3,5,7 but NOT on seed 7 (0.0226 m after 8 iterations, task still succeeds): `max_iters` termination is a normal outcome, so a caller must read the returned residual rather than assume `tol`. No cross-task evidence.
- used by: 02_push_button composition.
- CAUTION: `max_iters` termination is a NORMAL outcome (seed 7 ends 0.0226 m and the task passes) -- read the returned residual, never assume `tol` was met.

Proof history: `library/evidence/servo_fingertip_to_point.md`.
