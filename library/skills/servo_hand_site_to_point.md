# servo_hand_site_to_point -- open before calling

Signature: servo_hand_site_to_point(target, side='left', tol=0.006, step=0.022, dur=0.5, iters=220, max_steps=12)
- source hash: e8a38255a2b3db99e521c70b991cbf96994abbb7d5fa3097a4cb2548469605b5 (08_pick_drill_from_holder/actor_extract/skills/servo_hand_site_to_point.py)
- removes: reaching a hand site open-loop -- the final few cm are the difference between a grasp and a shove.
- applies when: the hand site is readable and the target is reachable within max_steps.
- contract: pre: hand site measurable -> post: hand site within tol of the target (converged in 7 of 12 allowed steps) or max_steps spent; returns the step record and final pose.
- verified scope: 08 seeds 3-7 (verifier 3-6, coordinator 7). Breaks when the target is unreachable within max_steps.
- used by: 08_pick_drill_from_holder composition.

Proof history: `library/evidence/servo_hand_site_to_point.md`.
