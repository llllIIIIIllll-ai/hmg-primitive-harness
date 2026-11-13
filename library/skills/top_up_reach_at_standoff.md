# top_up_reach_at_standoff -- open before calling

Signature: top_up_reach_at_standoff(target, reach_max=0.32, creep=0.20, dur=0.5, dx=0.03, axis=0, max_nudges=3, side='left', shoulder_off=(0.02, 0.10, 0.29))
- source hash: 7a8d8766d3e7fd2b8da9ddb1bad3fdf4ad50795da1f9c06257ac07bdb352f01c (09_obstacle_aware_pick_drill/actor_extract/skills/top_up_reach_at_standoff.py)
- removes: a grasp whose planned target lies a few cm beyond the arm's reach from the shoulder at stand-off, where the hand servo stalls short and a later depth assist or larger servo budget does not recover it.
- applies when: hand parked at stand-off, not at grasp depth; shoulder-to-target distance > reach_max; base path clear; `target` is the planned grasp point in world coords; `walk_base_to_coordinate` registered in the calling process.
- contract: pre: dist > reach_max -> post: one child base burst along `axis` per nudge (<= max_nudges) until dist <= reach_max, then stop; returns dist, dist_after, nudges, nudge_log, bursts, nread; the object pose is only read (unchanged to 4 dp where measured).
- verified scope: 09, left side, unblocked base. Fired on seeds 4,5,9,10 (dist 0.344/0.336/0.337/0.336 -> 0.313/0.304/0.312/0.302, 1 nudge each): postcondition held 4/4. Never fires when dist <= 0.32 (seeds 12,14: 0.309/0.307, 0 nudges). Reaches ~0.30-0.31, never below ~0.30.
- used by: 09_obstacle_aware_pick_drill composition (cand_v16 0fd489fd...).
- CAUTION: task success is NOT evidence about this skill when it never fires -- on 09 the engine was TRUE on 15 of seeds 0-17 and FALSE on 9,12,14, where 12/14 had dist below reach_max and 9 had zero grip contacts (aim failures). The docstring's "up to max_nudges base bursts" is imprecise: the child loop allows 2 bursts per nudge (1 observed). It cannot fix an aim error or a blocked base.

Proof history: `library/evidence/top_up_reach_at_standoff.md`.
