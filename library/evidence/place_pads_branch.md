# Evidence: place_pads_branch

Source pinned 35cc36386db2c99bb53924b4e2459e8f5b60e873a8576a515a009d9ca524eae7
(01_box_lift_floor/actor_7/skills/place_pads_branch.py, 18283 B).
Supersedes place_pads_closed_loop dd05958a86e3f27dcf548f42cdb326c59101a4e5977b3bf06480dbf5f3461b82.

## Merge test (verifier_3, seeds 0/13/20)
Same inputs (actor per-seed settle box_ref/lo/hi), one run per arm.
Trace comparison: 15 arrays, 0 differing (np.array_equal, NaN-aware). Response
comparison: DIFF 2 leaves per seed -- `/jump_guards` (absent vs []) and the
`/log` phase tag (`m0_closed` vs `m0_branch`); per-command records and `record`
equal ignoring that tag. Candidate-only keys: jump_guard None, jump_guards [],
branch_refused False, branch_refuse None.
Caveat kept explicit: the wrapper replays only the place call, not the
crouch/settle stance, so both arms' commands are reach failures (ik_err
1.25-1.33 m). This is a code-path identity proof (branch='off' skips preview and
guard entirely; the remaining loop is line-identical to the incumbent), not a
full-episode identity.

## Gate A / composed use (grasp_lift_floor_box_branch, verifier_3)
Pinned 3eacdc60... re-registered from disk; seeds 0,6,13,20,21 with ship kwargs
{"branch":"refuse","branch_min_cmd":1,"repair_n":1,"repair_wide_add":0.0,
"repair_back_add":0.02,"press_ncmd":4,"seated_tol":0.0}. 0 differing leaves
against the actor's log on all five; deterministic reruns byte-identical (s0
N=1061, s20 N=1198, all 15 arrays). Refusals: s20 reach_r0_ybef command 1, left,
why {'left': ['switch']}; repairs 1.

## Screen statistics (corrected units)
528 screened SIDE-ROWS = 264 command entries x 2 sides. 504 at cmd==1, 24 at
cmd==2. Exactly one cmd>0 row wins on a non-current branch: seed 20
reach_r0_ybef, left, seed 2, err 0.0, ratio 1.345, clear_pred_m 0.0491,
pad_travel 0.0487 m, dq 4.416 rad.
The actor's "64 ci==0 rows solve on a non-zero branch" is 64 first-command
LINES out of 252; as side-rows it is 71 (seed episode 2: 60, seed episode 1: 11),
matching the executed ik_seed != 0 sides exactly.
The actor's "worst clear_m -0.0003 m" is the MEASURED clearance of an executed
command (seed 9, reach_ybef, cmd 1, left), not the preview, whose predicted
clearance was +0.0427 m -- the preview overestimated clearance by ~0.043 m.
One command was executed despite a warning: seed 18 reach_wide_hi cmd1 left,
seed 2, why ['travel'] (branch_min_cmd=1 does not block it; it did not shove).

## Gate C -- REJECT, no cross-task instance
The solved 03 placement loop was replayed with one read-only ik_preview per
commanded target on 5 seeds (20 commands, 40 command-sides). All 03 targets are
unsolved for the 01 stance: pre_ik_err 0.2443-0.2680 m; per-command winning err
0.149-0.218 m vs the 03 skill's own ik_err_max=0.09. Seed 0 wins on 38/40 sides;
the two non-current wins are seed-2 artifacts of non-convergence. The shipped
criteria would fire on 5/20 commands, every firing a false positive. Geometry:
the 03 loop commands z ~= 0.277 m; this robot cannot solve below z ~= 0.477 m
here. Conclusion: no valid second-task instance. `ik_preview` being shared is a
shared TOOL, not a reused skill.

## Scope boundary
Unblocked stand, solvable targets. The refusal criterion is only meaningful
when the preview converges; on unsolved targets it fires false positives. Not
verified: that any refusal prevents a worse command by itself (the repair pass
is what fixes seed 20 -- refuse-only is engine FALSE at t 17.96).
