# Evidence: top_up_reach_at_standoff

Source pinned: 7a8d8766d3e7fd2b8da9ddb1bad3fdf4ad50795da1f9c06257ac07bdb352f01c
(09_obstacle_aware_pick_drill/actor_extract/skills/top_up_reach_at_standoff.py).
Program evidence: cand_v16 0fd489fdedf8d4d7e4af3895e7dbdc23d525abc386ebcfe0b45f1558d2eee19c.
Child: walk_base_to_coordinate 66f57455e45d293bb63e6b73f8423059b71e3d7482f07912db5e5e205496cd64 (08).

## Gate A (verifier, unseen seeds 6-11, pinned re-registration before the run)
Proxy = hold hand-contacts >= 4 and object z ~= 1.02-1.03 m.

| seed | reach dist -> after / nudges | close contacts | hold ncon / z | proxy |
|---|---|---|---|---|
| 6 | 0.299 -> 0.299 / 0 | 2 | 5 / 1.031 | PASS |
| 7 | 0.282 -> 0.282 / 0 | 1 | 5 / 1.027 | PASS |
| 8 | 0.305 -> 0.305 / 0 | 2 | 3 / 1.028 | PASS |
| 9 | 0.337 -> 0.312 / 1 | 0 | 0 / 0.838 | FAIL |
| 10 | 0.336 -> 0.302 / 1 | 2 | 5 / 1.023 | PASS |
| 11 | 0.313 -> 0.313 / 0 | 0 -> 1 hold 6 | 6 / 1.031 | PASS |

Nudge dbase on 9 (+0.046,+0.034,+0.009), on 10 (+0.047,+0.025,+0.007); object
moved 0 in both. Engine bits agree with the proxy on all six (TRUE 6,7,8,10,11;
FALSE 9).

## Coordinator held-out (seeds 12-17, untouched by actor or verifier)
Engine on cand_v16 0fd489fd...: TRUE 13, 15, 16, 17; FALSE 12, 14.
Diagnosis of the two failures (results="full" re-run): dist 0.309 (s12) and
0.307 (s14), nudges 0 in both -- the skill never fired because the distance was
already inside reach_max=0.32. The failures are aim/grasp failures outside this
skill's trigger, not contract violations.

## Gate B (all six gate-A episodes)
Joint-limit margin: worst -0.00823 rad (right index finger, s8 t 4.92), all
finger joints, 0-19 raw steps < -1e-3; lower body min +0.1056 rad. Torque: peak
|tau|/ctrlrange 0.89127 (left middle, s7 t 1.2), zero raw steps >= 0.995. Lower
body peak 0.56796 (right ankle pitch, s9 t 28.62), WBC-owned; act is 31 cols +
nav 3 + height = 35-D goal, no hip/knee/ankle ever commanded. Determinism:
independent rerun of 6-11 -- all 15 trace arrays byte-identical per seed, only
wall_s differs.

## Gate C (call through the catalogued 08 child)
v14 (37a9994d...) vs v15 (2e59058e...) on seeds 4,5,9,10: all 15 trace arrays
bit-identical; t identical (43.72/82.82/71.52/73.12). The only delta is
`creep_skill`: 1 burst of 2 allowed, nread 2, base overshoots b0+0.046 for
dx=0.03, lateral drift 0.025-0.034. Bit-identity is mechanically expected: the
child issues the identical move_base stream and the correction loop never
engaged. Widened `walk_base_to_coordinate` scope only to this single-burst,
unblocked-base composition.

## Ablation
Seeds 4,5,9, default vs max_nudges=0: default s4 0.344->0.313/1 close 2 hold 4
@1.034 PASS; s5 0.336->0.304/1 close 2 hold 6 @1.030 PASS; s9 fails both. n0:
s4 close 0 hold 0 @0.838 FAIL; s5 close 0 hold 0 @0.838 FAIL; s9 close 2 hold 0
@0.799 FAIL. The control reproduces the actor's resp_v16 exactly on 4,5 (diff
keys empty). Effect present, vanishes when disabled.

## Claims not reproduced
"resp_v17 reproduces resp_v16 byte-identically": refuted (92699 vs 92698 B,
sha256 df6d4f6d... vs 796bf610...; differs in trace path, wall_s, reset_s; per
seed result dicts and all 15 trace arrays equal). "6/6 on seeds 0-5": only 4,5
(and 9) independently re-run by the verifier; 0-3 rest on the actor's runs, whose
engine bits the coordinator read (TRUE 0-5).

## Scope boundary
Unblocked base, left side, default shoulder_off, reach_max 0.32, dist
0.31-0.34, <= 3 nudges. Cannot repair an aim error (seed 9) or a blocked base.
