# Evidence: grasp_lift_floor_box_branch

Source pinned 3eacdc603b2bc9153717de2d85946775b4b52729bd7ebbc794393bb646a5fe3c
(01_box_lift_floor/actor_7/skills/grasp_lift_floor_box_branch.py, 19116 B).
Child: place_pads_branch 35cc3638...; the pinned jumpguard composition
grasp_lift_floor_box_jumpguard is reproduced at branch='off', repair_n=0.

## Coordinator engine read (the task bit, coordinator-only)
Ship kwargs {"branch":"refuse","branch_min_cmd":1,"repair_n":1,
"repair_wide_add":0.0,"repair_back_add":0.02,"press_ncmd":4,"seated_tol":0.0}.
Engine TRUE on ALL 22 seeds 0-21 (sweep 19:20-19:23, matched to
ev/all_A_refuse_repair.json by identical t; seed 20 TRUE at t 23.96).
Rerun of seed 20 TRUE at t 23.96 (deterministic).
Controls, same source and environment:
- branch='off' (pinned placement): engine FALSE on seed 20 at t 18.76.
- refuse WITHOUT the repair pass: engine FALSE on seed 20 at t 17.96.
- re-aim only (branch_alphas 0.5/0.25/0.0): engine FALSE on seed 20 at t 17.96.
So the repair pass is necessary and re-aiming cannot escape the seed-2 basin.

## Gate A (verifier_3, independent)
Re-registered 3eacdc60... from disk and ran seeds 0,6,13,20,21 with the ship
kwargs. Deep comparison to the actor log: 0 differing leaves, all ok=True.
Proxies reproduce exactly (s0 box_end [1.049,-0.035,0.304]; s6 z_hi 0.858
recovered; s13 z_hi 0.8861; s20 repairs 1, refusal reach_r0_ybef cmd1 left
'switch'; s21 z_hi 0.8476). Determinism: two independent reruns (reload=true)
deep-compare identical and all 15 trace arrays are byte-identical (s0 N=1061,
s20 N=1198).

## Gate B (verifier_3) -- ACCEPT NARROWED
Command surface 35-D every seed (31 upper + 3 nav + 1 height); lower body never
commanded (lower_cmd empty at every step) while 12 WBC joints move
0.0615-1.7225 rad. Torque: peak |tau|/ctrlrange = 1.00000 (raw per-step
saturation) on left_shoulder_pitch (s0 t 8.200) and left_shoulder_roll (s13,
s21, t 8.560); s6 0.89127, s20 0.70161. Raw steps with a dof >= 0.995: s0 4,
s6 0, s13 13, s20 0, s21 5 -- the debounced TORQUE_SATURATION event count
(1/0/3/0/1) undercounts. Joint-limit margin: worst overall is right_ankle_pitch
-0.03536/-0.04118/-0.02412/-0.03903 rad (s0/s13/s20/s21), driven by the WBC, not
by this skill's commands; worst UPPER is left_wrist_roll
-0.02272/-0.03216/-0.01376/-0.02116 rad. Commanded values exceed the model range
on 9-12 channels per episode, mostly sub-mm. Base drop 0.1179-0.1237 m, peak
tilt 0.4608-0.4835 rad. Determinism byte-identical (above).

## Gate C -- REJECT
No valid second-task instance; see place_pads_branch evidence. The mechanism
was screened against 03 and fired only false positives on unsolved IK targets.
This skill is a 01 solution, not a cross-task skill.

## Falsifiers
Any trace difference against the jumpguard composition at branch='off',
repair_n=0; any refusal on a row whose current branch solves; torque/limit
behaviour outside the measured ranges above.

## Scope extension: seeds 22-40 (coordinator, 2026-11-11)

Same pinned source (`3eacdc60…fe3c`), same ship kwargs
(`branch='refuse', branch_min_cmd=1, repair_n=1, repair_wide_add=0.0,
repair_back_add=0.02, press_ncmd=4, seated_tol=0.0`), 19 unseen seeds:
**15/19 engine TRUE**. FALSE: s28 (t 19.16), s29 (19.22), s32 (20.02),
s33 (20.36). Passing seeds span 16.88-21.62 s, so t does not separate them.
Request `work/coordinator/req_coord01_22_40.json` (source 19116 B, sha256
asserted before sending), one batch on the 01 queue.

Consequence: the verified scope is seeds 0-21 (22/22, unchanged) and the task is
bounded there; over 0-40 it is 37/40. Do not quote "01 solved" without the range.
