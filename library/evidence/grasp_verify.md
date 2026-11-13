# grasp_verify -- proof history

Pinned source: `e6d66d025681099ad1ec18645f298185d47526cafd0418455d87439c5a83cb61`
(05_drill_lift/actor_5/skills/grasp_verify.py). Orchestrated by
`drill_lift_v35` (`9ec69d48dca4a0280255bada089d4cc79aa00f3d6cef025ef0f2b80db7837fbf`)
WITH the actor's call kwargs
`{"off":{"seat":1,"seat_mode":"branch","br_on":1,"br_gate":1,"ps_on":0,"tw":0,"pin1_ramp":0,"pre_stance":1,"gate":1,"servo_on":0}}`.

## Gate A (unseen seeds) -- ACCEPT
Coordinator-requested verifier round. Seven seeds re-run at the pinned artefact;
post-close classifier values compared with the actor's `v35a.json`:

| seed | held | n_fing | fmax | gap | final obj z | vs actor |
|---|---|---|---|---|---|---|
| 0 | 1 | 4 | 18.169 | 0.0832 | 0.965 | exact |
| 1 | 1 | 4 | 19.89 | 0.081 | 0.935 | exact |
| 2 | 1 | 3 | 25.239 | 0.0794 | 0.939 | exact |
| 3 | 1 | 5 | 17.805 | 0.0813 | 0.937 | exact |
| 4 | 1 | 4 | 19.713 | 0.072 | 0.923 | exact |
| 7 | 0 | 0 | 0.0 | 0.0108 | 0.838 | exact (never lifts) |
| 12 | 1 | 4 | 15.578 | 0.0683 | 0.953 | exact |

`pinch_seed`, `seat_pre.mode` and `gated` also match on all seven. Seeds
5,6,8,9,10,11 are actor-reported only (held=1, gap 0.0693-0.0813).

## Gate B (no side effects) -- ACCEPT NARROWED
Static: source calls only `api.observe`, `api.hand_geoms`, `api.contact_detail`,
`api.t`; no command path, no writes. Empirical (05 seeds 0,7): call vs a no-op
copy (`gvv_v35_noop` 00de48e5...) leaves the result dict identical to 4 dp and
all discrete counts equal; call-vs-call traces bit-identical on all 15 trace
fields. Call-vs-noop raw-trace max |delta|: qd 1.668e-4, tau 1.547e-4,
q 4.649e-6, act 3.576e-6, obj 9.23e-7 -- the read-side `mj_forward` warmstart
perturbation, not a command difference. Narrowed wording: "same logged state and
commands", not bit-identical raw solver trace.

## Gate C (cross-task reuse) -- REJECT
Same unchanged skill, called after the close on two other tasks, on objects that
were then genuinely lifted:
- 07_box_table_to_shelf (box, two-palm hold, probe gvv_pp26_a5; box lifted to
  z 0.734/0.741): seed 6 held=0, n_fing=5, gap -0.0266, n_bc=13; seed 0 held=0,
  n_fing=3, gap -0.0263, n_bc=11. All grasping contacts are `gripper0_right_*`
  while the gap test is the LEFT hand, so the sign is meaningless.
- 09_obstacle_aware_pick_drill (drill, left-hand top grasp, probe gvv_cand_v16;
  drill picked to z 1.031/1.032): seed 6 held=0, n_fing=2, gap -0.0301;
  seed 0 held=0, n_fing=2, gap -0.0308. Hand side is right, n_min is met, but
  the z-gap sign is inverted for a top grasp.
The `gripper0` contact-row count transfers (07 n_fing equals the actor's own
handbox counts at the same instant). The `held` decision does not.

## Falsifiers / boundaries
- 05: held=1 with an unchanged final object z -- none among the actor's 13 seeds
  or the verifier's 7.
- Any task: held=0 with n_fing >= n_min while the engine lifts the object --
  observed on 07 seeds 6/0 and 09 seeds 6/0; this is what rejects cross-task use.
- Called before close: always held=0 (pre-close n_fing=0 on every 05 seed).

## Provenance and retractions
The verifier's first gate-A pass reported seeds 1,3,4,12 as "not reproduced" and
narrowed gate A to 0/2/7. That was retracted: it had submitted the request with
empty kwargs (server-side `{}`), which selects a different program branch on
05 seeds 1,3,4,7,12 (seed 12: pinch_seed 2, `pad_seat`, final z 0.838, engine
FALSE). With the actor's kwargs the run matches exactly (seed 12: pinch_seed 0,
final z 0.953, engine TRUE t 13.4). The reproduction claim is therefore pinned
to (source hash + call kwargs). See `library/LESSONS.md` and the 05 track
memory for the coordinator recheck.

Evidence files: `work/coordinator/gv_verify/` (`req_gvv_A2_kw.json`,
`resp_gvv_A2_kw.json`, `gateA_kw_run.py`, `resp_runB_noop.json`,
`resp_gvv_07_run.json`, `resp_gvv_09_run.json`, `hashes_before.txt`).
