# seat_pads_for_grip -- open before calling

Signature: seat_pads_for_grip(box_ref=None, press_steps=None, press_ncmd=4, press_out0=0.004, off=(0.076,-0.018,-0.030), x_in=0.03, z_frac=0.94, pad_half=0.0217, box_move_tol=0.02, box_move_hard=0.25, place_dur=0.40)
- source hash: f0d78ba66919b105b2190f8e877aeb7b256c50659f04de7507f57a2833ad3b5a (01_box_lift_floor/actor_4/skills/seat_pads_for_grip.py)
- removes: a grip that closes with the right pad only -- with 1-3 correction commands per press step, seed 5 closes c3 L0/R3.
- applies when: the pads were brought to the face pair by `place_pads_closed_loop`, `box_ref` is settled, and the box is at rest.
- contract: pre: pads at the face pair -> post: for each press step (default [0.012,0.003,-0.006] m of inboard stand offset) the pads are commanded normal to the faces with `press_ncmd` measured corrections each; stops at the first step whose per-command displacement gate trips; returns `{ok, failed_at, records, contacts, sides, commands}`. Commands arms only and adds NO trailing observation, so the caller can close immediately without changing the read sequence. Side effects: one `geom_detail` per press step plus, per command, the `place_pads_closed_loop` read block.
- verified scope: 01_box_lift_floor, floor box. `press_ncmd=4` seats BOTH pads on seed 5 only (press2 ends c4 L2/R2, clear L -0.0008/R -0.0022; close c5 L1/R4; dz +0.316; engine success true). On unseen 10/11/12 it aborts after 3/2/2 commands and leaves one pad 4.5-6.6 cm clear -- and the task STILL passes. pn2/pn3 on seed 5 are proxy failures but engine successes (t 18.82/20.42) and all 8 settings of the T2 grid on seeds 3,4,5 at pn2 pass, so "fewer than four press commands fails" is FALSE as a task claim. No cross-task evidence.
- used by: 01_box_lift_floor composition.
- CAUTION: fewer than four press commands does NOT fail the task (the 8-setting pn2 grid all passes) -- do not gate on it.

Proof history: `library/evidence/seat_pads_for_grip.md`.
