# Evidence: place_pads_closed_loop

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## place_pads_closed_loop(tag='place', stand=0.0, xb=0.0, zu=0.0, ncmd=2, free=True, hold=None, out0=None, dur=0.40, lo=None, hi=None, box_ref=None, x_in=0.03, z_frac=0.94, pad_half=0.0217, off=(0.076,-0.018,-0.030), box_move_tol=0.02, box_move_hard=0.25)
- source hash: dd05958a86e3f27dcf548f42cdb326c59101a4e5977b3bf06480dbf5f3461b82 (work/tracks/01_box_lift_floor/actor_4/skills/place_pads_closed_loop.py)
- removes: a one-sided grip from an unseated pad -- the pad centre sits at the commanded IK site plus a fixed world offset, so an open-loop site leaves the pad 1-6 cm clear of its face; and a CUMULATIVE displacement gate aborts every later pad correction once one command has swept the box.
- applies when: box at rest with a `box_ref` settled after the crouch (never taken while the box is airborne), both palms have a two-palm IK solution, and the caller supplies the box AABB. The caller owns the base and the fingers.
- contract: pre: settled box_ref as above -> post: up to `ncmd` measured corrections of both palm sites, each gated on the displacement THAT command caused; returns `{ok, record, log, contacts, sides}`. Side effects: per command one `reach_box` (steps `dur` s), one `geom_detail`, two `observe`, one `hand_contacts_side`.
- verified scope: 01_box_lift_floor, floor box. Seated bilaterally on seed 5 (verifier, independent pn4) and re-scored from saved responses on 3,4,7,8; NOT seated on 0,2,6,9,10,11,12. It does NOT recover a gross miss: on unseen 10/11/12 the press aborts at 3/2/2 commands with one pad 4.5-6.6 cm clear (and those episodes still lift). On coordinator held-out seed 13 one retry recovers the reach but the close is c2 L1/R1 and the box is shoved (AABB top 0.276). Composition traces are bit-identical to the pre-extraction candidate `cand_v8` (05b76db4...662e) on seeds 5,10,11,12. No cross-task evidence.
- used by: 01_box_lift_floor composition (free-space reach and the press's inner loop).
```
