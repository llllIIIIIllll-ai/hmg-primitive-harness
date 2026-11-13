# Measured corrections to the library records (coordinator, 2026-11-10)

The `library/*.json` records are claims written at promotion time. This file
records, per record, which of their claims were RE-MEASURED during the skill
extraction + verification round and did not survive. The records themselves are
left byte-unchanged so their provenance stays intact; `PRIMITIVES.md` is the
catalog of what is actually verified.

## pick_drill_from_holder.json (`pick_drill.json`) — SEVERAL CLAIMS REFUTED
Pinned source `035ede59…80f9`, 8069 B. The record is internally consistent
(embedded `contract.source` == disk == `contract.source_sha256` ==
`contract_frozen_sha256`), so these results are about the bytes it names.

- `validated_domain.seeds [3,4,5]` with "engine_success 3/3 unseen" and
  `gate_a` "PASS 3/3 unseen seeds (3,4,5)": **FALSE**. Seed 3 fails at these
  bytes, composed and promoted alike: t 28.34, engine success False, drill on
  the floor at [0.5827, -0.1432, 0.0148]. Chain: creep guard `moved` fires at
  j=16 (drift 20.1 mm vs move_tol 20 mm); the close happens around a 20 mm
  misaligned handle; the drill leaves the gripper inside
  `extract_held_horizontally` at row 4 (object z 0.718 -> 0.398).
  CORRECTED domain, measured: seeds {4,5,6} pass (t 34.10 / 32.50 / 34.10;
  final drill z 0.8247 / 0.8126 / 0.8201); seed 3 excluded and documented;
  seed 7 coordinator-only.
- "lift 113/112/109 mm": **not reproducible**. Measured rise from pre-lift is
  91.6 / 105.7 / 104.1 mm on seeds 5/4/6; seed 3 rise is 0.
- `gate_b` "no commanded range/actuator violations": **FALSE** under the
  harness's own definition. debounced TORQUE_SATURATION counts 8/17/19/19 on
  seeds 3/4/5/6, raw per-step frac>=1.0 counts 96/293/324/329; dof 29
  `gripper0_left_left_hand_thumb_1_joint`, max frac 1.3369 (ctrlrange 1.0472).
- `index.json` repeats the wrong domain ("seeds 3,4,5 + held-out 6,7").
- Unqualified "zero drill-holder contacts during lift": vacuous on seed 3
  (drill on the floor) and the EXTRACT rows do have drill-holder contacts
  (seed 5 rows 2,4,2,0,2; seed 4 4,4,2,0,2,0,0; seed 6 2,3,3,1,1,0,0).

## box_lift.json — "unseen" and gate-B wording do not survive
- `validated_domain` "3/3 unseen seeds (16,17,18)" describes the PROMOTED
  program. For the extracted composition, 16/17 had already been exercised and
  18 was already inside `validated_domain.seeds`; only seed 19 was new to both.
  Do not inherit "unseen" for an extraction.
- `gate_b`: TORQUE_SATURATION fires 9 times on seed 19 and 9 on seed 17
  (dof 43, frac 1.3369) and JOINT_LIMIT fires on every seed (margins -0.00118
  to -0.00506, `safety_ok` False on 16-19). The promoted control has IDENTICAL
  counts and maxima per seed, so this is pre-existing task/controller
  behaviour, not an extraction defect -- but "zero violations" is not a claim
  the record can make.

## drill_pick_place.json — same gate-B wording issue
- "gate B pass" cannot mean "no saturation": seeds 5,6,7 carry
  TORQUE_SATURATION 34/30/34, CONTACT_PENETRATION 18/34/33, ACTION_JUMP 11/11/11
  and torque_frac 1.3369 at dof 43. The composed traces are bit-identical to
  the promoted ones, so these events are pre-existing.
- "10/10 at default kwargs": the extracted composition reproduces the promoted
  program exactly on seeds 0-9, so the claim is inherited intact for the
  MONOLITH; it does not automatically transfer to any single skill.
- The success predicate was not available to the verifier: `ok` in the response
  means "no exception", not task success. Any future claim must say which
  source produced its success bit.

## push_shelf_forward.json — seed 5 is not unseen
- `validated_domain.seeds` is [3,4,5], so the verifier's "seeds 5,6" contains a
  seed the record itself lists. Seed 5 is a reproduction, not a new seed; only
  6 was new to the composition and 7 to the coordinator.
- The composed program returns one EXTRA key `bursts` (a per-mode burst
  census) that the promoted program does not, so "byte-identical responses" is
  not a true claim; the physical fields are identical (11/11) and t matches.

## push_button.json — weakest domain, unchanged
- No success string in the recorded domain (seeds 1,2,3,5,7,11,12,13,21 with
  held-out 31,37). Nothing to refute, but nothing to inherit either: the
  extraction was verified on seeds 1,2,3,5,7 + 11,12,13 + 21,31,37.

## pick_drill_from_holder.json — domain extension (coordinator, 2026-11-11)
Pinned `035ede59…80f9` (8069 B, `promoted_pinned`), one batch, seeds 0-20,
engine bits: **18/21 TRUE**, FALSE on seed 3 (t 28.34), seed 16 (27.66) and
seed 17 (29.70). This supersedes the earlier "corrected domain {4,5,6}":
seeds 0,1,2,7-15,18-20 pass as well, so the domain is
{0,1,2,4-15,18-20} on the tested range. Seed 3's established failure
(creep guard 'moved', drill on the floor) reproduces exactly at t 28.34.
