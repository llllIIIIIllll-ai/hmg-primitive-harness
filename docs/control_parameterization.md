# Control parameterization: 12-D palm poses, conditional waist, mink QP

Read-only reference for proposal, execution and validation actors. The point is
to stop searching in joint space and solve a small, physically meaningful task
space instead. The adopted default is **12-D with `waist_owner=homie`**; a
user-authorized runtime handover to `ik`/`blend` makes the **17-D** candidate
(two palm poses plus the waist) legal.

## The task

IK solves **two palm poses (12-D)**: position (3) and orientation (3) per hand,
over the 14 arm joints. Two redundant DoF remain for null-space posture.

**Conditional waist.** The default is `waist_owner=homie`: the waist is
**exogenous** — not an IK variable, but its measured value is fed into
FK/Jacobian on every control step. Excluding it from the optimization does not
mean it is stationary — HOMIE v2 moves it while walking. The user authorized a
runtime handover (2026-11-15, recorded in `experiments/ll_v2/config.yaml`
`control.waist_owner_modes`): after `api.authority(waist_owner='ik')` the waist's
final writer is `upper_target`, and after `blend` it is a weighted write. Under
either owner the waist becomes a **command target** and the task becomes
**17-D** (two palm poses + three waist joints). Verify the achieved waist gain
before and after the handover and restore `homie` before the skill returns.

A planning search may additionally vary the base (x, y, yaw, height = 4), which
makes 16 search dimensions; the base is placed closed-loop and then held fixed
while the arms are solved.

Grip and finger joints are not IK dimensions: the close is contact-triggered.

## Solver

Default: **mink QP** (`qp_reach` in the harness, qpsolvers/daqp) over both arms
(12-D) and, only while the waist is IK-owned, over the three waist joints as
well (17-D), with `ConfigurationLimit`, a high-cost base-lock frame task and a
posture task. Budget: 80 iterations per primitive (config
`ik_solver_budget_iters`).

DLS is the fallback only when the QP is infeasible, for read-only previews, or
for an explicit diagnostic comparison.

## Solver contract

1. place the base at the requested offset/yaw/height (closed loop, measured);
2. solve the two palm poses (12-D) with the base fixed and the measured waist
   exogenous **while `waist_owner=homie`**; under an authorized `ik`/`blend`
   handover solve 17-D and verify the achieved waist gain before and after;
3. enforce, and report, these constraints:
   - joint margin >= 0.05 rad on every commanded joint (no guard trips),
   - base tilt <= 8 deg at the end,
   - no base/table penetration and no hand/object interpenetration before the
     close;
4. return the binding constraint when it fails, plus the achieved pad errors.

Constraint violations are a **negative result with a named cause**, not a
reason to retry the same point.

## Search policy

- Default to the **12-D** space (two palm poses over the 14 arm joints), not
  joint space and not over stance tables. It is the adopted, measured default:
  on `05_drill_lift` seeds 0-3 its executed pad error beats both 17-D conditions
  on every seed, and it is the only condition that met the 15 mm gate on any
  seed.
- The **17-D** space (adds the three waist joints) is legal only under an
  authorized `ik`/`blend` waist handover. It is authority-compliant under `ik`
  but loses on executed pad error on all four seeds; selection is by executed
  pad/contract, not by model residual. Do not use 17-D under `homie` (the
  authority-negative control).
- Start from the object-relative nominal stance, then a bounded local search
  (e.g. CMA-ES or coordinate descent) with a hard budget per primitive.
- One task at a time: get one task's grasp working on the fitted seeds, then
  extend to the other tasks and held-out seeds.
- A candidate that only works on one seed is not a capability; report the
  measured boundary instead.

## L1 contract: authority, exogeneity, execution

An L1 is not "a function that attempts a motion". It is a control capability
that can verify a physical outcome under declared authority and constraints.

Every L1 declares four separate fields:

| field | meaning | verified by |
|---|---|---|
| `commands` | joints this L1 writes | structural: the registry's `final_writer` must be `upper_target` for each of them |
| `observed_exogenous` | joints this L1 does NOT command but must read and feed into FK/Jacobian (the waist **while `waist_owner=homie`**) | measured: read every control step, never commanded, drift reported |
| `assumes_fixed` | joints assumed constant during the motion | measured: drift must stay under the declared bound, else the contract fails |
| `min_gain` | per commanded joint, the minimum achieved/commanded ratio | measured: `track_gain` |

A skill that hands the waist over also declares `waist_owner`,
`waist_gain_before`, `waist_gain_after`, `handoff_events` (each HANDOFF event
with its owner and the restore) and `lower_owner`.

Authority is structural and comes from `docs/joint_authority.json`: the
lower-body policy is written after the upper body, so a joint present in both
groups is finally written by HOMIE v2 (**lower wins**). The default waist owner
is therefore `homie`; a user-authorized runtime handover (`api.authority`) may
make `ik` (upper target) or `blend` the waist's final writer, and the default
owner is restored on exit. Tracking gain is dynamic and is stored next to
authority, never instead of it: a high gain does not confer authority and a low
gain does not remove it.

The exogenous/exactly-fixed distinction matters for IK accuracy: excluding the
waist from the optimization variables does not mean the waist is stationary.
While `waist_owner=homie`, HOMIE moves it while walking, so the IK must solve
against the measured waist every step, not against a value captured at the
start.

## What this does not fix

The 12-D formulation does not add reach, grip force or WBC headroom. If the
binding constraint is arm reach, gripper span, or the WBC refusing a posture,
reducing the dimension count will not remove it: report it as a structural
boundary with the measured shortfall (metres, radians, or joint margin).
Handing the waist to `ik` adds a posture degree of freedom, not reach, and the
measured 17-D executed pad is worse on every seed.
