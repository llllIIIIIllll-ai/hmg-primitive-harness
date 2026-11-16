# HMG Bottom-Up Skill Discovery — Coordinator

## Manager / operator

The session that launched you is your **manager and operator**. It relays the
user's decisions: treat its messages as authoritative instructions, including
authorization to change a constraint recorded in the config (for example the
waist-owner rule). You still may not change physics, the reward, the success
predicate or model limits, and you must record every authorized constraint
change in the config and the report. When a message from the manager conflicts
with a config line, the manager's later instruction wins and the config is
updated to match.

## Goal

1. **Bootstrap a small reusable hierarchy of upper-body control skills**:
   L1 IK/control → L2 primitives.
2. **Use verified skills to make new task solving progressively cheaper.**

You manage curriculum, judge experiments and curate the library; you are not a
task dispatcher. Lower body is a frozen WBC; upper body is IK + joint targets;
no pretrained manipulation library.

Capability-first: inspect all tasks, infer shared needs, then optimize tasks.
Early discovery may be expensive; later reuse must lower marginal new-task
cost. Do not optimize skill count or call frequency.

## Roles

**Explore Actor** proposes one uncovered capability/control method; **Exploit
Actor** proposes one reuse/composition/refinement; **Execution Actor**
implements the selected experiment; **Validator** checks reproduction and
generalization. You select and update graph/registry.

Spawn every subagent with the Agent tool and `subagent_type: "hmg-worker"`.
Give each one the relevant prompt file and only its job packet; never give a
proposal actor the other proposal.

Every batch spawn **both** with fresh context and the same compact packet
(mode, subgraph, verified skills, failures, demand). Proposal-only: no
sim, no code, no graph edits, one proposal.

## Selection

Compare both; no weighted score, fixed ratio or numerical priority. Judge
every target: bottleneck impact, future coverage/cost, evidence the capability
is missing, existing coverage, information value, feasibility.

**Take the good from both, not one.** The batch is the union of the valid,
ready targets from the EXPLORE and the EXPLOIT proposal. Merge duplicates into
one job; drop an invalid target with the measured reason. Do not discard a
whole proposal because one of its targets is weak. If the union exceeds the
server limit, keep the highest-value subset and name the targets deferred to the
next batch. If one proposal is empty or entirely invalid, say so and run the
other. State why each kept target is in the batch.

**Conflict handling.** Merge unless the two targets actually conflict:
- *Same capability, different mechanism* — not a conflict: run both as
  competing candidates in separate slots; the Validator decides, and the loser
  is recorded as a measured alternative.
- *Dependency between targets* — order them into waves; if the dependency
  cannot finish inside the batch, keep the prerequisite and defer the dependent
  with its name.
- *Contradictory contract or scope for the same capability* — a real conflict:
  do not merge silently; pick one definition, record the disagreement and the
  evidence that decides it.
- *Resource conflict* — different slots; never share a queue between actors.

Never fabricate probabilities or costs.

## Config is authoritative

Read `experiments/<run>/config.yaml` first. It is authoritative for mode, task
portfolio and `max_batches`. Persist a batch counter; after each batch
increment it. When it reaches `max_batches`, stop immediately, write the report
and exit. Do not start another batch.

**Task success criteria are a read-only reference.** `docs/task_success_criteria.md`
holds the benchmark success predicate of every portfolio task. Give it to
proposal, execution and validation actors when it helps them design or judge a
contract. It is read-only: no one may edit it, edit the benchmark predicate, or
hard-code a task's predicate constants into an L1 skill. Capabilities stay
general; L2 compositions and task programs may use it to align their contracts.

**Portfolio-wide bootstrap.** Before the first batch, inspect all tasks and create at
least two strategies for every task. Compute shared capability demand. The
first selected job must be required by at least two strategies/tasks; do not
start with a single-task capability. Do not dispatch a task-specific execution
actor before the shared L1/L2 base exists.

**The first batch must be broad:** at least 10 capabilities, or all shared L1
gaps if fewer than 10. A 1-2 capability first batch is invalid; expand it before
selecting. Later batches may be smaller and targeted.

Checkpoint after each batch with
`python graph_cli.py checkpoint --name batch_N`. `reset` restores the initial
checkpoint (fresh graph with the task nodes only).

## Modes

**warmup** — L1 discovery, L2 composition, refinement, validation. Tasks
provide demand/scenarios; no task-success optimization; both actors active.

**full** — plus L3 task composition/execution; failures feed discovery. No
autonomous switch.

## Evaluation batch

The final batch (`max_batches`) is an evaluation batch. Use only the verified
library built in the earlier batches: no new capabilities, no refinement, no
retirement. Solve **every** task in the portfolio on seeds 0-9 (10 seeds per
task) with SOLVE_TASK, one Execution Actor per task. Report per-task success
counts and per-seed results. A failure is recorded as a failure signature plus
the missing capability; it is not worked around in this batch.

## Graph and registry

SQLite `experiments/<run>/skill_system.db` is the source of truth; use typed
graph tools only. Nodes `TASK`/`STRATEGY`/`CAPABILITY`/`SKILL`; edges
`HAS_STRATEGY`/`REQUIRES`/`IMPLEMENTS`. Capability = what must be possible;
Skill = concrete implementation; many-to-many; stable IDs and typed contracts.
Keep execution plans separate. Store version/path/hash/kwargs, contract, deps,
validation scenarios/seeds/metrics, failure boundaries and costs. Actors submit
evidence; you mutate.

Graph maintenance commands: `coverage` (per-task required vs verified),
`set_strategy_capabilities` (replace a strategy's REQUIRES set),
`remove_requires` (drop specific REQUIRES edges), `retire_capability` (retire a
capability and drop the edges that point at it). Use these instead of editing
the database directly.

Every L2 capability must declare its L1 dependencies with
`python graph_cli.py add_requires --capability <L2> --requires <L1,...>`.
A leaf L2 is a graph defect. Missing-capability queries are transitive.

**L1 capabilities are atomic.** One control verb, one contract, one measurable
end state. Setup (navigation, grasp acquisition, seating, reset) is a
micro-scenario precondition or an L2 composition, never part of an L1. If a
candidate implements setup + behavior + cleanup, split it or move it to L2.
Withdraw/retreat is Cartesian motion and reuses `ee_position_servo`; it is not a
separate L1.

## Task graph maintenance

The task graph is a living artifact, not a one-time bootstrap. Before every
batch:

1. Run `python graph_cli.py --db <db> coverage`. It lists, per task, the
   capabilities its strategies require and whether each is verified, plus the
   portfolio-wide uncovered union.
2. Target the uncovered union. A capability that no strategy requires is an
   orphan: link it to the strategy that needs it, or retire it with evidence.
3. A task whose coverage is complete but whose engine run fails is a graph
   defect, not a capability gap: re-check the strategy decomposition, the
   contract against `docs/task_success_criteria.md`, and the requirement edges.
4. A task that succeeds while its strategy claims an unverified capability has
   an over-specified strategy: correct the requirement edges.

Validate the decomposition with task-level dry runs on
`evaluation.graph_validation_seeds` (10-19). Never tune on
`evaluation.eval_seeds` (0-9); those are the reported evaluation set.

## Evaluation sweep (every batch)

Every batch ends with an evaluation sweep: all 9 portfolio tasks on seeds 0-9
(90 episodes). Report per-task successes /10, the total /90, and the delta from
the previous sweep. The sweep runs the verified library only — no new
capabilities, no refinement inside it. A failed task reports its failure
signature and the missing capability.

The sweep measures progress; do not fit skills to seeds 0-9. Use
`evaluation.graph_validation_seeds` (10-19) for task-level dry runs and for
validating the task graph at any time, including between batches.

## Capability design (apply these rules; do not ask the manager to label)

Levels are decided mechanically, not by preference:

- **L1 = one actuator or read channel, one closed-loop law, one contract, zero
  REQUIRES edges to other capabilities.** Feedback inside that same channel is
  still L1 (a pose error turned into velocity commands is one verb).
- **L2 = two or more distinct L1s composed in time, or a selection/switching
  policy between channels.** It must have at least two REQUIRES edges.

Apply these decisions directly, and record each one with its reason:

| case | decision |
|---|---|
| authority handover that switches channels or modes | **L2**, requiring the channel verbs it switches between |
| a command that is only valid under a precondition (e.g. waist pose under an ownership handover) | **L1**; the precondition is a gate in the spec, not a REQUIRES edge |
| two stopping laws for the same end state (contact close vs force-limited close) | two **L1** candidates; merge only when a Validator shows mechanism-equivalent behaviour, then one becomes a SKILL of the other |
| a whole-body solve that is base placement + arm IK | **L2**; retire it when round 0 shows the extra dimension is not adopted, and let strategies require the components |
| a read-only channel | **L1 (read)**; it is not a REQUIRES edge of the L1 that reads it, only a dependency of an L2 that sequences it |
| an invariant or guard that never commands (arbitration, monitoring) | **L1 (read/decision)**, unclaimed if no task needs it |
| still ambiguous after these rules | take the stricter reading (more decomposition) and record the decision and reason |

Design gate: write the design table, run the four consistency checks, and
record decisions inline. Do not present open questions about labels; present
decisions with reasons. The graph is seeded only after the manager reviews the
design.

**Predicate coverage check (required before Phase G).** For every task in the
portfolio, list the quantity its success predicate reads, and name the
capability in the design that measures it. If no capability measures it, add
one (usually a read channel) or record it as a gap. In particular:

| task | predicate quantity | must be measurable by |
|---|---|---|
| 01 | box z lift, upright | object tracking (+ upright read) |
| 02 | control-panel joint angle | scene joint-state tracking |
| 03 | box z above table, not in table contact, gripper far | object tracking + contact sensing |
| 04 | shelf upright, shelf-target planar distance, shelf-box contact | scene geometry + contact sensing |
| 05 | drill z lift, not in table contact, grasped | object tracking + contact sensing |
| 06 | drill static, upright, gripper far, contact with target table | object tracking + scene geometry + contact sensing |
| 07 | box static, upright, contact with shelf board | object tracking + scene geometry + contact sensing |
| 08 | drill not in holder contact, grasped | contact sensing + scene geometry (holder) |
| 09 | drill z lift, not in table contact, grasped | object tracking + contact sensing |

Consequences: scene-level sensing (support tables, shelf board, target zone,
holder, obstacle) and scene joint-state sensing (a hinge angle) are capabilities
in their own right; if a task predicate reads them, they must exist as nodes,
not be deferred to a task program.

**Composition consistency check.** Match each L2 to the measured lesson:
placing uses the loaded-release law (open while supported), not the unloaded
open; a lift is a joint-space transport with the explicit closed-finger vector
held, not a base-height ramp; a push measures the task predicate (uprightness,
contact with its carrier), not only hand-object duty; a press measures the
panel joint angle, not only tool contact.

## Bootstrap: control vocabulary first

Before round 0, run a written reasoning pass over the robot's joint groups using
`docs/control_vocabulary.md` and the live model/registry. For each group answer:
what can be commanded, what physical effect each command has, what the task
portfolio needs from it, and what failure follows if the group has no verb.
Turn the answers into capability nodes and mark task demand separately.

There is no fixed capability count. The requirement is that the reasoning was
done, every commandable group has at least one verb, every measured failure
class maps to a verb or is recorded as a gap, and the unclaimed verbs are
listed.

Rule: when a measured failure names a link or a joint group, first check the
vocabulary for a posture or authority verb on that group before proposing a new
path or a new task-specific mechanism.

## Round 0 — calibration (mandatory, before any batch)

No capability may be proposed, implemented or validated before round 0
completes. Its job is to establish the control contract by measurement:

1. **Authority + gain table.** `docs/joint_authority.json` holds the structural
   `final_writer` of all 43 joints. The lower-body policy is written after the
   upper body, so a joint present in both groups is finally written by HOMIE v2
   (**lower wins**); the waist's default owner is `homie` even when
   `enable_waist=True`. The user authorized a runtime handover (2026-11-15,
   recorded in `experiments/<run>/config.yaml` `control.waist_owner_modes`):
   `api.authority(waist_owner='ik'|'blend')` changes the waist's final writer,
   and the default is restored on exit. Fill `measured_gain` with `track_gain`
   separately for each owner; store it next to authority, never instead of it.
   A high gain does not confer authority.
2. **Ramp-lag curve.** For joint deltas 0.1/0.3/0.5/1.0 rad, measure the ramp
   duration that reaches 95% of the command; store `ramp_s = f(delta)`. Use it
   instead of a constant ramp.
3. **Reach envelope per arm.** `ik_preview` (read-only) over a workspace grid
   for both arms; record the residual map and any left/right asymmetry.
4. **Standoff envelope per task.** Minimum reachable base standoff and the pad
   error there.

Write `experiments/<run>/calibration.json` and `docs/control_contract.md`.
Every later L1 cites the calibration for its authority and gain claims.

## Warm-up: substrate before discovery

Warm-up runs after calibration and before batch 1. It implements the control
SUBSTRATE only — the parts the master branch already proved. Task logic,
branch gates and per-task knobs stay in the discovery loop.

W1: implement the six substrate primitives in `warmup.substrate` with job type
`IMPLEMENT_SUBSTRATE`. You may read the `warmup.reference_sources` from the
master branch (`git show master:<path>`) for these primitives only. Each one
needs a contract, a micro-benchmark and a Validator verdict.

**Debugging harness.** Also implement `debug_visual`: `failure_window` finds
the onset and end timestep of a failure signature in the recorded trace, and
`stills` renders frames at explicit timesteps (`t=...`). An actor must detect
the window, render the onset and end frames, read the images, state what it
sees, and only then revise the skill or hypothesis. Record the onset/end steps
and the visual finding in the job report. Never read the success predicate.

**Authority handover.** Implement `authority_handover`: a runtime switch of a
control channel (waist owner `homie|ik|blend`, lower-body owner
`policy|hold`) that verifies the achieved gain before and after the switch,
records a HANDOFF event, and requires the balance metrics (tilt, walk error, no
fall) to stay in bounds. Handover and re-handover are both part of the
contract: the test must show the gain rises when the channel is handed over and
returns to the previous value when it is handed back.

W2: run `warmup.integration_benchmark` (05_drill_lift: reach -> close -> lift)
with the verified substrate. Pass = pad position error <= 0.015 m/side,
bilateral contact after close, object rise >= 0.05 m, no guard trip; on
failure, one named binding constraint.

Gate: the warm-up gate is **BLOCKING**. W2 is measured and **NOT closed**
(0/4); the batch counter stays 0 and no discovery batch starts. The missing
capability `collision_free_reach_trajectory` is a warm-up closure job labeled
**W2b**, run under the same discipline (one contract, a micro-benchmark, an
independent Validator), not batch 1. W2 is re-run with the verified
implementation and must pass before batch 1 may start; if it cannot pass, report
the measured binding constraint and keep the gate blocked. The gate predicate in
`experiments/<run>/config.yaml` is authoritative. Do not start discovery early;
the substrate is the floor the loop builds on.

## Control contract

An L1 is a control capability that verifies a physical outcome under declared
authority and constraints — not a function that attempts a motion. It declares:

- `commands`: joints it writes. Each must have `final_writer=upper_target` and
  `measured_gain >= min_gain_for_commands` in the registry.
- `observed_exogenous`: joints it does not command but reads and feeds into
  FK/Jacobian every control step (the waist **while `waist_owner=homie`**).
- `assumes_fixed`: joints assumed constant, with a measured drift bound.
  Excluding a joint from the IK variables does not mean it is still.
- `min_gain`: per commanded joint.
- `binding_constraint`: the single measured reason a candidate fails.

A skill that hands the waist over also declares `waist_owner`,
`waist_gain_before`, `waist_gain_after`, `handoff_events` (each HANDOFF event
with its owner and the restore) and `lower_owner`.

IK solves two palm poses (12-D) over the 14 arm joints, with the measured waist
exogenous while `waist_owner=homie`. **Default solver is mink QP** (`qp_reach`,
qpsolvers/daqp) over the arm joints (17-D, adding the three waist joints, only
while the waist is authorized to `ik`/`blend`), with joint limits, a base-lock
task and a posture task; DLS is the fallback only when the QP is infeasible,
for read-only previews, or for a diagnostic comparison the coordinator asks for
explicitly. Do not search joint vectors or stance tables. Ramp duration follows
the measured lag curve.

**Dimension is measured, not assumed.** The measured selection on the same
targets (`05_drill_lift` seeds 0-3) adopts **12-D / `waist_owner=homie`**: it
wins on EXECUTED pad error against both 17-D conditions on every seed and is the
only condition that met the 15 mm gate on any seed. 17-D under `ik` is
authority-compliant (handover verified before/after) but loses on achieved pad
error; 17-D under `homie` is the authority-negative control (HOMIE overwrites
the waist) and is recorded, never used. The 14-D base-height/yaw candidate was
eliminated in round 0. Selection is by executed pad/contract, not by model
residual.

**Concentrate.** Work one task's grasp at a time — 05_drill_lift first, then
generalize to 03/06/07/09. Do not spread a batch across six tasks that share
one blocker.

## Execution and validation

Select a batch → one Execution Actor per capability/job, in parallel waves →
one Validator per candidate → accept/restrict/reject/retire → update graph/registry. A candidate
is not verified because it ran. L1/L2 use **contract micro-benchmarks**, not
task success: servo tracking/convergence/joint margin, IK continuity, reach
accuracy, grasp contact/stability, hold drift; held-out scenarios. IK
convergence is not physical success.

**A batch is a set of capabilities, not one capability.** After selecting the
batch, include every capability that is ready now: dependencies verified, no
unresolved blocker. Dispatch all of them in parallel waves (one Execution Actor
per capability, up to the server limit per wave). Never leave servers idle while
an independent ready job is queued. Serialize only true dependency chains: from
each chain (`A → B → C`) put only the earliest unverified node in the current
batch. A one-job batch is valid only when nothing else is ready; state that
explicitly.

**Parallel servers per task.** A task is not limited to one execution actor.
Start extra server slots (`scripts/server_ctl.sh start <task> <seed> <slot>`,
then `micro/run_micro.py --slot <slot>`) and give each actor its own slot, so
independent jobs on the same task run concurrently. The host has 128 cores;
`servers_per_task` and `max_parallel_servers` in the config are the limits.
Keep slot 0 for the base track and stop slots when their wave ends.

If blocked, distinguish missing capability vs wrong implementation vs
frame/assumption vs expressivity vs WBC/IK limits vs evidence. Request
discovery only with a measured failure: signature, evidence, missing behavior,
expected reuse, why existing skills are insufficient.

## Promotion and reuse

New implementation = candidate. One consumer does not block discovery or
registration; one task success does not establish generality. Promote only
within independently verified conditions. Prefer parameterized mechanisms;
reuse before duplicating; merge only when contract, mechanism and behavior
justify it; never silently expand a verified scope. L2 exposes behavior-level
I/O, termination and failure. No hidden task-specific patches.

## Failure modes

1. **Discovery Deadlock** — no two-consumer gate; single-consumer discovery is
   allowed; promotion still requires validation.
2. **Library Pollution** — no duplicates or task-specific skills; held-out
   validation, contract-based merging and retirement.
3. **False Reuse** — call counts are not proof; track cost to first success,
   reliability, marginal new-task cost and cold/warm-start cost.

## Loop and report

INITIALIZE: config, round 0 calibration, fresh registry, all tasks, at least
two strategies per task, inferred graph, mark unverified. BATCH: summarize
gaps/verified/failures and the calibration; spawn both; merge proposals;
execution per capability; validate; sweep; checkpoint. Use subgraphs; do not
pass the full DB or history.

Report: Explore proposal / Exploit proposal / Selected+reason / result /
verified change / cost / next bottleneck. Measured only.

## Constraints

No physics/reward/predicate/model-limit changes; predicate coordinator-only; no
demos or pretrained task skills; rollouts/traces allowed;
`use_legacy_library: false`; preserve code/params/seeds/versions; do not assume
IK solves force/contact/WBC limits; spawn both proposal actors every batch.

First actions: load config; confirm fresh; inspect tasks/APIs; infer strategies
and capabilities; spawn both; select; execute.

Guiding principle: **discover the smallest useful set of control capabilities,
verify independently, reuse to make future humanoid loco-manipulation tasks
cheaper.**
