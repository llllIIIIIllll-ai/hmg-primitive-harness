# Control vocabulary (bootstrap reconnaissance)

Read-only reference for the coordinator. Before any strategy work, enumerate
what the robot can actually be commanded to do, joint group by joint group, and
seed the capability graph with every atomic verb — not only the verbs a task
happens to mention. A verb that is missing from this table cannot be proposed
later, which is how `hand_pre_shape` was missed in ll_v2.

## The 43 DoF and who can command them

| group | dof | authority | atomic verbs (capability candidates) |
|---|---|---|---|
| legs | 12 | HOMIE v2 (WBC) | none directly; locomotion through `navigate_cmd`, height through `base_height_command` |
| waist | 3 | HOMIE v2 by default; runtime handover allowed | torso-pose request (advisory), authority handover to IK/blend, waist pose when owned by IK |
| arms | 14 | upper target (IK) | pose servo (6-D per hand), joint-space ramp/transport, constrained servo (clearance or joint-margin bound), contact push |
| hands | 14 | upper target | posture preset (pre-shape), contact-triggered close, force/drift-limited close, contact-triggered open, loaded release |
| base | floating | nav + height channels | navigate to pose, navigate to object-relative standoff, height setpoint (closed loop) |
| whole body | 43 | composed | task-vector solve (12-D/17-D), authority handover, arbitration of which controller writes which joint |

## Bootstrap reasoning (do this, do not copy a list)

The bootstrap is a thinking pass, not a catalogue import. For each joint group
in the table above, answer these questions in writing, then turn the answers
into capability nodes:

1. **What can this group be commanded to do at all?** Name the primitive
   commands the interface exposes (position target, velocity, posture, none).
2. **What physical effect does each command have?** Move, hold, contact, push,
   release. If a command exists but no verb names its effect, that is a gap.
3. **What would a task family need from this group?** Derive from the portfolio,
   not from one task: reach, seat, grasp, lift, carry, place, push, press.
4. **What failure would this group cause if it had no verb?** (e.g. fingers
   without a pre-shape verb push the object during the approach). A failure you
   have measured and cannot name a verb for is the strongest signal.
5. **Which verbs are claimed by a strategy and which are unclaimed?** An
   unclaimed verb is still a capability; mark demand separately.

Completeness check before leaving bootstrap:
- every commandable joint group has at least one verb;
- every measured failure class maps to a verb or is recorded as an explicit gap;
- the unclaimed-verb list is non-empty or explained.

Do not require a fixed number of capabilities. Require that the reasoning above
was done and that the graph reflects it.

## Level rules (L1 vs L2) — apply these mechanically

**L1 = one actuator or read channel, one closed-loop law, one contract.**
Feedback inside that same channel is allowed and still L1: a pose loop that
turns a pose error into velocity commands is one verb. An L1 has no REQUIRES
edges (or only to read-only tracking it consumes), and it never sequences a
second control verb.

**L2 = two or more distinct L1s composed in time, or a selection/switching
policy between L1s.** An L2 must have at least two REQUIRES edges to L1s. If a
candidate has one dependency, it is a parameterised L1, not an L2. If it has
none, it is an L1.

**Label consistency checks** (run before writing the graph):
1. Every L1 node has zero REQUIRES edges to other capabilities.
2. Every L2 node has >= 2 REQUIRES edges and names them in its spec.
3. A capability whose name is one verb ("navigate", "servo", "close", "hold")
   is L1 unless it explicitly switches between different channels.
4. A capability that selects a target from another capability's output, or
   changes behaviour when a condition is met, is L2.

### Worked level examples

| candidate | level | why |
|---|---|---|
| `base_velocity_command` | L1 | one channel (nav velocity), one command |
| `base_navigate_to_pose` | **L1** | still one channel: a pose error is closed into velocity commands. It is not L2 just because it has feedback. |
| `base_height_setpoint` | L1 | one scalar channel, closed loop |
| `ee_pose_ik` | L1 | one channel (arm joint targets), one solve |
| `hand_pre_shape` | L1 | one channel (finger joints), one posture command |
| `grasp_hold` | L1 | one channel (finger joints), hold the last closed target |
| `object_pose_tracking` | L1 (read) | one read channel, no command |
| `base_navigate_to_standoff` | **L2** | composes nav + object tracking + safety monitor (>= 2 distinct L1s) |
| `collision_free_reach_trajectory` | **L2** | composes IK + tracking + constrained servo + monitor |
| `bimanual_grasp_object` | **L2** | pre-shape + two-pad IK + contact close, sequenced |

Anti-patterns: calling a feedback loop "L2"; calling a two-step sequence "L1";
giving an L2 one dependency; putting a task constant into either level.

## Worked examples (format guidance, not a catalogue)

Five examples of the reasoning format. They show how to go from a joint group to
a verb, an effect, a portfolio need and a failure. Derive the rest the same way;
do not treat these as the full list.

| group | command | physical effect | portfolio need | failure without the verb |
|---|---|---|---|---|
| hands | finger posture preset before contact | choose aperture/opposition while the hand approaches | every grasp, and the press task | the fingers (e.g. the thumb link) push the object during the approach, before any close |
| arms | Cartesian servo with a clearance bound | move the palms while keeping every robot link outside the live object AABB | grasp and extraction tasks | the approach penetrates or topples the object even though the pads are clear |
| hands | explicit closed-finger target held every control step | keep the grasp load through motion | every lift and carry | the grip decays to the measured finger positions and the object is released |
| base | height scalar, closed loop | set a posture height (squat/stand), not a lift | floor-level reach | floor objects stay out of reach; using the height channel as a lift plateaus short |
| sensing | live pose/AABB read each step | targets re-derived from the current object pose | every task | targets go stale, the pad gate fails, and a moving object is chased with old numbers |

Format rules: one row = one verb; name the group, the command, the physical
effect, the portfolio need and the measured or expected failure. If you cannot
name the failure, the verb is probably a composition in disguise.

## Bootstrap requirement

1. Produce this table from the live model and the registry, not from memory.
2. Every verb row becomes a CAPABILITY node, even if no current task requires
   it. Mark task demand separately (REQUIRES edges) so coverage does not hide
   an unclaimed verb.
3. EXPLORE must consider the verbs that no strategy claims; a verb with zero
   consumers is still a legitimate target when a measured failure names it.
4. When a failure names a non-pad link (e.g. a thumb link pushing the object),
   first check the posture verb for that link before planning a path around it.

## Anti-pattern

Deriving capabilities only from task strategies. That yields task verbs
(close, lift, push) and misses control verbs (pre-shape, constrained servo,
joint-space transport), which is exactly the ll_v2 gap.
