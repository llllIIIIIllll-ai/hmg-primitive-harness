# HMG Primitive Discovery Harness

A harness for discovering reusable whole-body skills on the
[HumanoidMimicGen](https://github.com/NVlabs/humanoidmimicgen) (HMG) benchmark.
A coordinator spawns fresh-context actors per task; actors write primitives,
verify them on seeds, and promote the reusable ones into a skill catalogue.
Task solutions are short compositions of those skills.

The benchmark's success predicate is owned by HMG and is never exposed to
actors. The harness only records it for the coordinator.

## What is in this repository

| Path | Contents |
|---|---|
| `hmg_server.py` | One persistent MuJoCo server per task, driven through a file queue |
| `hmg_tools.py` | The tool API used by primitives (observe, step, reach, grip, probes) |
| `hmg_render.py` | Offscreen stills / mp4 rendering |
| `hmg_harness/` | Adapter, observation/recording, primitive registry and composition |
| `library/skills/` | Skill descriptions: signature, preconditions, contract, scope, cautions |
| `library/evidence/` | Proof history for each skill |
| `library/CORRECTIONS.md`, `SUPERSEDED.md`, `LESSONS.md`, `NOT_CATALOGUED.md` | Corrections, retired entries, cross-task lessons, rejected candidates |
| `docs/PRIMITIVES.md` | Skill-catalogue system prompt and compact index |
| `docs/PROTOCOL.md` | Queue protocol, tool schemas and hard rules |
| `docs/DEBUG_TOOLS.md` | Debug tools, their costs and known blind spots |
| `docs/coordinator_skill.md` | Coordinator instructions for Claude Code |
| `docs/coordinator_prompt.txt` | Standalone coordinator prompt |
| `examples/compositions/` | Canonical task compositions for 02/03/04/06/08 |
| `docs/HMG_harness_overview.tex` | Beamer overview (compile with XeLaTeX) |

Large local artefacts (`queue/`, `out/`, `.coord/`, `.scratch/`, task working
directories and videos) are intentionally excluded.

## Requirements

- Linux, Python **3.10**
- The pinned packages in `requirements.txt` (MuJoCo 3.2.6, robosuite 1.5.1,
  PyTorch/ONNX Runtime for the WBC policy)
- The HMG benchmark checkout: <https://github.com/NVlabs/humanoidmimicgen>
- Optional: an NVIDIA GPU for EGL rendering; the simulation itself is CPU-based

## Setup

```bash
git clone https://github.com/llllIIIIIllll-ai/hmg-primitive-harness.git hmg_primitive_harness
cd hmg_primitive_harness
bash scripts/bootstrap.sh
source env.sh
```

`bootstrap.sh` creates `.venv`, installs `requirements.txt`, clones the HMG
benchmark into `third_party/humanoidmimicgen`, installs it editable, and
downloads the separately licensed lower-body WBC policies. Set `PYTHON` to a
Python 3.10 interpreter if it is not on `PATH`.

## Quick start

```bash
# One server per task, one file queue per server.
scripts/server_ctl.sh start 02_push_button 0

# Wait for the "server ready" line in .scratch/srv_02_push_button_*.log.
HMG_TASK=02_push_button scripts/qtool.sh '{"tool":"observe"}'

# Run a registered primitive over several seeds.
HMG_TASK=02_push_button scripts/qtool.sh \
  '{"tool":"run_seeds","name":"walk_to_reach","seeds":[0,1,2],"source":"<python source>"}'
```

Stop servers with `scripts/server_ctl.sh stop [task]` and check them with
`status`.

## Composition

A primitive is a Python function `def run(api, **kwargs)`. It may call another
registered primitive:

```python
def run(api, **kwargs):
    walk = api.skill("walk_to_reach", target_match="button", gapx_max=0.44)
    target = walk["target_pos"]
    return api.skill("servo_fingertip_to_point", side="right", target=target)
```

`api.skill` resolves the name at call time and is depth-capped to prevent
cycles. Physics, reward, the success predicate and model limits are never
modified by a skill. A task program keeps the final judgement outside the
skills.

## Tasks

| ID | Task | Goal |
|---|---|---|
| 01 | `box_lift_floor` | Lift the box from the floor |
| 02 | `push_button` | Press the red button |
| 03 | `box_lift` | Lift the box |
| 04 | `push_shelf_forward` | Push the shelf with the box to the marked area |
| 05 | `drill_lift` | Pick up the drill |
| 06 | `drill_pnp` | Move the drill from one table to the other |
| 07 | `box_table_to_shelf` | Place the box on the shelf |
| 08 | `pick_drill_from_holder` | Pick the drill from the standing-height holder |
| 09 | `obstacle_aware_pick_drill` | Pick up the drill while avoiding the obstacle |

## Skill catalogue

The catalogue currently contains **40 skills**, grouped by primary task:

- 02: `walk_to_reach`, `servo_fingertip_to_point`, `press_until_joint_stop`
- 03: `approach_standoff`, `reach_opposed_faces`, `align_pads_to_faces`, `limit_safe_close`, `shoulder_lift_hold`, `arm_ramp`
- 04: `measure_scene_state`, `walk_to_first_contact`, `steer_behind_leader`, `push_object_to_target_x`, `hold_commanded_pose`
- 06: `reach_palm_to_object_standoff`, `park_hand_clear_of_object`, `curl_fingers_onto_handle`, `hoist_gripped_object_in_steps`, `carry_gripped_object_to_pose`, `level_upright`, `release_grip_and_settle_in_place`, `retreat_hands_radially_from_object`, `hold_parked_arms_and_measure_drift`
- 08: `walk_base_to_coordinate`, `servo_hand_site_to_point`, `creep_to_object_standoff`, `move_held`, `boost_base_height_holding_command`
- 01: `servo_base_to_target_pose`, `place_pads_branch`, `reach_attitude_guard`, `seat_pads_for_grip`, `close_fingers_flip_left`, `secure_grip_after_close`, `stand_lift_box`, `grasp_lift_floor_box_press_branch`
- 05: `grasp_verify`, `lift_drill_pad_route_geom_close`
- 07: `carry_and_seat_on_shelf`
- 09: `top_up_reach_at_standoff`

Cross-task reuse: `servo_base_to_target_pose` (01, 06), `arm_ramp` (03, 05)
and `walk_base_to_coordinate` (08, 09). `move_held` and `level_upright` are
merged skills whose variants were retired only after their union of scopes
reproduced the originals.

## Evaluation note

The HMG benchmark defines success. The harness reads the engine bit at the end
of an episode for its own bookkeeping. That is stricter than the official
evaluation, which ORs the predicate over every step: a transient success
followed by a drop counts as a failure here but as a success in the official
metric. The official protocol also uses seeds 0-99 and a step limit; compare
success rates only after re-running with the same protocol.

## Licenses and attribution

This repository does not vendor the HMG benchmark. Clone
[NVlabs/humanoidmimicgen](https://github.com/NVlabs/humanoidmimicgen) and follow
its Apache-2.0 license and notices. The lower-body WBC policies are downloaded
by `python -m humanoidmimicgen.download_wbc_policies` and are separately
licensed by their upstream project; review those terms before redistribution.
