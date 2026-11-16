# ll_v3 Phase G — graph seed report

Status: **graph seeded from the reviewed Phase D design; round 0 NOT started;
warm-up and batches NOT started.** Batch counter 0, round counter 0.

- Source design: `work/ll_v3/reports/capability_design.md`
- Graph DB: `experiments/ll_v3/skill_system.db`
- Seed driver: `work/ll_v3/seed_graph.py` (typed `graph_cli.py` commands only)
- Check driver: `work/ll_v3/check_graph.py`
- Machine-readable checks: `work/ll_v3/reports/graph_seed_checks.json`
- Checkpoint: `experiments/ll_v3/checkpoints/phase_g.db`

## 1. What was run

```
python3 work/ll_v3/seed_graph.py
python3 work/ll_v3/check_graph.py
python3 graph_cli.py --db experiments/ll_v3/skill_system.db checkpoint --name phase_g
python3 graph_cli.py --db experiments/ll_v3/skill_system.db coverage
```

The seeder created 33 `CAPABILITY` nodes, ensured every L2's `REQUIRES` edges,
and created 18 `STRATEGY` nodes (two per task) with their direct `REQUIRES`
edges. No direct SQLite writes were used.

## 2. Node counts

| type | count | notes |
|---|---:|---|
| `TASK` | 9 | portfolio tasks |
| `CAPABILITY` | 33 | all status `proposed` (unverified) |
| `STRATEGY` | 18 | two per task |
| `SKILL` | 0 | none implemented yet (expected at Phase G) |

### Capabilities by level and kind

| level | kind | count |
|---|---|---:|
| L1 | control | 15 |
| L1 | read | 6 |
| L1 | decision | 1 |
| L2 | composition | 11 |
| **total** | | **33** |

The design's "L1 (read)" / "L1 (read/decision)" labels are stored as
`level: "L1"` plus `kind: "read"` / `"decision"`. This is a representation
normalization so the level check is uniform; it is not a semantic mismatch.

### Capability inventory (as seeded)

| capability | level | kind | spec status | demand |
|---|---|---|---|---|
| `base_velocity_command` | L1 | control | discovery | 9/9 (via standoff/04) |
| `base_navigate_to_pose` | L1 | control | discovery | 9/9 |
| `base_height_setpoint` | L1 | control | discovery | 1/9 (01) + optional reach |
| `waist_pose_servo` | L1 | control | discovery | 0/9 direct (conditional) |
| `ee_pose_ik` | L1 | control | substrate (arm_pose_ik) | 9/9 |
| `ee_position_servo` | L1 | control | discovery | 9/9 |
| `joint_space_transport` | L1 | control | discovery | 6/9 |
| `constrained_servo` | L1 | control | discovery (W2b component) | 9/9 (via C27) |
| `contact_push` | L1 | control | discovery | 2/9 (02, 04) |
| `hand_pre_shape` | L1 | control | discovery | 9/9 |
| `grasp_close_contact` | L1 | control | substrate (hand_close_contact) | 7/9 |
| `grasp_close_force_limited` | L1 | control | competing | 0/9 required; 05 S2 competing candidate |
| `grasp_hold` | L1 | control | discovery | 7/9 |
| `grasp_open_release` | L1 | control | substrate (hand_open) | 0/9 (unclaimed) |
| `loaded_release` | L1 | control | discovery | 2/9 (06, 07) |
| `object_pose_tracking` | L1 | read | substrate (object_pose_track) | 9/9 |
| `scene_geometry_tracking` | L1 | read | discovery | 9/9 |
| `scene_joint_state_tracking` | L1 | read | discovery | 1/9 (02) |
| `contact_sensing` | L1 | read | discovery | 9/9 |
| `grasp_state_tracking` | L1 | read | discovery | 7/9 |
| `safety_limit_monitor` | L1 | decision | substrate (safety_guard) | 9/9 |
| `failure_visualization` | L1 | read | substrate (debug_visual) | 0/9 (debug) |
| `base_navigate_to_standoff` | L2 | composition | substrate (base_standoff_servo) | 9/9 |
| `control_authority_handover` | L2 | composition | substrate (authority_handover) | 0/9 direct (warm-up gate) |
| `whole_body_ik_17d` | L2 | composition | candidate | 0/9 (candidate) |
| `whole_body_ik_14d` | L2 | composition | candidate | 0/9 (candidate) |
| `collision_free_reach_trajectory` | L2 | composition | W2b closure (missing W2 capability) | 9/9 |
| `bimanual_grasp_object` | L2 | composition | discovery | 7/9 |
| `lift_object` | L2 | composition | discovery | 6/9 |
| `place_object` | L2 | composition | discovery | 2/9 (06, 07) |
| `push_object_to_target` | L2 | composition | discovery | 1/9 (04) |
| `press_panel_button` | L2 | composition | discovery | 1/9 (02) |
| `extract_object_from_holder` | L2 | composition | discovery | 1/9 (08) |

`spec_json` carries the level, kind, group, channel/law or read channel,
contract, `commands`, `observed_exogenous`, `assumes_fixed`, `min_gain`,
`binding_constraint`, demand, status, notes, and (for L2) `requires`,
`gate`/`retire_condition` where applicable. `grasp_state_tracking` also carries
`proxy_for: "benchmark IsGrasped (not exposed)"` and `gap: "G1"`.

## 3. Edge counts

| relation | count | composition |
|---|---:|---|
| `REQUIRES` | 187 | 47 capability→capability (L2 deps) + 140 strategy→capability |
| `HAS_STRATEGY` | 18 | 9 tasks × 2 strategies |
| **total** | **205** | |

The 47 L2→capability edges are exactly the design's L2 `REQUIRES` sets:

| L2 capability | total REQUIRES | of which L1 | requires |
|---|---:|---:|---|
| `base_navigate_to_standoff` | 3 | 3 | `base_velocity_command`, `object_pose_tracking`, `safety_limit_monitor` |
| `control_authority_handover` | 3 | 3 | `waist_pose_servo`, `base_velocity_command`, `safety_limit_monitor` |
| `whole_body_ik_17d` | 2 | 2 | `ee_pose_ik`, `waist_pose_servo` |
| `whole_body_ik_14d` | 3 | 3 | `base_navigate_to_pose`, `base_height_setpoint`, `ee_pose_ik` |
| `collision_free_reach_trajectory` | 4 | 4 | `ee_pose_ik`, `object_pose_tracking`, `constrained_servo`, `safety_limit_monitor` |
| `bimanual_grasp_object` | 3 | 3 | `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact` |
| `lift_object` | 7 | 6 | `bimanual_grasp_object`, `joint_space_transport`, `grasp_hold`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking` |
| `place_object` | 7 | 7 | `joint_space_transport`, `grasp_hold`, `loaded_release`, `ee_position_servo`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing` |
| `push_object_to_target` | 6 | 6 | `contact_push`, `base_velocity_command`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `safety_limit_monitor` |
| `press_panel_button` | 4 | 4 | `contact_push`, `contact_sensing`, `scene_joint_state_tracking`, `safety_limit_monitor` |
| `extract_object_from_holder` | 5 | 4 | `bimanual_grasp_object`, `ee_position_servo`, `grasp_hold`, `contact_sensing`, `scene_geometry_tracking` |

`lift_object` and `extract_object_from_holder` each have one L2 dependency
(`bimanual_grasp_object`) and at least four L1 dependencies, so every L2 still
has >= 2 L1 dependencies. No L2 is a leaf.

## 4. Strategies (two per task)

| task | strategy | direct REQUIRES | direct capabilities |
|---|---|---:|---|
| 01_box_lift_floor | `floor_standoff_grasp_lift` | 4 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `bimanual_grasp_object`, `lift_object` |
| 01_box_lift_floor | `floor_crouch_direct_grasp_lift` | 12 | `base_navigate_to_pose`, `base_height_setpoint`, `ee_position_servo`, `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact`, `joint_space_transport`, `grasp_hold`, `object_pose_tracking`, `contact_sensing`, `grasp_state_tracking`, `safety_limit_monitor` |
| 02_push_button | `standoff_reach_press` | 4 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `hand_pre_shape`, `press_panel_button` |
| 02_push_button | `direct_cartesian_press` | 7 | `base_navigate_to_pose`, `ee_position_servo`, `contact_push`, `scene_joint_state_tracking`, `contact_sensing`, `scene_geometry_tracking`, `safety_limit_monitor` |
| 03_box_lift | `table_standoff_grasp_lift` | 4 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `bimanual_grasp_object`, `lift_object` |
| 03_box_lift | `table_direct_grasp_lift` | 12 | `base_navigate_to_pose`, `ee_position_servo`, `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact`, `joint_space_transport`, `grasp_hold`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking`, `safety_limit_monitor` |
| 04_push_shelf_forward | `standoff_push_to_target` | 4 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `hand_pre_shape`, `push_object_to_target` |
| 04_push_shelf_forward | `walk_and_push_tracking` | 8 | `base_navigate_to_pose`, `base_velocity_command`, `ee_position_servo`, `contact_push`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `safety_limit_monitor` |
| 05_drill_lift | `drill_standoff_grasp_lift` | 4 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `bimanual_grasp_object`, `lift_object` |
| 05_drill_lift | `drill_direct_grasp_lift` | 13 | `base_navigate_to_pose`, `base_height_setpoint`, `ee_position_servo`, `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact`, `joint_space_transport`, `grasp_hold`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking`, `safety_limit_monitor` |
| 06_drill_pnp | `drill_pick_lift_place` | 5 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `bimanual_grasp_object`, `lift_object`, `place_object` |
| 06_drill_pnp | `drill_pick_transport_release` | 13 | `base_navigate_to_pose`, `ee_position_servo`, `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact`, `joint_space_transport`, `grasp_hold`, `loaded_release`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking`, `safety_limit_monitor` |
| 07_box_table_to_shelf | `box_pick_lift_place_board1` | 5 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `bimanual_grasp_object`, `lift_object`, `place_object` |
| 07_box_table_to_shelf | `box_direct_transport_release` | 13 | `base_navigate_to_pose`, `ee_position_servo`, `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact`, `joint_space_transport`, `grasp_hold`, `loaded_release`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking`, `safety_limit_monitor` |
| 08_pick_drill_from_holder | `holder_standoff_grasp_extract` | 4 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `bimanual_grasp_object`, `extract_object_from_holder` |
| 08_pick_drill_from_holder | `holder_direct_grasp_retract` | 10 | `base_navigate_to_pose`, `ee_position_servo`, `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact`, `grasp_hold`, `contact_sensing`, `scene_geometry_tracking`, `grasp_state_tracking`, `safety_limit_monitor` |
| 09_obstacle_aware_pick_drill | `obstacle_standoff_clearance_grasp_lift` | 4 | `base_navigate_to_standoff`, `collision_free_reach_trajectory`, `bimanual_grasp_object`, `lift_object` |
| 09_obstacle_aware_pick_drill | `obstacle_direct_clearance_grasp_lift` | 14 | `base_navigate_to_pose`, `base_height_setpoint`, `ee_position_servo`, `constrained_servo`, `hand_pre_shape`, `ee_pose_ik`, `grasp_close_contact`, `joint_space_transport`, `grasp_hold`, `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking`, `safety_limit_monitor` |

The strategy names match the design's Section 10 names exactly. The strategy
direct sets match the design's sketches; the transitive closure adds the L2
dependencies.

## 5. Graph-side consistency checks

All checks were run against the seeded graph through `graph_cli.py export` and
are recorded in `work/ll_v3/reports/graph_seed_checks.json`.

| check | result | detail |
|---|---|---|
| Every L1 has zero REQUIRES edges | **PASS** | 0 violations among 22 L1 nodes |
| Every L2 has >= 2 REQUIRES | **PASS** | 11/11; minimum 2 (`whole_body_ik_17d`) |
| Every L2 has >= 2 L1 dependencies | **PASS** | 11/11; minimum 2 (`whole_body_ik_17d`) |
| Every L2 names its REQUIRES in `spec_json` | **PASS** | edge targets == `spec.requires` for all 11 |
| Every capability spec carries the required contract fields | **PASS** | 33/33 field audit: L1 control has `commands`, `observed_exogenous`, `assumes_fixed`, `min_gain`, `binding_constraint`; L1 read/decision has the read fields and empty command fields; L2 has `requires` and `binding_constraint` |
| No capability lacks a level | **PASS** | 33/33 have `level` in `spec_json` |
| Every task has two strategies | **PASS** | 9/9 tasks have exactly 2 `HAS_STRATEGY` edges |
| Predicate-coverage mapping resolves | **PASS** | see Section 6 |
| Strategy direct sets match the design | **PASS** | 18/18 |
| L2 REQUIRES sets match the design | **PASS** | 11/11 |
| No REQUIRES cycle | **PASS** | capability subgraph is acyclic |

## 6. Predicate-coverage resolution against the seeded nodes

| task | required read capability/capabilities | reachable from the task's strategies? |
|---|---|---|
| 01_box_lift_floor | `object_pose_tracking` | yes |
| 02_push_button | `scene_joint_state_tracking` | yes |
| 03_box_lift | `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing` | yes |
| 04_push_shelf_forward | `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing` | yes |
| 05_drill_lift | `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking` | yes |
| 06_drill_pnp | `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing` | yes |
| 07_box_table_to_shelf | `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing` | yes |
| 08_pick_drill_from_holder | `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking` | yes |
| 09_obstacle_aware_pick_drill | `object_pose_tracking`, `scene_geometry_tracking`, `contact_sensing`, `grasp_state_tracking` | yes |

The five predicate-read nodes (`object_pose_tracking`, `scene_geometry_tracking`,
`scene_joint_state_tracking`, `contact_sensing`, `grasp_state_tracking`) all
exist in the graph and are reachable from every task that needs them. The
exact `IsGrasped` predicate is still not exposed; `grasp_state_tracking` remains
the proxy recorded as gap G1 in the design.

### Transitive closure size per task

| task | capabilities reached |
|---|---:|
| 01_box_lift_floor | 19 |
| 02_push_button | 15 |
| 03_box_lift | 18 |
| 04_push_shelf_forward | 14 |
| 05_drill_lift | 19 |
| 06_drill_pnp | 20 |
| 07_box_table_to_shelf | 20 |
| 08_pick_drill_from_holder | 17 |
| 09_obstacle_aware_pick_drill | 19 |

## 7. Unclaimed capabilities

These nodes have no incoming path from any task strategy. They are intentionally
seeded and remain legitimate targets; the design's dispositions are unchanged.

| capability | level | reason / disposition |
|---|---|---|
| `grasp_close_force_limited` | L1 | competing stopping law to `grasp_close_contact`; run both in one slot when a failure names grip force/slip; merge only if the Validator shows mechanism-equivalent behaviour |
| `grasp_open_release` | L1 | unloaded open; no current task uses it (placement uses `loaded_release`); retained for failed/unsupported releases and future drop tasks |
| `waist_pose_servo` | L1 (gated) | no task strategy requires it under the adopted 12-D / `waist_owner=homie` default; it is the channel the handover switches to |
| `control_authority_handover` | L2 | required by the warm-up gate, not by any task strategy under the default; the only legal route to `waist_pose_servo` |
| `whole_body_ik_17d` | L2 | candidate; retire if round 0 does not adopt 17-D |
| `whole_body_ik_14d` | L2 | candidate; retire if round 0 does not adopt 14-D |
| `failure_visualization` | L1 (read) | debug-only actor tool; never reads the success predicate |

The task-reachable union is 26 capabilities; 26 + 7 unclaimed = 33. The
unclaimed list matches the design exactly.

## 8. Coverage snapshot

`graph_cli.py coverage` (read-only) reports:

| task | strategies | covered | total |
|---|---|---:|---:|
| 01_box_lift_floor | 2 | 0 | 19 |
| 02_push_button | 2 | 0 | 15 |
| 03_box_lift | 2 | 0 | 18 |
| 04_push_shelf_forward | 2 | 0 | 14 |
| 05_drill_lift | 2 | 0 | 19 |
| 06_drill_pnp | 2 | 0 | 20 |
| 07_box_table_to_shelf | 2 | 0 | 20 |
| 08_pick_drill_from_holder | 2 | 0 | 17 |
| 09_obstacle_aware_pick_drill | 2 | 0 | 19 |

All capabilities are `proposed` and no `SKILL` nodes exist, so covered = 0 for
every task. This is expected at Phase G; verification happens in round 0 /
warm-up / batches. The uncovered union is the 26 task-reachable capabilities.

## 9. Design-vs-graph mismatches

**No semantic mismatch.** Every capability name, level, L2 dependency and
strategy direct set matches the reviewed design; the checks above are the
evidence. The following are representation notes, not mismatches:

1. The design's "L1 (read)" / "L1 (read/decision)" labels are stored as
   `level: "L1"` plus `kind: "read"` / `"decision"`, so the level check is
   uniform. Counts: 15 L1 control + 6 L1 read + 1 L1 decision + 11 L2 = 33.
2. `propose_strategy` does not accept a `spec_json`, so strategy nodes store
   `spec_json: "{}"`. The strategy description holds the chain and the
   `REQUIRES` edges are the machine-readable content. The direct sets were
   checked against the design and match.
3. Capability node status is `proposed` for all 33 (unverified), as expected.
   The design's `substrate` / `discovery` / `W2b` / `candidate` / `competing` /
   `debug` labels are recorded in `spec_json.status`, not in the node status.
4. `grasp_close_force_limited` has no strategy edge, so it remains unclaimed;
   this matches the design's "competing candidate" disposition.
5. `whole_body_ik_17d` and `whole_body_ik_14d` are seeded as candidates with
   `retire_condition` in `spec_json`; they are not required by any strategy,
   matching the design.
6. The design's C01–C33 IDs are documentation handles; the graph keys on the
   capability names. The mapping is the design table.
7. The design's demand annotations (e.g. "9/9 via standoff/04") are metadata,
   not edges; the graph's transitive coverage reproduces the same reachability.

## 10. State and next turn

- Batch counter: **0**. Round counter: **0**.
- Checkpoint `phase_g.db` written after the seed.
- Round 0 was **not** started. Warm-up and batches were **not** started.
- The config's round 0 now includes the **grasp-proxy check**:
  `grasp_state_tracking` vs the benchmark's own grasp outcome on a few
  episodes, recorded as a correlation with its failure boundary (`IsGrasped` is
  not exposed; the proxy must be validated, not assumed). This is a next-turn
  round-0 item and was not run here.
- Round 0's other mandatory items remain pending: authority + measured gain per
  joint, ramp-lag curve, reach envelope per arm, standoff envelope per task, and
  the IK dimension A/B (12/14/17), followed by
  `experiments/ll_v3/calibration.json` and `docs/control_contract.md`.

**Stop.** The graph is seeded and checked; the next action is round 0, which
requires a separate instruction.
