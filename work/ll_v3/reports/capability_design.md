# ll_v3 Phase D — capability design (no graph writes)

Status: **design complete; graph NOT seeded; round 0 / warm-up / batch NOT started.**

This is the Phase D deliverable required by `experiments/ll_v3/config.yaml` and
`prompts/coordinator.md`. It is a written reasoning pass over the **live model**
and the **authority registry**, not a catalogue import. The graph still contains
only the nine `TASK` nodes (`graph_cli.py coverage` shows 0 strategies and 0
capabilities for every task).

## 0. Provenance and scope

Read (clean-room scope only):

- `experiments/ll_v3/config.yaml`, `prompts/coordinator.md`
- `docs/control_vocabulary.md`, `docs/control_parameterization.md`,
  `docs/joint_authority.json`, `docs/task_success_criteria.md`,
  `docs/PROTOCOL.md`
- live simulator/model through the harness queue
- `work/ll_v3/reports/raw/*.json` (probe output written by this pass)

Live probes were run on **all nine portfolio tasks, seed 0**, one server per
task, and the servers were stopped after the pass. Every probe is read-only
(`probe` calls `mj_forward`, never `mj_step`):

```
HMG_VENV=/home/pjy0422/hmg_migration/stack/hmg_venv \
HMG_REPO=/home/pjy0422/hmg_migration/stack/humanoidmimicgen_repo \
scripts/server_ctl.sh start <task> 0 0

HMG_TASK=<task> scripts/qtool.sh '{"tool":"probe","what":"scene","detail":true}'
HMG_TASK=<task> scripts/qtool.sh '{"tool":"probe","what":"bodies","match":"..."}'
HMG_TASK=<task> scripts/qtool.sh '{"tool":"probe","what":"joints"}'
HMG_TASK=<task> scripts/qtool.sh '{"tool":"probe","what":"actuators"}'
HMG_TASK=<task> scripts/qtool.sh '{"tool":"probe","what":"hands"}'
HMG_TASK=<task> scripts/qtool.sh '{"tool":"probe","what":"state"}'
```

Raw evidence: `work/ll_v3/reports/raw/` (`*_scene.json`, `bodies/*.json`,
`robot_joints.json`, `robot_actuators.json`, `robot_hands.json`,
`robot_state.json`). The report quotes measured values from those files.

No subagents were spawned for Phase D: this is the coordinator's own design
pass, and the manager explicitly asked to stop before graph seeding, round 0,
warm-up or any batch. (Standing rule for later phases: every subagent is
`subagent_type: "hmg-worker"` with only its job packet.)

---

## 1. Live model and registry inventory

### 1.1 Robot (live, task 05 seed 0; identical robot in every task)

- **43 DoF, 43 actuators** (`probe actuators` n = 43), one actuator per joint.
- **45 model joints** in a task with one movable object: `robot0_base` (free) +
  43 robot joints + one task-object free joint (`<object>_joint0`, qposadr 50
  in the sampled tasks). Task 02 adds the two panel joints.
- Joint naming: body joints are `robot0_<name>_joint`; hand joints are
  `gripper0_{left,right}_<name>_joint`.

| group | dof | default `final_writer` | measured gain (live registry) | commandable through the 31-D upper channel? |
|---|---:|---|---|---|
| legs | 12 | `homie_v2` | `null` (WBC only) | **no** — never command hip/knee/ankle |
| waist | 3 | `homie_v2` | homie: pitch 0.137, roll -0.026, yaw 1.137; ik: 1.0 / 0.998 / 1.0; re-handover: 0.139 / -0.062 / 1.135 | **conditional** — only after an authorized `waist_owner=ik|blend` handover |
| arms | 14 | `upper_target` | 0.95–0.997 (every joint >= 0.8) | **yes** |
| hands | 14 | `upper_target` | 0.856–0.863 (every joint >= 0.8; one-sided) | **yes** |
| base | 1 free | nav + height channels | n/a | through `navigate_cmd` (3) + `base_height` (1), never a joint target |

`min_gain_for_commands = 0.8` (config). The waist's high *upper-channel response*
under `homie` is **not authority**: `final_writer` decides authority
(`docs/joint_authority.json`, `gain_note`). Under `homie` the waist is
`observed_exogenous` (read every control step), never a command target.

Waist live ranges: yaw ±2.618, roll ±0.52, pitch ±0.52 rad. Arm live ranges:
shoulder pitch [-3.089, 2.670], shoulder roll left [-1.588, 2.252] / right
[-2.252, 1.588], shoulder yaw ±2.618, elbow [-1.047, 2.094], wrist roll ±1.972,
wrist pitch/yaw ±1.614 rad.

### 1.2 Hand sign asymmetry (live ranges + registry usable direction)

The two hands are mirrored. A pre-shape / close / open command that uses one
sign on both sides clips on one hand.

| joint | left range (rad) | right range (rad) | usable closing direction |
|---|---|---|---|
| index_0 | [-1.571, 0.0] | [0.0, 1.571] | left negative, right positive |
| index_1 | [-1.745, 0.0] | [0.0, 1.745] | left negative, right positive |
| middle_0 | [-1.571, 0.0] | [0.0, 1.571] | left negative, right positive |
| middle_1 | [-1.745, 0.0] | [0.0, 1.745] | left negative, right positive |
| thumb_0 | [-1.047, 1.047] | [-1.047, 1.047] | registry usable direction: negative on both sides |
| thumb_1 | [-0.724, 1.047] | [-1.047, 0.724] | left positive, right negative |
| thumb_2 | [0.0, 1.745] | [-1.745, 0.0] | left positive, right negative |

The live palm sites are at `[0.1804, ±0.1539, 0.8884]` at t=0. The palm pads are
`gripper0_{left,right}_g0_col` (thin plates, `local_size[0] = 0.0217` vs
0.044/0.0444; column 0 of the pad `xmat` is the face normal). Finger link `_col`
geoms provide the opposing contact surfaces.

### 1.3 Scene inventory per task (live `probe scene`, seed 0)

| task | scene objects (root body, direct geoms) | movable joints | predicate-relevant entity and live numbers |
|---|---|---|---|
| 01_box_lift_floor | `obj` (`obj_main`, 6) | `obj_joint0` free | box root z = 0.0484; body-tree AABB z [0.0500, 0.6049]; floor is plane `gear_factory/floor` at z = 0 |
| 02_push_button | `control_box` (`control_box_main`, 18), `control_panel` (`control_panel_main`, **0 direct geoms**) | `control_panel_button_z` slide, qposadr 50, range [-0.015, 0]; `control_panel_lever_zrot` hinge, qposadr 51, range [-0.4363, 0.4363] | panel **joint 0 = button slide**; button body [0.9564, -0.0469, 0.8455]; button disc `control_panel_g8` cylinder |
| 03_box_lift | `table` (`table_main`, 10), `obj` (`obj_main`, 10) | `obj_joint0` free | tabletop = `table_g7` AABB top z = 0.7995; box root z = 0.7996 |
| 04_push_shelf_forward | `shelf` (`shelf_main`, 13), `box` (`box_main`, 5), `target` (`target_main`, 1 plane) | `shelf_joint0` free, `box_joint0` free; shelf wheel hinges (not predicate) | shelf root [1.5301, 0.0377]; box root z = 1.1692 (on shelf); target body [3.927, 0.0443, 0] (plane AABB is degenerate — use the body pose) |
| 05_drill_lift | `table` (`table_main`, 10), `bottle` (`bottle_main`, **0 direct geoms**) | `bottle_joint0` free | tabletop `table_g7` top z = 0.7995; drill root z = 0.8000; drill collision geoms are children |
| 06_drill_pnp | `table_origin` (`table_origin_main`, 10), `table_target` (`table_target_main`, 10), `obj` (`obj_main`, **0 direct geoms**) | `obj_joint0` free | both tabletops z = 0.7995; drill root z = 0.8000; target table root [-0.0146, 1.2089] |
| 07_box_table_to_shelf | `shelf_board_0..3` (`shelf_board_N_main`, 5 each), `shelf_cabinet` (`shelf_cabinet_main`, 14), `table` (`table_main`, 9), `box` (`box_main`, 10) | `box_joint0` free; boards/cabinet static | shelf board 1 (`shelf_board_1_main`) top z = 0.7408; box root z = 0.6850 |
| 08_pick_drill_from_holder | `drill` (`drill_main`, **0 direct geoms**), `shelf` (`shelf_main`, 14), `holder` (`holder_main`, 35) | `drill_joint0` free | drill root z = 0.7274; holder root z = 0.9674; holder collision geoms are children |
| 09_obstacle_aware_pick_drill | `table` (`table_main`, 10), `bottle` (`bottle_main`, **0 direct geoms**), `obstacle` (`obstacle_main`, 14) | `bottle_joint0` free | tabletop z = 0.7995; drill root z = 0.8000; obstacle root [0.6854, 0.0443, -0.0001] |

Live-model consequences that drive the design:

1. **Zero-geom roots are normal** (02 panel, 05/06/08/09 drill, 04 shelf box is
   fine but 05 drill is not). Object/contact/geometry reads must walk the body
   tree; the root-body-only `observe()['hand_object_contacts']` is not enough
   (`docs/PROTOCOL.md`).
2. **02's predicate joint 0 is the button slide, not the lever hinge.** The live
   model exposes `control_panel_button_z` first (qposadr 50, slide, range
   [-0.015, 0]) and the lever second (qposadr 51, hinge). The success criterion
   calls it "joint 0 ... 0.01 rad"; in the live model it is a **slide
   displacement**, so the read must report qpos + joint type and must not
   convert to radians or read the lever.
3. **04's target is a plane body** (`target_main`, [3.927, 0.0443, 0]) whose
   computed AABB is degenerate. Scene geometry must report the body pose for
   plane geoms.
4. **01's support is the world floor plane** (`gear_factory/floor`, z = 0), not
   a `mujoco_objects` entry. Scene geometry must include it for the floor task.
5. **The object root z is the predicate quantity** for `IsPositionInRange(obj)`
   (01: 0.0484, 03/05/06/09: ~0.80). The body-tree AABB (01: [0.05, 0.6049])
   is for clearance and support-top geometry; using the AABB centre as the
   "object z" would falsely satisfy 01's `z > 0.1` predicate.

### 1.4 Live read channels already exposed

`observe()`/`probe state` exposes: `t`, base `[x, y, z]`, `base_tilt_deg`,
task-object `{name, pos, hand{left, right}}`, and `hand_object_contacts`.
`probe contacts` exposes truthful contact pairs over body trees; `probe joints`
exposes the 43 robot joints + scene joints; `probe bodies` exposes body poses,
geoms, joints, axes, ranges and AABBs. These are the raw channels the read
capabilities below package; no new sensor is invented.

---

## 2. Control-vocabulary reasoning pass

For each commandable group: what the interface can command, the physical effect,
the portfolio need, and the failure if the group has no verb. One row = one verb.

| group | command the interface exposes | physical effect | portfolio need | failure without the verb |
|---|---|---|---|---|
| legs / base | `navigate_cmd` velocity (vx, vy, wz) | walk, turn, hold stance | every task's approach and 04's long push | no locomotion; the robot can only reach what is already in arm range |
| legs / base | pose error closed into `navigate_cmd` | go to a measured base pose | explicit-pose approaches (S2), 14-D candidate | standoff-only navigation cannot place the base at a required pose |
| legs / base | object-relative standoff, closed loop | stop at a measured distance from the live object AABB | every grasp/push/press task | the base stops at a fixed world point; the object moves and the pad gate fails |
| legs / base | `base_height` scalar, closed loop | squat / stand posture height | 01 floor reach; reach adjustments | floor objects stay out of reach; using height as a lift plateaus short |
| waist | advisory torso-pose request (upper channel, `homie` overwrites) | *no verified effect* under `homie` | 17-D candidate / handover experiments | relying on it produces the authority-negative control: HOMIE overwrites the waist |
| waist | waist pose command under `waist_owner=ik|blend` | move the 3 waist joints; 17-D reach/posture | conditional 17-D candidate; handover tests | the waist cannot be used as a posture DoF even after an authorized handover |
| waist | runtime authority handover `homie|ik|blend` | change the waist's final writer, verify gain, restore | the only legal route to `waist_pose_servo` | a high upper-channel gain is mistaken for authority; the waist is silently overwritten |
| arms | two palm poses (12-D) over the 14 arm joints, mink QP | place both palms at a target pose | every grasp / press approach | no accurate pad placement; joint-space guessing |
| arms | Cartesian position servo | move the palms along a Cartesian path (approach / withdraw / retreat) | approach, retract, retreat after place | no clean withdrawal; retreat gets folded into a task-specific hack |
| arms | joint-space ramp / transport | move the arm joints along a measured ramp | lift and carry with the grasp held | the lift becomes a base-height ramp and plateaus short |
| arms | constrained servo (clearance / joint-margin bound) | move while keeping links outside the live object/scene AABB | every approach; W2's missing capability | the approach penetrates or topples the object even though the pads are clear |
| arms | contact push | sustain a contact force along a direction and stop on force/duty | 02 press, 04 push | the hand touches the object but cannot apply a controlled push |
| hands | finger posture preset (pre-shape) | choose aperture / opposition before contact | every grasp and the press | the thumb link pushes the object during the approach (the measured ll_v2 gap) |
| hands | contact-triggered close | close until contact | every grasp | the fingers close to a blind target and crush or miss the object |
| hands | force/drift-limited close | close to a force/drift bound | competing grasp candidate | a hard close on a fragile object; no force boundary |
| hands | hold the last closed target every step | keep the grasp load through motion | every lift / carry / extract | the grip decays to the measured finger positions and the object is released |
| hands | contact-triggered open | open until contact is lost | release after a failed grasp; future drop tasks | the fingers stay closed |
| hands | loaded release (open while supported) | open only while the object is supported by the target surface | 06/07 placement | the unloaded open drops the object before the target supports it |
| sensing | live object pose / body-tree AABB / velocity | re-derive targets every step; measure upright/static/distance | every task | targets go stale, the pad gate fails, a moving object is chased with old numbers |
| sensing | static scene geometry (tables, boards, target zone, holder, obstacle, floor) | measure support tops, target planar centre, clearance volumes | 03/04/06/07/08/09 (and 01's floor) | the task predicate's support/target quantity is unmeasurable |
| sensing | scene joint state (panel joint 0, lever, shelf wheels) | measure the articulated scene quantity | 02 | the press cannot tell a button press from a tool touch |
| sensing | truthful contact pairs over body trees | measure object-table, object-holder, shelf-box, drill-target, hand-object contacts | 03/04/05/06/07/08/09 | root-body contact reads report 0 for child-geom objects; "not in contact" is unverifiable |
| sensing | grasp state (bilateral contact, aperture, relative motion) | proxy for the benchmark grasp test | 05/08/09 (and every grasp) | "grasped" is unmeasurable; the task predicate cannot be checked |
| sensing | safety monitor (joint margin, tilt, penetration) | flag/abort before a limit | every task | silent guard trips and unreported binding constraints |
| sensing | failure visualisation | find failure onset/end and render frames | debugging | actors revise blind; no measured failure window |

Completeness check before leaving bootstrap:

- every commandable joint group has at least one verb: legs/base (nav, pose,
  standoff, height), waist (gated pose + handover), arms (IK, position servo,
  transport, constrained servo, contact push), hands (pre-shape, contact close,
  force-limited close, hold, contact open, loaded release), whole body
  (compositions), sensing (six read/decision verbs) — **yes**;
- every measured failure class maps to a verb or an explicit gap — **yes**
  (Section 8);
- the unclaimed-verb list is non-empty and explained — **yes** (Section 7).

---

## 3. Full capability table

33 nodes: 15 L1 control, 7 L1 read/decision, 11 L2. `REQUIRES` is the graph edge
set; `—` means zero. Demand is the number of portfolio tasks whose strategies
reach the node (transitive), from Section 6.

### 3.1 L1 control verbs

| ID | capability | group | level | channel / law | REQUIRES | contract (verifies) | demand |
|---|---|---|---|---|---|---|---|
| C01 | `base_velocity_command` | legs/base | **L1** | nav velocity (vx,vy,wz); one command channel | — | commanded velocity achieved within gain; base tilt/guard in bounds; binding constraint = tilt or foot slip | 9/9 (via standoff/04) |
| C02 | `base_navigate_to_pose` | legs/base | **L1** | nav velocity; pose error closed into velocity commands | — | final xy/yaw error, tilt, guard; binding constraint = tilt/slip | 9/9 |
| C03 | `base_height_setpoint` | legs/base | **L1** | scalar base height; closed-loop height | — | height error, tilt, guard; binding constraint = height clamp/WBC refusal | 1/9 (01) + optional reach |
| C04 | `waist_pose_servo` | waist | **L1** | 3 waist joints; joint-target servo; **gate `waist_owner in {ik, blend}`** | — | waist pose error, achieved gain >= 0.8, joint margin; restores `homie`; binding constraint = authority/gain | 0/9 direct (conditional) |
| C05 | `ee_pose_ik` | arms | **L1** | 14 arm joints; one mink-QP solve of two palm poses (12-D) | — | pad position/normal error, joint margin, binding constraint (reach / margin / QP infeasible) | 9/9 |
| C06 | `ee_position_servo` | arms | **L1** | 14 arm joints; Cartesian position error closed into joint targets | — | position error, continuity, joint margin; binding constraint = reach/margin | 9/9 |
| C07 | `joint_space_transport` | arms | **L1** | 14 arm joints; joint-space ramp from current to target, measured lag curve | — | joint tracking, ramp duration, joint margin; binding constraint = ramp lag/margin | 6/9 |
| C08 | `constrained_servo` | arms | **L1** | 14 arm joints; Cartesian servo with a clearance or joint-margin bound | — | clearance margin, joint margin, binding constraint = clearance/margin | 9/9 (via C27) |
| C09 | `contact_push` | arms | **L1** | 14 arm joints; advance along a direction, regulate contact force/duty, stop on force/pose | — | contact duty, force bound, object displacement; binding constraint = slip/force | 2/9 (02, 04) |
| C10 | `hand_pre_shape` | hands | **L1** | 14 finger joints; posture preset (aperture/opposition) | — | finger pose error, aperture, no object contact before close; per-side sign; binding constraint = finger range/sign | 9/9 |
| C11 | `grasp_close_contact` | hands | **L1** | 14 finger joints; close until contact (same-channel feedback) | — | bilateral contact, grip stability, object disturbance; binding constraint = no contact/slip | 7/9 |
| C12 | `grasp_close_force_limited` | hands | **L1** | 14 finger joints; close to a force/drift bound | — | grip force bound, slip, object disturbance; binding constraint = force limit | 0/9 required; 05 S2 competing candidate |
| C13 | `grasp_hold` | hands | **L1** | 14 finger joints; hold the last closed target every step | — | hold drift, contact retained, object not released; binding constraint = slip/release | 7/9 |
| C14 | `grasp_open_release` | hands | **L1** | 14 finger joints; open until contact is lost / open target | — | contacts after open, object disturbance; binding constraint = object disturbance | 0/9 (unclaimed) |
| C15 | `loaded_release` | hands | **L1** | 14 finger joints; open while support contact/load is present | — | support contact at open, object settled, no drop; binding constraint = no support contact | 2/9 (06, 07) |

**Authority declaration for the L1 command verbs.** `commands` lists a joint
only if the registry says `final_writer=upper_target` and
`measured_gain >= 0.8`; otherwise the joint is `observed_exogenous` or
`assumes_fixed` with a measured drift bound.

| L1 | `commands` | `observed_exogenous` | `assumes_fixed` | `min_gain` |
|---|---|---|---|---|
| C01/C02/C03 | nav channel (3) / nav channel / base height (1) | all 43 joints + base pose | none | n/a (channel; round 0 measures achieved velocity/height) |
| C04 | waist yaw/roll/pitch **only under `waist_owner=ik|blend`** (registry: `upper_target` under ik, `blend` under blend; gains 1.0 / 0.998 / 1.0) | 14 arms + 14 hands + base pose | none | 0.8 |
| C05 | 14 arm joints (`upper_target`, gains 0.95–0.997) | 3 waist joints while `homie` (read every step) | base pose (x,y,yaw,height) held by the base-lock task; 14 hand joints pre-shaped | 0.8 |
| C06 | 14 arm joints | 3 waist joints (`homie`) + base pose | 14 hand joints (pre-shape/closed vector) | 0.8 |
| C07 | 14 arm joints | 3 waist joints (`homie`) + base pose | 14 hand joints (explicit closed-finger vector held by C13) | 0.8 |
| C08 | 14 arm joints | 3 waist joints (`homie`) + base pose + scene AABBs | 14 hand joints (pre-shape/closed vector) | 0.8 |
| C09 | 14 arm joints | 3 waist joints (`homie`) + base pose | 14 hand joints (pre-shape/closed vector) | 0.8 |
| C10–C15 | 14 hand joints (`upper_target`, gains 0.856–0.863; **command only the usable direction per side**) | 3 waist joints (`homie`) + 14 arm joints + base pose | 14 arm joints (held by IK/servo) | 0.8 |

The legs are never in `commands`; they are WBC-owned. The waist is never in
`commands` under the default owner `homie`.

### 3.2 L1 read / decision verbs

| ID | capability | group | level | read channel | REQUIRES | contract (verifies) | demand |
|---|---|---|---|---|---|---|---|
| C16 | `object_pose_tracking` | sensing | **L1 (read)** | task-object root pose + body-tree AABB + linear/angular velocity + per-side pad-object distance | — | pose, orientation (upright cosine), AABB, speed (static), pad-object distance; reports raw quantities, never a task threshold | 9/9 |
| C17 | `scene_geometry_tracking` | sensing | **L1 (read)** | static scene body poses + body-tree AABBs + joint axes (tables, boards, target zone, holder, obstacle, floor) | — | entity pose/AABB, support-top z, planar centre, plane-geom body pose; zero-geom roots walked | 9/9 |
| C18 | `scene_joint_state_tracking` | sensing | **L1 (read)** | scene-object joint qpos, type, range, axis, per-object joint order/name | — | joint 0 qpos for 02 (`control_panel_button_z`), lever qpos, shelf-wheel qpos; raw qpos, not an assumed angle | 1/9 (02) |
| C19 | `contact_sensing` | sensing | **L1 (read)** | sim contact pairs over body trees (distance/penetration, normal, force) | — | object-table, object-holder, shelf-box, drill-target-table, hand-object pairs; truthful for child-geom roots | 9/9 |
| C20 | `grasp_state_tracking` | sensing | **L1 (read)** | hand-object state: bilateral pad contact, aperture, relative pose/velocity, stability | — | proxy for the benchmark grasp test (the exact predicate is not exposed — G1) | 7/9 |
| C21 | `safety_limit_monitor` | sensing | **L1 (read/decision)** | joint margin, base tilt, penetration, force | — | flags/aborts before a limit; never commands | 9/9 |
| C22 | `failure_visualization` | sensing/debug | **L1 (read)** | recorded trace | — | failure onset/end steps + rendered frames; never reads the success predicate | 0/9 (debug) |

### 3.3 L2 compositions

| ID | capability | level | composes / switches | REQUIRES | contract (verifies) | demand |
|---|---|---|---|---|---|---|
| C23 | `base_navigate_to_standoff` | **L2** | nav + object tracking + safety; selects standoff from the live object AABB | C01, C16, C21 | standoff error, final xy/yaw error, tilt, guard; binding constraint = standoff/tilt | 9/9 |
| C24 | `control_authority_handover` | **L2** | switches waist owner `homie|ik|blend` and lower owner `policy|hold`; verifies gain before/after | C04, C01, C21 | HANDOFF event, gain before/after, restore, balance in bounds; binding constraint = gain/balance | 0/9 direct (warm-up gate) |
| C25 | `whole_body_ik_17d` | **L2** | arm IK + waist command (17-D); gate `waist_owner=ik` | C05, C04 | pad error, waist gain before/after, joint margin; **retire if round 0 does not adopt 17-D** | 0/9 (candidate) |
| C26 | `whole_body_ik_14d` | **L2** | base placement + arm IK (14-D); base held while arms solve | C02, C03, C05 | pad error, base placement error, joint margin; **retire if round 0 does not adopt 14-D** | 0/9 (candidate) |
| C27 | `collision_free_reach_trajectory` | **L2** | IK + tracking + constrained servo + safety | C05, C16, C08, C21 | pad error, clearance margin, joint margin; binding constraint = clearance/reach; **W2b closure** | 9/9 |
| C28 | `bimanual_grasp_object` | **L2** | pre-shape + two-pad IK + contact close, sequenced | C10, C05, C11 | bilateral contact, stability, object disturbance; binding constraint = no contact/slip | 7/9 |
| C29 | `lift_object` | **L2** | grasp + joint-space transport + closed-finger hold + reads; **not a base-height ramp** | C28, C07, C13, C16, C17, C19, C20 | object root-z rise vs support top, grip retained, contact retained; binding constraint = slip/rise | 6/9 |
| C30 | `place_object` | **L2** | transport + hold + **loaded release** + retreat + reads | C07, C13, C15, C06, C16, C17, C19 | support contact at open, object settled/static/upright, pad-object distance; binding constraint = no support contact/drop | 2/9 (06, 07) |
| C31 | `push_object_to_target` | **L2** | contact push + locomotion + task-predicate reads | C09, C01, C16, C17, C19, C21 | shelf upright, shelf-target planar distance, shelf-box contact, tilt; binding constraint = slip/topple | 1/9 (04) |
| C32 | `press_panel_button` | **L2** | contact push + tool contact + **panel joint 0 read** + safety | C09, C19, C18, C21 | panel joint 0 qpos displacement, tool contact, force bound, tilt; binding constraint = no joint motion/slip | 1/9 (02) |
| C33 | `extract_object_from_holder` | **L2** | grasp + Cartesian retract + hold + holder read | C28, C06, C13, C19, C17 | drill-holder contact cleared, grasp retained, retract distance; binding constraint = holder collision/slip | 1/9 (08) |

---

## 4. Worked level examples (from `docs/control_vocabulary.md`) and agreement

The bootstrap doc gives these examples; our labels must agree with them.

| candidate | doc level | our node | our level | agreement |
|---|---|---|---|---|
| `base_velocity_command` | L1 | C01 | L1 | yes |
| `base_navigate_to_pose` | L1 | C02 | L1 | yes — feedback inside one channel is still L1 |
| `base_height_setpoint` | L1 | C03 | L1 | yes |
| `ee_pose_ik` | L1 | C05 | L1 | yes |
| `hand_pre_shape` | L1 | C10 | L1 | yes |
| `grasp_hold` | L1 | C13 | L1 | yes |
| `object_pose_tracking` | L1 (read) | C16 | L1 (read) | yes |
| `base_navigate_to_standoff` | L2 | C23 | L2 | yes — nav + tracking + safety |
| `collision_free_reach_trajectory` | L2 | C27 | L2 | yes — IK + tracking + constrained servo + monitor |
| `bimanual_grasp_object` | L2 | C28 | L2 | yes — pre-shape + two-pad IK + contact close |

No worked example is contradicted.

---

## 5. Label-consistency checks (the four required checks)

**Check 1 — every L1 has zero REQUIRES edges.**
All 15 L1 control verbs and all 7 L1 read/decision verbs have `REQUIRES = —`
(Section 3). Read verbs consume the simulator state directly; they do not declare
edges to other capabilities. **PASS.**

**Check 2 — every L2 has >= 2 REQUIRES and names them.**
C23: 3; C24: 3; C25: 2; C26: 3; C27: 4; C28: 3; C29: 7; C30: 7; C31: 6;
C32: 4; C33: 5. Every L2 names its edges in the table. **PASS.**

**Check 3 — a one-verb name is L1 unless it explicitly switches between
different channels.**
`base_velocity_command`, `base_navigate_to_pose`, `base_height_setpoint`,
`waist_pose_servo`, `ee_pose_ik`, `ee_position_servo`, `joint_space_transport`,
`constrained_servo`, `contact_push`, `hand_pre_shape`, `grasp_close_contact`,
`grasp_close_force_limited`, `grasp_hold`, `grasp_open_release`,
`loaded_release`, and all seven read/decision names are L1. The one-verb-looking
names that are L2 do explicitly switch or compose: `base_navigate_to_standoff`
(nav + tracking + safety), `control_authority_handover` (switches owners),
`collision_free_reach_trajectory`, `bimanual_grasp_object`, `lift_object`,
`place_object`, `push_object_to_target`, `press_panel_button`,
`extract_object_from_holder`, `whole_body_ik_17d/14d`. **PASS.**

**Check 4 — a capability that selects a target from another capability's output,
or changes behaviour when a condition is met, is L2.**
`base_navigate_to_standoff` selects the standoff from `object_pose_tracking` →
L2. `bimanual_grasp_object` changes behaviour at contact (close) → L2.
`lift_object`, `place_object`, `push_object_to_target`, `press_panel_button`,
`extract_object_from_holder` change behaviour on read conditions → L2.
**Decision on the borderline:** a stopping/guard condition *inside the same
channel* is feedback, not a channel switch, so `grasp_close_contact`,
`grasp_close_force_limited`, `grasp_hold`, `grasp_open_release`,
`loaded_release`, `constrained_servo` and `contact_push` stay L1 (the level rule
says "feedback inside that same channel is still L1"; the vocabulary lists
contact-triggered close/open as atomic hand verbs). Rule 4 is applied to
switching between distinct channels or selecting another capability's output,
not to a same-channel stopping condition. **PASS**, with the decision recorded
in D15/D19/D08/D09.

---

## 6. Predicate coverage check

For every task: the quantity its success predicate reads, the capability that
measures it, and the live evidence. The predicate text is from
`docs/task_success_criteria.md` (read-only).

| task | predicate quantity (read-only reference) | capability that measures it | live evidence / note |
|---|---|---|---|
| 01_box_lift_floor | box z > 0.1; box upright | `object_pose_tracking` (C16) | root z = 0.0484; orientation from root xmat; **use root-body z, not AABB centre** |
| 02_push_button | control-panel joint 0 moved at least 0.01 | `scene_joint_state_tracking` (C18) | joint 0 = `control_panel_button_z`, slide, qposadr 50, range [-0.015, 0]; lever is joint 1, not the predicate |
| 03_box_lift | box not in table contact; box z > table top + 0.1; gripper not far | `contact_sensing` (C19) + `object_pose_tracking` (C16) + `scene_geometry_tracking` (C17) | tabletop `table_g7` top z = 0.7995; box root z = 0.7996; pad-object distance from C16 |
| 04_push_shelf_forward | shelf upright; shelf within 0.3 planar of target; shelf still in contact with box | `object_pose_tracking` (C16) + `scene_geometry_tracking` (C17) + `contact_sensing` (C19) | shelf root [1.5301, 0.0377]; target body [3.927, 0.0443] (plane; use body pose); shelf-box pairs |
| 05_drill_lift | drill not in table contact; drill z > table top + 0.1; drill grasped | `contact_sensing` (C19) + `object_pose_tracking` (C16) + `scene_geometry_tracking` (C17) + `grasp_state_tracking` (C20) | tabletop z = 0.7995; drill root z = 0.8000; drill collision geoms are children |
| 06_drill_pnp | drill static; drill upright; gripper > 0.1 from drill; drill in contact with target table | `object_pose_tracking` (C16: velocity, orientation, pad distance) + `scene_geometry_tracking` (C17: `table_target`) + `contact_sensing` (C19) | drill root z = 0.8000; target table root [-0.0146, 1.2089]; contact pairs |
| 07_box_table_to_shelf | box static; box upright; box in contact with shelf board 1 | `object_pose_tracking` (C16) + `scene_geometry_tracking` (C17: `shelf_board_1_main`) + `contact_sensing` (C19) | board 1 top z = 0.7408; box root z = 0.6850; box-board 1 pairs |
| 08_pick_drill_from_holder | drill not in holder contact; drill grasped | `contact_sensing` (C19) + `scene_geometry_tracking` (C17: `holder_main`) + `grasp_state_tracking` (C20) | drill root z = 0.7274; holder root z = 0.9674; holder geoms are children |
| 09_obstacle_aware_pick_drill | drill not in table contact; drill z > table top + 0.1; drill grasped | `contact_sensing` (C19) + `object_pose_tracking` (C16) + `scene_geometry_tracking` (C17) + `grasp_state_tracking` (C20) | tabletop z = 0.7995; drill root z = 0.8000; obstacle root [0.6854, 0.0443] |

Cross-check against the coordinator prompt's required table:

| task | required "must be measurable by" | our design |
|---|---|---|
| 01 | object tracking (+ upright read) | C16 reports pose, orientation, velocity, AABB |
| 02 | scene joint-state tracking | C18 reports per-object joint 0 qpos (button slide) |
| 03 | object tracking + contact sensing | C16 + C19 (+ C17 for the table top) |
| 04 | scene geometry + contact sensing | C16 (upright/planar) + C17 (target) + C19 (shelf-box) |
| 05 | object tracking + contact sensing | C16 + C19 (+ C17 + C20) |
| 06 | object tracking + scene geometry + contact sensing | C16 (static/upright/distance) + C17 (target table) + C19 |
| 07 | object tracking + scene geometry + contact sensing | C16 (static/upright) + C17 (board 1) + C19 |
| 08 | contact sensing + scene geometry (holder) | C19 + C17 + C20 |
| 09 | object tracking + contact sensing | C16 + C19 (+ C17 + C20) |

**Result: every predicate quantity has a measuring capability.** The only
unmeasurable quantity is the benchmark's exact `IsGrasped` test, which is not
exposed; `grasp_state_tracking` (C20) is a measured proxy (bilateral pad
contact + aperture + relative pose/velocity). This is recorded as gap G1, not
deferred to a task program.

Scene-level sensing and scene joint-state sensing are therefore nodes in their
own right (C17, C18), as required by the prompt — not task-program details.

---

## 7. Composition consistency check

| measured lesson (prompt) | L2 that must obey it | how the design obeys it | result |
|---|---|---|---|
| placing uses the **loaded-release law** (open while supported), not the unloaded open | `place_object` (C30) | REQUIRES `loaded_release` (C15), not `grasp_open_release` (C14); contract checks support contact at open and object settling | **PASS** |
| a lift is a **joint-space transport with the explicit closed-finger vector held**, not a base-height ramp | `lift_object` (C29) | REQUIRES `joint_space_transport` (C07) + `grasp_hold` (C13); does **not** require `base_height_setpoint`; contract checks root-z rise and grip retention | **PASS** |
| a push measures the **task predicate** (uprightness, contact with its carrier), not only hand-object duty | `push_object_to_target` (C31) | REQUIRES `object_pose_tracking` (upright), `scene_geometry_tracking` (target zone), `contact_sensing` (shelf-box); contract reports those three | **PASS** |
| a press measures the **panel joint angle**, not only tool contact | `press_panel_button` (C32) | REQUIRES `scene_joint_state_tracking` (C18, panel joint 0) + `contact_sensing` (C19); contract reports joint-0 qpos | **PASS** |

Additional composition checks:

- `bimanual_grasp_object` = pre-shape + two-pad IK + contact close, sequenced
  (the worked example) — **PASS**.
- `collision_free_reach_trajectory` = IK + tracking + constrained servo +
  monitor (the worked example) — **PASS**.
- `base_navigate_to_standoff` = nav + object tracking + safety (the worked
  example) — **PASS**.

---

## 8. Unclaimed verbs / capabilities

An unclaimed capability is still a node; demand is marked separately. The list
below is non-empty and explained.

| capability | level | why unclaimed | disposition |
|---|---|---|---|
| `grasp_close_force_limited` (C12) | L1 | no task strategy requires it; it is the competing stopping law to `grasp_close_contact` | keep as a competing candidate; run both in one slot when a measured failure names grip force/slip; merge only if the Validator shows mechanism-equivalent behaviour, then one becomes a SKILL of the other |
| `grasp_open_release` (C14) | L1 | no task strategy uses the unloaded open; placement uses `loaded_release` | keep; it is the verb for releasing an unsupported/failed grasp and for future drop tasks; do not retire |
| `waist_pose_servo` (C04) | L1 (gated) | no task strategy requires it under the adopted 12-D / `waist_owner=homie` default; it is required only by the conditional 17-D and handover L2s | keep; it is the channel the handover switches to; a task that needs it must first pass the gate |
| `control_authority_handover` (C24) | L2 | required by the warm-up gate, not by any task strategy under the default | keep; the only legal route to `waist_pose_servo`; warm-up substrate |
| `whole_body_ik_17d` (C25) | L2 | candidate; no task strategy requires it under the adopted 12-D default | keep until round 0; retire with measured evidence if 12-D wins |
| `whole_body_ik_14d` (C26) | L2 | candidate; no task strategy requires it; eliminated in the prior round 0 and re-measured in ll_v3 | keep until round 0; retire with measured evidence if eliminated again |
| `failure_visualization` (C22) | L1 (read) | debug-only; no task strategy claims it; it is an actor tool | keep; never reads the success predicate |

No other capability has zero task reachability: the shared base
(`base_navigate_to_standoff`, `collision_free_reach_trajectory`,
`object_pose_tracking`, `contact_sensing`, `safety_limit_monitor`,
`ee_pose_ik`, `ee_position_servo`, `hand_pre_shape`,
`scene_geometry_tracking`) is 9/9; the grasp chain is 7/9; lift is 6/9; place is
2/9; push, press and extract are 1/9 each.

---

## 9. Failure class -> verb or gap

| failure class (measured or documented) | evidence | mapping | gap? |
|---|---|---|---|
| pre-shape missing; the thumb link pushes the object during the approach | `docs/control_vocabulary.md` (ll_v2 gap) | `hand_pre_shape` (C10) | no |
| grip decays to the measured finger positions; the object is released in motion | vocabulary worked examples | `grasp_hold` (C13) + `joint_space_transport` (C07) holding the explicit closed-finger vector | no |
| targets go stale; a moving object is chased with old numbers | vocabulary worked examples | `object_pose_tracking` (C16) | no |
| the approach penetrates or topples the object even though the pads are clear | vocabulary worked examples | `constrained_servo` (C08) + `collision_free_reach_trajectory` (C27) | no |
| floor objects stay out of reach; using the height channel as a lift plateaus short | vocabulary worked examples | `base_height_setpoint` (C03) + `lift_object` (C29) via joint-space transport, **not** a base-height ramp | no |
| 04 push contract passed 9/10 while the engine returned 0/10 (it measured hand-object duty only) | `docs/task_success_criteria.md` | `push_object_to_target` (C31) measures shelf upright, shelf-target planar distance and shelf-box contact via C16/C17/C19 | no |
| 02 press contract measures tool contact only | `docs/task_success_criteria.md` | `press_panel_button` (C32) measures panel joint 0 via C18 | no |
| W2 integration benchmark fails (0/4); `collision_free_reach_trajectory` is missing | `prompts/coordinator.md` | C27 (L2) + C08 (L1); W2b closure job, not batch 1 | no |
| placement opens unloaded and the object drops before support | prompt composition lesson | `loaded_release` (C15) + `place_object` (C30) | no |
| waist overwritten by HOMIE; a high upper-channel gain is mistaken for authority | `docs/joint_authority.json`, `docs/control_parameterization.md` | `waist_pose_servo` (C04, gated) + `control_authority_handover` (C24) | no |
| 17-D executed pad error is worse than 12-D on every seed | `docs/control_parameterization.md` | `whole_body_ik_17d` (C25) is a candidate; retire it if ll_v3 round 0 confirms; keep C05 + C04 | no (decision) |
| 14-D base-height/yaw candidate is eliminated | prior round 0 (documented) | `whole_body_ik_14d` (C26) is a candidate; retire it if ll_v3 round 0 confirms; strategies require C02 + C03 + C05 | no (decision) |
| zero-geom object roots make root-body contact reads report 0 | `docs/PROTOCOL.md`; live scene probe | `contact_sensing` (C19) and `object_pose_tracking` (C16) walk the body tree; do not use `hand_object_contacts` alone | no |
| panel root body has zero direct geoms | live scene probe | `scene_joint_state_tracking` (C18) and `scene_geometry_tracking` (C17) walk child bodies | no |
| one-sided finger ranges: a wrong sign clips to zero | live joint probe; registry `measured_gain_note` | C10–C15 use the per-side usable direction from the registry/live ranges | no |
| panel predicate joint 0 is a **slide**, not the lever hinge | live body/joint probe | C18 reports joint 0 qpos + type; C32 does not assume radians or read the lever | no |
| 04 target zone is a plane with a degenerate AABB | live scene probe | C17 reports the plane body pose, not the AABB | no |
| exact benchmark `IsGrasped` predicate is not exposed | `docs/task_success_criteria.md`; `docs/PROTOCOL.md` | `grasp_state_tracking` (C20) is a proxy | **G1** |
| WBC posture refusal / arm reach / gripper span | `docs/control_parameterization.md` ("what this does not fix") | no control verb removes it; every L1 reports the binding constraint (metres/radians/margin) | **G2** |
| no direct lower-body joint command | `docs/PROTOCOL.md` (WBC owns legs) | only the nav and height channels exist; never propose hip/knee/ankle commands | **G3** |
| predicate constants must not be hard-coded into L1s | `prompts/coordinator.md`; `docs/task_success_criteria.md` | L1s report raw quantities; L2s/task programs compare to the task predicate | **G4** (rule, not a missing verb) |

### Gaps (explicit)

- **G1 — exact grasp predicate not exposed.** `IsGrasped` is benchmark-owned and
  not readable. `grasp_state_tracking` (C20) reports bilateral pad contact,
  aperture, relative object-hand pose/velocity and stability. The proxy must be
  validated against task outcomes; it must never read or imitate the predicate
  implementation. This is a read gap, mitigated by the proxy.
- **G2 — structural WBC/reach/gripper-span limits.** No control verb can remove
  a WBC posture refusal, an arm-reach shortfall or a gripper-span shortfall.
  Every L1 contract returns the single binding constraint with its measured
  shortfall; the experiment reports it as a structural boundary, not a missing
  capability.
- **G3 — no direct lower-body joint verb (by design).** The legs are WBC-owned.
  The only lower-body-facing channels are `navigate_cmd` (C01/C02) and
  `base_height` (C03). This is recorded so no later failure is answered with a
  hip/knee/ankle command.
- **G4 — predicate constants are not capabilities.** L1s report raw measured
  quantities (root z, orientation, speed, pad distance, joint qpos, contact
  pairs). Only L2s and task programs may compare against the read-only
  `docs/task_success_criteria.md` thresholds. This is a design rule, not a
  missing verb.

---

## 10. Task demand (two strategies per task, for Phase G)

The task demand matrix is transitive over the L2 `REQUIRES` edges. `x` = required
by at least one of the task's two strategies; `o` = optional reach adjustment;
`c` = conditional on the round-0 dimension/handover decision; `—` = not reached.

| capability | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| C01 base_velocity_command | x | x | x | x | x | x | x | x | x |
| C02 base_navigate_to_pose | x | x | x | x | x | x | x | x | x |
| C03 base_height_setpoint | x | — | o | — | o | — | — | — | o |
| C04 waist_pose_servo | c | c | c | c | c | c | c | c | c |
| C05 ee_pose_ik | x | x | x | x | x | x | x | x | x |
| C06 ee_position_servo | x | x | x | x | x | x | x | x | x |
| C07 joint_space_transport | x | — | x | — | x | x | x | — | x |
| C08 constrained_servo | x | x | x | x | x | x | x | x | x |
| C09 contact_push | — | x | — | x | — | — | — | — | — |
| C10 hand_pre_shape | x | x | x | x | x | x | x | x | x |
| C11 grasp_close_contact | x | — | x | — | x | x | x | x | x |
| C12 grasp_close_force_limited | — | — | — | — | c | — | — | — | — |
| C13 grasp_hold | x | — | x | — | x | x | x | x | x |
| C14 grasp_open_release | — | — | — | — | — | — | — | — | — |
| C15 loaded_release | — | — | — | — | — | x | x | — | — |
| C16 object_pose_tracking | x | x | x | x | x | x | x | x | x |
| C17 scene_geometry_tracking | x | x | x | x | x | x | x | x | x |
| C18 scene_joint_state_tracking | — | x | — | — | — | — | — | — | — |
| C19 contact_sensing | x | x | x | x | x | x | x | x | x |
| C20 grasp_state_tracking | x | — | x | — | x | x | x | x | x |
| C21 safety_limit_monitor | x | x | x | x | x | x | x | x | x |
| C22 failure_visualization | — | — | — | — | — | — | — | — | — |
| C23 base_navigate_to_standoff | x | x | x | x | x | x | x | x | x |
| C24 control_authority_handover | c | c | c | c | c | c | c | c | c |
| C25 whole_body_ik_17d | c | c | c | c | c | c | c | c | c |
| C26 whole_body_ik_14d | c | c | c | c | c | c | c | c | c |
| C27 collision_free_reach_trajectory | x | x | x | x | x | x | x | x | x |
| C28 bimanual_grasp_object | x | — | x | — | x | x | x | x | x |
| C29 lift_object | x | — | x | — | x | x | x | — | x |
| C30 place_object | — | — | — | — | — | x | x | — | — |
| C31 push_object_to_target | — | — | — | x | — | — | — | — | — |
| C32 press_panel_button | — | x | — | — | — | — | — | — | — |
| C33 extract_object_from_holder | — | — | — | — | — | — | — | x | — |

Strategy sketches (names for Phase G; the exact decomposition is re-checked
against the live task before seeding):

| task | S1 (standoff / grasp chain) | S2 (explicit-pose / direct chain) |
|---|---|---|
| 01 | `floor_standoff_grasp_lift`: C23 → C27 → C28 → C29 | `floor_crouch_direct_grasp_lift`: C02 + C03 → C06 → C10 + C05 + C11 → C07 + C13 |
| 02 | `standoff_reach_press`: C23 → C27 → C10 → C32 | `direct_cartesian_press`: C02 → C06 → C09 + C18 + C19 |
| 03 | `table_standoff_grasp_lift`: C23 → C27 → C28 → C29 | `table_direct_grasp_lift`: C02 → C06 → C10 + C05 + C11 → C07 + C13 |
| 04 | `standoff_push_to_target`: C23 → C27 → C10 → C31 | `walk_and_push_tracking`: C02 + C01 → C06 → C09 + C16 + C17 + C19 |
| 05 | `drill_standoff_grasp_lift`: C23 → C27 → C28 → C29 | `drill_direct_grasp_lift`: C02 + C03 → C06 → C10 + C05 + C11/C12 → C07 + C13 |
| 06 | `drill_pick_lift_place`: C23 → C27 → C28 → C29 → C30 | `drill_pick_transport_release`: C02 → C06 → C10 + C05 + C11 → C07 + C13 + C15 |
| 07 | `box_pick_lift_place_board1`: C23 → C27 → C28 → C29 → C30 | `box_direct_transport_release`: C02 → C06 → C10 + C05 + C11 → C07 + C13 + C15 |
| 08 | `holder_standoff_grasp_extract`: C23 → C27 → C28 → C33 | `holder_direct_grasp_retract`: C02 → C06 → C10 + C05 + C11 → C06 + C13 + C19 |
| 09 | `obstacle_standoff_clearance_grasp_lift`: C23 → C27 → C28 → C29 | `obstacle_direct_clearance_grasp_lift`: C02 + C03 → C06 + C08 → C10 + C05 + C11 → C07 + C13 |

Shared first-batch demand (the first selected job must be required by at least
two strategies/tasks): C23, C27, C16, C19, C21, C05, C06, C10 and C17 are 9/9;
C28 is 7/9; C07/C13/C29 are 6/9. A broad first batch of >= 10 capabilities is
available from this shared base alone.

---

## 11. Decisions (every borderline label, decided)

Each entry is a decision with its reason, not an open question.

- **D1 — `base_velocity_command` = L1.** One nav-velocity channel; the worked
  example labels it L1.
- **D2 — `base_navigate_to_pose` = L1.** A pose error closed into velocity
  commands is one channel with feedback; the worked example explicitly says it
  is not L2 just because it has feedback.
- **D3 — `base_height_setpoint` = L1.** One scalar closed-loop channel; it is a
  posture height, not a lift.
- **D4 — `base_navigate_to_standoff` = L2.** It composes nav + object tracking +
  safety and selects the standoff from the live AABB; the worked example labels
  it L2.
- **D5 — `ee_pose_ik` = L1.** One solve over the arm channel; the worked example
  labels it L1.
- **D6 — `ee_position_servo` = L1.** One Cartesian position channel with one
  closed-loop law. It is the withdraw/retreat verb; the prompt says withdraw is
  not a separate L1.
- **D7 — `joint_space_transport` = L1.** It commands only the 14 arm joints with
  one interpolation law. The explicit closed-finger vector is held by
  `grasp_hold` (C13) in the lift L2, so transport is not a two-channel L2.
- **D8 — `constrained_servo` = L1.** The clearance/joint-margin constraint is a
  bound inside the same arm channel, not a second capability.
- **D9 — `contact_push` = L1.** One arm channel with one contact-force/duty law.
  Push and press are L2s that add the task-predicate reads.
- **D10 — `waist_pose_servo` = L1 with a gate.** The decision table says a
  command valid only under a precondition is L1, and the precondition is a gate,
  not a REQUIRES edge. Under `homie` the waist is `observed_exogenous`, not a
  command; under `ik|blend` it is a command target with a verified gain.
- **D11 — `control_authority_handover` = L2.** The decision table says an
  authority handover that switches channels/modes is L2, requiring the channel
  verbs it switches between. It REQUIRES `waist_pose_servo`,
  `base_velocity_command` and `safety_limit_monitor`.
- **D12 — `whole_body_ik_17d` = L2.** It composes arm IK and the waist command
  channel (17-D) under the `waist_owner=ik` gate. It is retired if round 0 does
  not adopt 17-D (the decision table's retirement rule).
- **D13 — `whole_body_ik_14d` = L2.** Base placement + arm IK is explicitly L2
  in the decision table. It is retired if round 0 does not adopt 14-D.
- **D14 — `hand_pre_shape` = L1.** One finger-posture channel; the worked example
  labels it L1, and it is the verb ll_v2 missed.
- **D15 — `grasp_close_contact` = L1.** Contact is feedback inside the finger
  channel; the vocabulary lists contact-triggered close as an atomic hand verb.
  Rule 4's "changes behaviour when a condition is met" is applied to switching
  between distinct channels or selecting another capability's output, not to a
  same-channel stopping condition.
- **D16 — `grasp_close_force_limited` = L1, competing candidate.** Two stopping
  laws for the same end state are two L1 candidates; merge only if the Validator
  shows mechanism-equivalent behaviour, then one becomes a SKILL of the other.
- **D17 — `grasp_hold` = L1.** One finger channel; hold the last closed target
  every step. The worked example labels it L1.
- **D18 — `grasp_open_release` = L1.** One opening law (unloaded open); unclaimed
  by the current strategies but retained.
- **D19 — `loaded_release` = L1.** One opening law (open while supported). It is
  a different stopping/support condition from the unloaded open, so it is a
  second L1 candidate for the same end state; the placement L2 uses this one.
- **D20 — `object_pose_tracking` = L1 (read).** One read channel: object root
  pose + body-tree AABB + velocity + pad distance. Upright, static and distance
  are derived from the same read, so they are not separate capabilities.
- **D21 — `scene_geometry_tracking` = L1 (read).** One read channel over static
  scene body poses/AABBs. Reading a set of static entities is not a sequence or
  a switch, so it is not L2.
- **D22 — `scene_joint_state_tracking` = L1 (read).** One read channel over
  scene joint qpos. It is a node because 02's predicate reads panel joint 0; the
  prompt says scene joint-state sensing is a capability in its own right.
- **D23 — `contact_sensing` = L1 (read).** One read channel over sim contacts.
  The L2s select which body-tree pairs matter; the read itself does not.
- **D24 — `grasp_state_tracking` = L1 (read).** One derived read channel over
  the hand-object state. If it is implemented by calling C19 and C16, that is an
  implementation detail, not a REQUIRES edge; read-only channels are L1 (read).
- **D25 — `safety_limit_monitor` = L1 (read/decision).** The decision table says
  an invariant/guard that never commands is L1 (read/decision); it is unclaimed
  if no task needs it, and here every task needs it.
- **D26 — `failure_visualization` = L1 (read).** One trace channel; debug-only;
  never reads the success predicate.
- **D27 — `bimanual_grasp_object` = L2.** Pre-shape + two-pad IK + contact
  close, sequenced; the worked example labels it L2.
- **D28 — `collision_free_reach_trajectory` = L2.** IK + tracking + constrained
  servo + monitor; the worked example labels it L2. It is the missing W2
  capability, so it is a W2b closure job, not batch 1.
- **D29 — `lift_object` = L2.** It composes the grasp, joint-space transport,
  the explicit closed-finger hold and the reads. It does **not** use the base
  height as the lift (composition lesson).
- **D30 — `place_object` = L2.** It composes transport, hold, the loaded
  release, the retreat and the reads. It does **not** use the unloaded open
  (composition lesson).
- **D31 — `push_object_to_target` = L2.** It composes contact push, locomotion
  and the task-predicate reads; it measures shelf uprightness, shelf-target
  planar distance and shelf-box contact, not only hand-object duty.
- **D32 — `press_panel_button` = L2.** It composes contact push, tool contact,
  the panel joint-0 read and safety; it measures the joint qpos, not only tool
  contact.
- **D33 — `extract_object_from_holder` = L2.** It composes the grasp, a
  Cartesian retract, the hold, the holder contact read and scene geometry; it
  measures holder contact cleared and grasp retained.
- **D34 — L2 -> L2 edges are allowed.** `lift_object` and
  `extract_object_from_holder` require `bimanual_grasp_object` (an L2) while
  also having >= 2 L1 edges; no L2 is a leaf. If the manager prefers a strict
  L2 -> L1-only graph, the grasp L1s can be inlined; the design keeps the
  hierarchy edge because the graph is a hierarchy and the leaf-L2 defect rule is
  satisfied.
- **D35 — root-body z vs body-tree AABB.** `object_pose_tracking` reports both.
  The L2/task program uses the **root-body z** to match `IsPositionInRange(obj)`
  (01 root z = 0.0484, not the AABB centre ~0.327); the AABB is for clearance and
  support-top geometry.
- **D36 — predicate constants stay out of L1.** No L1 hard-codes a task
  predicate constant; L1s report raw measured quantities, and only L2s/task
  programs compare against `docs/task_success_criteria.md`.
- **D37 — the lower-body `policy|hold` mode is a handover mode, not a separate
  L1.** The harness does not expose lower-body joint commands (PROTOCOL: the WBC
  owns the legs). `control_authority_handover` therefore REQUIRES the waist and
  nav channels plus the safety monitor; it does not create a leg verb.
- **D38 — the 17-D L2 does not require the handover L2.** The authorized
  handover is a precondition/gate in the 17-D spec (`waist_owner=ik`, gain
  verified before/after, restore on exit), not part of the solve. The task
  program sequences the handover first.
- **D39 — the panel predicate joint is the button slide.** Live model order:
  `control_panel_button_z` (slide, qposadr 50) then `control_panel_lever_zrot`
  (hinge, qposadr 51). `scene_joint_state_tracking` reports both by name and
  index; `press_panel_button` reads joint 0 (the button) and does not assume
  radians or read the lever. The task doc's "0.01 rad" is a unit label on a
  slide displacement; the contract uses the joint's own units.
- **D40 — 04's target zone is read from the body pose, not the AABB.** The
  `target_main` plane geom has a degenerate AABB; scene geometry reports the
  body pose and the planar centre.
- **D41 — 01's support is the world floor plane.** Scene geometry includes
  `gear_factory/floor` at z = 0 so the floor task can measure a support top
  without a table object.
- **D42 — contact reads must walk body trees.** `contact_sensing` does not rely
  on `observe()['hand_object_contacts']` (root-body-only) for zero-geom roots;
  it uses truthful contact pairs over the object/body trees.

---

## 12. Design gate status and next step

- Four label-consistency checks: **PASS** (Section 5).
- Predicate coverage check: **complete** for all nine tasks; one proxy gap (G1,
  exact `IsGrasped` not exposed) and three structural/rule gaps (G2–G4).
- Composition consistency check: **PASS** for all four measured lessons
  (Section 7).
- Unclaimed-verb list: **non-empty and explained** (Section 8).
- Failure class -> verb-or-gap mapping: **complete** (Section 9).
- Every commandable joint group has at least one verb: **yes** (Section 2).
- Graph writes: **none**. `graph_cli.py coverage` still shows 9 tasks, 0
  strategies, 0 capabilities. Round 0, warm-up and batches: **not started**.

**Next step:** the manager reviews this design. Phase G (graph seeding) starts
only after that review, using the levels and REQUIRES sets above; the same four
checks are then run on the graph (L1 with zero REQUIRES, L2 with >= 2 REQUIRES).
