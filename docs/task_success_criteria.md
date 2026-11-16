# Task success criteria (read-only reference)

Benchmark-owned success predicates, read from the task classes in
`humanoidmimicgen/locomanipulation/envs/` (the benchmark repo).

**Read-only for every agent.** The coordinator, proposal actors, execution
actors and validators may read this file to check whether a contract covers the
task condition. Nobody may edit it, edit the benchmark predicate, or hard-code a
task's predicate constants into an L1 skill. Capabilities stay general; L2
compositions and task programs may use these conditions to align their
contracts. Do not optimize a rollout against the predicate (no predicate-driven
reward shaping, no per-seed constant fitting).

`IsPositionInRange(obj, axis=2, max_val)` means the object's z is above
`max_val`. `IsClose(a, b, d, planar=True)` means planar distance <= d.
`IsGripperFar(obj, threshold)` means the gripper is farther than the threshold
from the object. `IsGrasped(obj)` is the benchmark's grasp test.
`IsJointQposInRange(panel, 0, -1, -0.01)` means joint 0 moved at least 0.01.

| task | class | success = AND of |
|---|---|---|
| 01_box_lift_floor | LMBoxLiftFloor | box z > 0.1 m (lifted off the floor); box upright |
| 02_push_button | LMPushButton | control-panel joint 0 moved at least 0.01 rad (button pressed) |
| 03_box_lift | LMBoxLift | box not in contact with the table; box z > table top + 0.1 m; gripper not far from the box |
| 04_push_shelf_forward | LMPushShelfForward | shelf upright; shelf within 0.3 m (planar) of the target zone; shelf still in contact with the box |
| 05_drill_lift | LMDrillLift | drill not in contact with the table; drill z > table top + 0.1 m; drill grasped |
| 06_drill_pnp | LMDrillPnP90 | drill static; drill upright (threshold 0.95); gripper farther than 0.1 m from the drill; drill in contact with the target table |
| 07_box_table_to_shelf | LMBoxTableToShelf | box static; box upright (0.95); box in contact with shelf board 1 |
| 08_pick_drill_from_holder | LMPickDrillFromHolder | drill not in contact with the holder; drill grasped |
| 09_obstacle_aware_pick_drill | LMDrillLiftObstacle (inherits LMDrillLift) | drill not in contact with the table; drill z > table top + 0.1 m; drill grasped |

## Consequences for contract design

- **04**: a push contract that measures planar progress, target gap, hand-object
  contact duty and robot base tilt is **not** the task condition. The task also
  requires the shelf to stay upright and the box to remain in contact with the
  shelf. Any 04 contract must measure shelf orientation and shelf-box contact,
  not hand-shelf contact. This is why `push_object_to_target_v2` passed 9/10
  while the engine returned 0/10.
- **08**: `not in contact with holder` + `grasped` matches the extraction
  contract; the program passed 10/10.
- **02**: the contract must measure the panel joint angle, not just tool-point
  contact or reach.
- **03/05/09**: "lifted" is defined in world z relative to the table top, plus
  the benchmark grasp test; a contact-only or rise-only contract is insufficient.
- **06/07**: "placed" requires static + upright + contact with the target
  surface, and for 06 the gripper must be away from the object.
- **01**: only lift height and uprightness; no table contact term.
