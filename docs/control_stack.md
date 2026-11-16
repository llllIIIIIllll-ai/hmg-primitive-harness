# Control stack — source map (read this first on a control anomaly)

**Rule.** When a commanded motion does not happen, do not infer a "policy
refusal" from tracking numbers. Read the source below first, then design the
measurement. Cite `path:line` as evidence in the report. Measurement is the
second step, not the first.

The source below is **in scope** for the ll_v3 run. The isolation rule covers
other *experiment runs* (ll_v1/ll_v2, work/tracks, library) — not the control
stack this harness drives. All paths are live (editable install + venv used by
`scripts/server_ctl.sh`).

## One control step

```
hmg_server._send
  -> env.step_goal(WBCGoal(upper, nav, height))          hmg_harness/adapter.py
  -> ActionEnv.step                                      humanoidmimicgen/dataset_playback.py
  -> G1DecoupledWholeBodyPolicy.get_action               humanoidmimicgen/wbc/policy/__init__.py
       q[upper_body_indices] = target_upper_body_pose    (IdentityPolicy: pass-through)
       q[lower_body_indices] = HOMIE ONNX policy         (legs + waist; reads q_arms as input)
  -> sync_env.step({"q": ...})                           humanoidmimicgen/wbc/envs/locomanipulation/sync_env.py
  -> robosuite env -> composite controller (HybridWholeBodyMinkIK)
  -> part controllers (JOINT_POSITION)                   robosuite/controllers/parts/generic/joint_pos.py
  -> <motor ctrlrange> in the G1 XML                     g1_29dof_rev_1_0.xml
  -> MuJoCo sim
```

## File map

| what it answers | path |
|---|---|
| Is the arm a policy output or a pass-through? | `/home/pjy0422/hmg_migration/stack/humanoidmimicgen_repo/humanoidmimicgen/wbc/policy/__init__.py` (`G1DecoupledWholeBodyPolicy.get_action`; `IdentityPolicy`) |
| Which joints does HOMIE ONNX write? | `.../humanoidmimicgen/wbc/policy/g1_homie_policy.py` (`use_policy_action`, stand/walk ONNX) |
| Lower-body PD gains / action scale | `.../humanoidmimicgen/configs/wbc/g1_29dof_homie_v2.yaml`; `.../wbc/external_dependencies/sim2mujoco/resources/robots/g1/g1_homie.yaml` |
| Arm PD gains, kd, torque compensation | `.../humanoidmimicgen/locomanipulation/controllers/default_mink_ik_g1_homie_v2_grav_comp_tuned.json` (arms: kp `[300,300,300,300,800,800,800]`, kd `[8,8,8,8,15,15,15]`, `use_torque_compensation: true`; legs/torso in the same file) |
| Actuator torque limit (ctrlrange) and joint friction | `.../humanoidmimicgen/locomanipulation/models/assets/robots/unitree_g1/g1_29dof_rev_1_0.xml` — `<motor ctrlrange>`: shoulder/elbow ±25 N·m, wrist roll ±30, wrist pitch/yaw ±10; `frictionloss` 0.2 N·m (arm), 0.1 (wrist); damping 0.05; armature 0.01 |
| How commanded q becomes torque | `/home/pjy0422/hmg_migration/stack/hmg_venv/lib/python3.10/site-packages/robosuite/controllers/parts/generic/joint_pos.py` — `torques = M(q) @ (kp*e + kd*(-v)) + qfrc_bias`; base `torque_compensation = qfrc_bias`; `clip_torques` at the actuator limits (`robosuite/controllers/parts/controller.py`) |
| Action -> sim plumbing | `.../humanoidmimicgen/dataset_playback.py` (`ActionEnv.step`), `.../wbc/envs/locomanipulation/sync_env.py` (`convert_q_to_actuated_joint_order`, `queue_action`), `.../wbc_runtime.py`, and the harness `hmg_harness/adapter.py` |
| Teleop/mink IK path (what `use_joint_angle_action_input: false` means) | `.../humanoidmimicgen/locomanipulation/controllers/mink.py` (`IKSolverMink`, `HybridWholeBodyMinkIK`) |

## Known measured consequences (ll_v3, 2026-11-16)

- **The arms are pass-through.** The HOMIE ONNX policy writes only
  `lower_body_indices` (legs + waist); `upper_body_indices` come from the
  harness command. OpenHomie's own architecture is exoskeleton joint positions
  passed straight through.
- **Small arm commands are absorbed by joint friction, not refused by a
  policy.** The JPC torque is `M(q)*(kp*e + kd*(-v)) + bias`, clipped by
  `<motor ctrlrange>`. When `M(q)*kp*e < frictionloss`, the joint does not
  move: a static-friction deadband whose size scales with `1/M(q)`, hence
  posture-dependent. Measured on `05_drill_lift` (backoff, tuck): actuator
  force 0.57-2.91 N·m (well below ctrlrange); elbow 0.006 rad command ->
  0.0008 rad motion with the actuator at -2.9 N·m dominated by gravity
  compensation.
- **Consequences for skills:** command amplitude must exceed the measured
  deadband; use deadband compensation with a settle law (no limit cycle);
  record the deadband as an actuator property in the substrate contract, not
  as a WBC refusal. `min_gain_for_commands` is an amplitude map, not a global
  constant.
