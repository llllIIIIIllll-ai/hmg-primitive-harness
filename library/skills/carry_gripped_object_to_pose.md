# carry_gripped_object_to_pose -- open before calling

Signature: carry_gripped_object_to_pose(x=0.15, y=0.55, yaw_deg=90.0, tol=0.06, yaw_tol=8.0, max_iter=10, upper=None, recover=2, recover_iter=6, recover_dz=0.02, recover_yaw_tol=8.0, trace_obj=True)
- source hash: 7679c6d0ff85eab19a68e8680f7f88e6b45632b894ae5d03020aaaa6b21fd645 (06_drill_pnp/actor_extract/skills/carry_gripped_object_to_pose.py)
- removes: carrying at the height inherited from a lift -- the held object's lowest collider drags a table edge, the base velocity saturates and heading plateaus short of the goal.
- applies when: a grasp is held and the carry posture is the one captured after the clearance raise.
- contract: pre: grasp held -> post: base near the target pose (8-10 correction windows measured, yaw error 1.89-7.75 deg against 8) with the object pose measured; raises `recover_dz` and re-walks if short. Side effects: the servo's per-window reads plus object/contact tracing.
- verified scope: 06_drill_pnp, drill, seeds 0-9. The stall-recovery branch NEVER fired (every heading was already inside 8 deg) -- do not rely on it. Base tilt reaches 5.03 deg during the carry.
- used by: 06_drill_pnp composition.
- CAUTION: the stall-recovery branch never fired -- do not rely on it; base tilt reaches 5.03 deg during the carry.

Proof history: `library/evidence/carry_gripped_object_to_pose.md`.
