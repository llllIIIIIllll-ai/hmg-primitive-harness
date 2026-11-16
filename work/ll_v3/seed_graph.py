#!/usr/bin/env python3
"""Seed the ll_v3 capability graph from work/ll_v3/reports/capability_design.md.

Uses only the typed graph_cli.py commands (no direct SQLite writes).
Idempotent: existing capability/strategy nodes are reused, and a strategy's
REQUIRES set is replaced with the design set.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))  # work/ll_v3 -> repo root
DB = os.path.join(ROOT, "experiments", "ll_v3", "skill_system.db")
CLI = os.path.join(ROOT, "graph_cli.py")

WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
ARMS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
HANDS = [
    "left_hand_index_0_joint", "left_hand_index_1_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "right_hand_index_0_joint", "right_hand_index_1_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
]
LEGS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
ALL_READ = ["base_pose", "all_43_joints"]


def cap(name, level, kind, group, contract, **kw):
    d = {"name": name, "level": level, "kind": kind, "group": group,
         "contract": contract}
    if kind in ("read", "decision"):
        d.setdefault("commands", [])
        d.setdefault("observed_exogenous", [])
        d.setdefault("assumes_fixed", [])
        d.setdefault("min_gain", None)
        d.setdefault("binding_constraint", None)
    d.update(kw)
    return d


CAPS = [
    # ---------------- L1 control ----------------
    cap("base_velocity_command", "L1", "control", "legs/base",
        "commanded velocity achieved within gain; base tilt/guard in bounds",
        channel="navigate_cmd (vx, vy, wz)",
        law="one velocity command channel",
        commands=["navigate_cmd"], observed_exogenous=ALL_READ,
        assumes_fixed=[], min_gain=None,
        binding_constraint="base tilt or foot slip",
        demand="9/9 (via standoff/04)", status="discovery",
        notes="WBC closes the velocity loop; not a joint command"),
    cap("base_navigate_to_pose", "L1", "control", "legs/base",
        "final xy/yaw error, tilt, guard",
        channel="navigate_cmd", law="pose error closed into velocity commands",
        commands=["navigate_cmd"], observed_exogenous=ALL_READ,
        assumes_fixed=[], min_gain=None, binding_constraint="tilt/slip",
        demand="9/9", status="discovery",
        notes="L1 despite feedback; worked example"),
    cap("base_height_setpoint", "L1", "control", "legs/base",
        "height error, tilt, guard",
        channel="base_height (scalar)", law="closed-loop height",
        commands=["base_height"], observed_exogenous=ALL_READ,
        assumes_fixed=[], min_gain=None,
        binding_constraint="height clamp/WBC refusal",
        demand="1/9 (01) + optional reach", status="discovery",
        notes="posture height, not a lift"),
    cap("waist_pose_servo", "L1", "control", "waist",
        "waist pose error, achieved gain >= 0.8, joint margin",
        channel="3 waist joints", law="joint-target servo",
        commands=WAIST,
        observed_exogenous=ARMS + HANDS + ["base_pose"],
        assumes_fixed=[], min_gain=0.8,
        binding_constraint="authority/gain",
        gate="waist_owner in {ik, blend}; verify achieved gain before/after; "
             "restore homie on exit",
        demand="0/9 direct (conditional)", status="discovery",
        notes="under homie observed_exogenous, not a command"),
    cap("ee_pose_ik", "L1", "control", "arms",
        "pad position/normal error, joint margin, binding constraint",
        channel="14 arm joints",
        law="one mink-QP solve of two palm poses (12-D)",
        commands=ARMS,
        observed_exogenous=WAIST + ["base_pose"],
        assumes_fixed=["base_pose (held by base-lock task)",
                       "hand_joints (pre-shape)"],
        min_gain=0.8,
        binding_constraint="pad error / joint margin / QP infeasible",
        demand="9/9", status="substrate (arm_pose_ik)",
        notes="waist measured exogenous while homie"),
    cap("ee_position_servo", "L1", "control", "arms",
        "position error, continuity, joint margin",
        channel="14 arm joints",
        law="Cartesian position error closed into joint targets",
        commands=ARMS,
        observed_exogenous=WAIST + ["base_pose"],
        assumes_fixed=["hand_joints (pre-shape/closed vector)"],
        min_gain=0.8, binding_constraint="reach/joint margin",
        demand="9/9", status="discovery",
        notes="withdraw/retreat verb; not a separate L1"),
    cap("joint_space_transport", "L1", "control", "arms",
        "joint tracking, ramp duration, joint margin",
        channel="14 arm joints",
        law="joint-space ramp with measured lag curve",
        commands=ARMS,
        observed_exogenous=WAIST + ["base_pose"],
        assumes_fixed=["hand_joints (explicit closed-finger vector held by "
                       "grasp_hold)"],
        min_gain=0.8, binding_constraint="ramp lag/joint margin",
        demand="6/9", status="discovery",
        notes="arms only; the lift L2 holds the fingers via grasp_hold"),
    cap("constrained_servo", "L1", "control", "arms",
        "clearance margin, joint margin",
        channel="14 arm joints",
        law="Cartesian servo with a clearance or joint-margin bound",
        commands=ARMS,
        observed_exogenous=WAIST + ["base_pose", "scene_aabbs"],
        assumes_fixed=["hand_joints (pre-shape/closed vector)"],
        min_gain=0.8, binding_constraint="clearance/joint margin",
        demand="9/9 (via C27)", status="discovery (W2b component)",
        notes="constraint inside the same channel"),
    cap("contact_push", "L1", "control", "arms",
        "contact duty, force bound, object displacement",
        channel="14 arm joints",
        law="advance along a direction, regulate contact force/duty, stop on "
            "force/pose",
        commands=ARMS,
        observed_exogenous=WAIST + ["base_pose"],
        assumes_fixed=["hand_joints (pre-shape/closed vector)"],
        min_gain=0.8, binding_constraint="slip/force",
        demand="2/9 (02, 04)", status="discovery",
        notes="push/press L2s add the predicate reads"),
    cap("hand_pre_shape", "L1", "control", "hands",
        "finger pose error, aperture, no object contact before close; "
        "per-side sign",
        channel="14 finger joints", law="posture preset (aperture/opposition)",
        commands=HANDS,
        observed_exogenous=WAIST + ARMS + ["base_pose"],
        assumes_fixed=["arm_joints (held by IK/servo)"],
        min_gain=0.8, binding_constraint="finger range/sign",
        demand="9/9", status="discovery",
        notes="the ll_v2 gap; left negative/right positive for index/middle"),
    cap("grasp_close_contact", "L1", "control", "hands",
        "bilateral contact, grip stability, object disturbance",
        channel="14 finger joints", law="close until contact",
        commands=HANDS,
        observed_exogenous=WAIST + ARMS + ["base_pose"],
        assumes_fixed=["arm_joints (held)"], min_gain=0.8,
        binding_constraint="no contact/slip",
        demand="7/9", status="substrate (hand_close_contact)",
        notes="contact is feedback inside the finger channel"),
    cap("grasp_close_force_limited", "L1", "control", "hands",
        "grip force bound, slip, object disturbance",
        channel="14 finger joints", law="close to a force/drift bound",
        commands=HANDS,
        observed_exogenous=WAIST + ARMS + ["base_pose"],
        assumes_fixed=["arm_joints (held)"], min_gain=0.8,
        binding_constraint="force limit",
        demand="0/9 required; 05 S2 competing candidate", status="competing",
        notes="competing stopping law to grasp_close_contact; merge only if "
              "the Validator shows mechanism-equivalent behaviour"),
    cap("grasp_hold", "L1", "control", "hands",
        "hold drift, contact retained, object not released",
        channel="14 finger joints",
        law="hold the last closed target every control step",
        commands=HANDS,
        observed_exogenous=WAIST + ARMS + ["base_pose"],
        assumes_fixed=["arm_joints (held)"], min_gain=0.8,
        binding_constraint="slip/release",
        demand="7/9", status="discovery",
        notes="explicit closed-finger vector; do not re-read measured fingers"),
    cap("grasp_open_release", "L1", "control", "hands",
        "contacts after open, object disturbance",
        channel="14 finger joints", law="open until contact is lost",
        commands=HANDS,
        observed_exogenous=WAIST + ARMS + ["base_pose"],
        assumes_fixed=["arm_joints (held)"], min_gain=0.8,
        binding_constraint="object disturbance",
        demand="0/9 (unclaimed)", status="substrate (hand_open)",
        notes="unloaded open; placement uses loaded_release"),
    cap("loaded_release", "L1", "control", "hands",
        "support contact at open, object settled, no drop",
        channel="14 finger joints",
        law="open while support contact/load is present",
        commands=HANDS,
        observed_exogenous=WAIST + ARMS + ["base_pose", "support_contact"],
        assumes_fixed=["arm_joints (held)"], min_gain=0.8,
        binding_constraint="no support contact",
        demand="2/9 (06, 07)", status="discovery",
        notes="placement opening law"),

    # ---------------- L1 read / decision ----------------
    cap("object_pose_tracking", "L1", "read", "sensing",
        "reports raw pose/orientation/AABB/speed/pad distance; never a task "
        "threshold",
        read_channel="task-object root pose + body-tree AABB + linear/angular "
                     "velocity + per-side pad-object distance",
        reports=["pose", "orientation", "aabb", "linear_velocity",
                 "angular_velocity", "pad_object_distance"],
        demand="9/9", status="substrate (object_pose_track)",
        notes="root-body z is the predicate quantity; AABB is for "
              "clearance/support"),
    cap("scene_geometry_tracking", "L1", "read", "sensing",
        "entity pose/AABB/support top/planar centre; zero-geom roots walked; "
        "plane geoms by body pose",
        read_channel="static scene body poses + body-tree AABBs + joint axes",
        reports=["entity_pose", "aabb", "support_top_z", "planar_centre",
                 "joint_axis", "floor_plane"],
        demand="9/9", status="discovery",
        notes="tables, boards, target zone, holder, obstacle, floor"),
    cap("scene_joint_state_tracking", "L1", "read", "sensing",
        "joint 0 qpos for 02 (control_panel_button_z), lever qpos, "
        "shelf-wheel qpos; raw qpos",
        read_channel="scene-object joint qpos, type, range, axis, per-object "
                     "joint order/name",
        reports=["joint_qpos", "joint_type", "joint_range", "joint_axis",
                 "joint_index"],
        demand="1/9 (02)", status="discovery",
        notes="panel joint 0 is the button slide, not the lever"),
    cap("contact_sensing", "L1", "read", "sensing",
        "object-table, object-holder, shelf-box, drill-target-table, "
        "hand-object pairs; truthful for child-geom roots",
        read_channel="sim contact pairs over body trees",
        reports=["body_pairs", "geom_pairs", "distance", "penetration",
                 "normal", "force"],
        demand="9/9", status="discovery",
        notes="do not use observe()['hand_object_contacts'] alone"),
    cap("grasp_state_tracking", "L1", "read", "sensing",
        "proxy for the benchmark grasp test (IsGrasped not exposed)",
        read_channel="hand-object state: bilateral pad contact, aperture, "
                     "relative pose/velocity, stability",
        reports=["bilateral_contact", "aperture", "relative_pose",
                 "relative_velocity", "stability"],
        demand="7/9", status="discovery",
        proxy_for="benchmark IsGrasped (not exposed)", gap="G1",
        notes="validate against task outcomes; never read/imitate the predicate"),
    cap("safety_limit_monitor", "L1", "decision", "sensing",
        "flags/aborts before a limit; never commands",
        read_channel="joint margin, base tilt, penetration, force",
        reports=["joint_margin", "base_tilt", "penetration", "force",
                 "guard_status"],
        demand="9/9", status="substrate (safety_guard)",
        notes="invariant/guard; no command"),
    cap("failure_visualization", "L1", "read", "sensing/debug",
        "failure onset/end + rendered frames; never reads the success "
        "predicate",
        read_channel="recorded trace",
        reports=["failure_onset_step", "failure_end_step", "rendered_frames",
                 "visual_finding"],
        demand="0/9 (debug)", status="substrate (debug_visual)",
        debug_only=True,
        notes="actor tool, not a task capability"),

    # ---------------- L2 compositions ----------------
    cap("base_navigate_to_standoff", "L2", "composition", "whole_body/base",
        "standoff error, final xy/yaw error, tilt, guard",
        requires=["base_velocity_command", "object_pose_tracking",
                  "safety_limit_monitor"],
        binding_constraint="standoff/tilt", demand="9/9",
        status="substrate (base_standoff_servo)",
        notes="nav + object tracking + safety; selects standoff from the live "
              "AABB"),
    cap("control_authority_handover", "L2", "composition", "whole_body",
        "HANDOFF event, gain before/after, restore, balance in bounds",
        requires=["waist_pose_servo", "base_velocity_command",
                  "safety_limit_monitor"],
        binding_constraint="gain/balance",
        gate="waist owner homie|ik|blend; lower owner policy|hold; restore "
             "default on exit",
        demand="0/9 direct (warm-up gate)", status="substrate "
        "(authority_handover)",
        notes="switches channel ownership; lower owner hold is a mode, not a "
              "leg verb"),
    cap("whole_body_ik_17d", "L2", "composition", "whole_body",
        "pad error, waist gain before/after, joint margin",
        requires=["ee_pose_ik", "waist_pose_servo"],
        binding_constraint="pad error/gain",
        gate="waist_owner=ik (authorized handover); verify gain before/after",
        demand="0/9 (candidate)", status="candidate",
        retire_condition="retire if round 0 does not adopt 17-D",
        notes="17-D adds the 3 waist joints to the 12-D solve"),
    cap("whole_body_ik_14d", "L2", "composition", "whole_body",
        "pad error, base placement error, joint margin",
        requires=["base_navigate_to_pose", "base_height_setpoint", "ee_pose_ik"],
        binding_constraint="pad error/base placement",
        demand="0/9 (candidate)", status="candidate",
        retire_condition="retire if round 0 does not adopt 14-D",
        notes="base placement + arm IK; base held while arms solve"),
    cap("collision_free_reach_trajectory", "L2", "composition", "whole_body",
        "pad error, clearance margin, joint margin",
        requires=["ee_pose_ik", "object_pose_tracking", "constrained_servo",
                  "safety_limit_monitor"],
        binding_constraint="clearance/reach", demand="9/9",
        status="W2b closure (missing W2 capability)",
        notes="IK + tracking + constrained servo + safety; not batch 1"),
    cap("bimanual_grasp_object", "L2", "composition", "whole_body",
        "bilateral contact, stability, object disturbance",
        requires=["hand_pre_shape", "ee_pose_ik", "grasp_close_contact"],
        binding_constraint="no contact/slip", demand="7/9",
        status="discovery",
        notes="pre-shape + two-pad IK + contact close, sequenced"),
    cap("lift_object", "L2", "composition", "whole_body",
        "object root-z rise vs support top, grip retained, contact retained",
        requires=["bimanual_grasp_object", "joint_space_transport",
                  "grasp_hold", "object_pose_tracking",
                  "scene_geometry_tracking", "contact_sensing",
                  "grasp_state_tracking"],
        binding_constraint="slip/rise", demand="6/9", status="discovery",
        notes="joint-space transport with the explicit closed-finger vector "
              "held; not a base-height ramp"),
    cap("place_object", "L2", "composition", "whole_body",
        "support contact at open, object settled/static/upright, pad-object "
        "distance",
        requires=["joint_space_transport", "grasp_hold", "loaded_release",
                  "ee_position_servo", "object_pose_tracking",
                  "scene_geometry_tracking", "contact_sensing"],
        binding_constraint="no support contact/drop", demand="2/9 (06, 07)",
        status="discovery",
        notes="loaded-release law; transport + hold + release + retreat"),
    cap("push_object_to_target", "L2", "composition", "whole_body",
        "shelf upright, shelf-target planar distance, shelf-box contact, tilt",
        requires=["contact_push", "base_velocity_command",
                  "object_pose_tracking", "scene_geometry_tracking",
                  "contact_sensing", "safety_limit_monitor"],
        binding_constraint="slip/topple", demand="1/9 (04)", status="discovery",
        notes="measures the task predicate, not only hand-object duty"),
    cap("press_panel_button", "L2", "composition", "whole_body",
        "panel joint 0 qpos displacement, tool contact, force bound, tilt",
        requires=["contact_push", "contact_sensing",
                  "scene_joint_state_tracking", "safety_limit_monitor"],
        binding_constraint="no joint motion/slip", demand="1/9 (02)",
        status="discovery",
        notes="measures the panel joint 0, not only tool contact"),
    cap("extract_object_from_holder", "L2", "composition", "whole_body",
        "drill-holder contact cleared, grasp retained, retract distance",
        requires=["bimanual_grasp_object", "ee_position_servo", "grasp_hold",
                  "contact_sensing", "scene_geometry_tracking"],
        binding_constraint="holder collision/slip", demand="1/9 (08)",
        status="discovery",
        notes="grasp + Cartesian retract + hold + holder read"),
]

STRATEGIES = [
    # task, name, direct capabilities, description
    ("01_box_lift_floor", "floor_standoff_grasp_lift",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "bimanual_grasp_object", "lift_object"],
     "standoff -> collision-free reach -> bimanual grasp -> lift"),
    ("01_box_lift_floor", "floor_crouch_direct_grasp_lift",
     ["base_navigate_to_pose", "base_height_setpoint", "ee_position_servo",
      "hand_pre_shape", "ee_pose_ik", "grasp_close_contact",
      "joint_space_transport", "grasp_hold", "object_pose_tracking",
      "contact_sensing", "grasp_state_tracking", "safety_limit_monitor"],
     "crouch + explicit pose -> direct Cartesian approach -> pre-shape/IK/close "
     "-> joint-space transport with hold"),
    ("02_push_button", "standoff_reach_press",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "hand_pre_shape", "press_panel_button"],
     "standoff -> collision-free reach -> pre-shape -> press the panel joint"),
    ("02_push_button", "direct_cartesian_press",
     ["base_navigate_to_pose", "ee_position_servo", "contact_push",
      "scene_joint_state_tracking", "contact_sensing",
      "scene_geometry_tracking", "safety_limit_monitor"],
     "explicit pose -> Cartesian approach -> contact push with panel joint read"),
    ("03_box_lift", "table_standoff_grasp_lift",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "bimanual_grasp_object", "lift_object"],
     "standoff -> collision-free reach -> bimanual grasp -> lift"),
    ("03_box_lift", "table_direct_grasp_lift",
     ["base_navigate_to_pose", "ee_position_servo", "hand_pre_shape",
      "ee_pose_ik", "grasp_close_contact", "joint_space_transport",
      "grasp_hold", "object_pose_tracking", "scene_geometry_tracking",
      "contact_sensing", "grasp_state_tracking", "safety_limit_monitor"],
     "explicit pose -> direct Cartesian approach -> pre-shape/IK/close -> "
     "joint-space transport with hold"),
    ("04_push_shelf_forward", "standoff_push_to_target",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "hand_pre_shape", "push_object_to_target"],
     "standoff -> collision-free reach -> pre-shape -> push while measuring "
     "upright/target/shelf-box contact"),
    ("04_push_shelf_forward", "walk_and_push_tracking",
     ["base_navigate_to_pose", "base_velocity_command", "ee_position_servo",
      "contact_push", "object_pose_tracking", "scene_geometry_tracking",
      "contact_sensing", "safety_limit_monitor"],
     "explicit pose + walk velocity -> contact push with live tracking"),
    ("05_drill_lift", "drill_standoff_grasp_lift",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "bimanual_grasp_object", "lift_object"],
     "standoff -> collision-free reach -> bimanual grasp -> lift (concentrate "
     "task)"),
    ("05_drill_lift", "drill_direct_grasp_lift",
     ["base_navigate_to_pose", "base_height_setpoint", "ee_position_servo",
      "hand_pre_shape", "ee_pose_ik", "grasp_close_contact",
      "joint_space_transport", "grasp_hold", "object_pose_tracking",
      "scene_geometry_tracking", "contact_sensing", "grasp_state_tracking",
      "safety_limit_monitor"],
     "explicit pose + height -> direct Cartesian approach -> pre-shape/IK/close "
     "-> joint-space transport with hold; grasp_close_force_limited is the "
     "competing candidate"),
    ("06_drill_pnp", "drill_pick_lift_place",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "bimanual_grasp_object", "lift_object", "place_object"],
     "standoff -> collision-free reach -> grasp -> lift -> loaded-release place "
     "on the target table"),
    ("06_drill_pnp", "drill_pick_transport_release",
     ["base_navigate_to_pose", "ee_position_servo", "hand_pre_shape",
      "ee_pose_ik", "grasp_close_contact", "joint_space_transport",
      "grasp_hold", "loaded_release", "object_pose_tracking",
      "scene_geometry_tracking", "contact_sensing", "grasp_state_tracking",
      "safety_limit_monitor"],
     "explicit pose -> direct approach -> grasp -> transport -> loaded release"),
    ("07_box_table_to_shelf", "box_pick_lift_place_board1",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "bimanual_grasp_object", "lift_object", "place_object"],
     "standoff -> collision-free reach -> grasp -> lift -> loaded-release place "
     "on shelf board 1"),
    ("07_box_table_to_shelf", "box_direct_transport_release",
     ["base_navigate_to_pose", "ee_position_servo", "hand_pre_shape",
      "ee_pose_ik", "grasp_close_contact", "joint_space_transport",
      "grasp_hold", "loaded_release", "object_pose_tracking",
      "scene_geometry_tracking", "contact_sensing", "grasp_state_tracking",
      "safety_limit_monitor"],
     "explicit pose -> direct approach -> grasp -> transport -> loaded release "
     "on board 1"),
    ("08_pick_drill_from_holder", "holder_standoff_grasp_extract",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "bimanual_grasp_object", "extract_object_from_holder"],
     "standoff -> collision-free reach -> grasp -> Cartesian retract from the "
     "holder"),
    ("08_pick_drill_from_holder", "holder_direct_grasp_retract",
     ["base_navigate_to_pose", "ee_position_servo", "hand_pre_shape",
      "ee_pose_ik", "grasp_close_contact", "grasp_hold", "contact_sensing",
      "scene_geometry_tracking", "grasp_state_tracking",
      "safety_limit_monitor"],
     "explicit pose -> direct approach -> grasp -> retract while measuring "
     "holder contact"),
    ("09_obstacle_aware_pick_drill", "obstacle_standoff_clearance_grasp_lift",
     ["base_navigate_to_standoff", "collision_free_reach_trajectory",
      "bimanual_grasp_object", "lift_object"],
     "obstacle-aware standoff -> clearance reach -> grasp -> lift"),
    ("09_obstacle_aware_pick_drill",
     "obstacle_direct_clearance_grasp_lift",
     ["base_navigate_to_pose", "base_height_setpoint", "ee_position_servo",
      "constrained_servo", "hand_pre_shape", "ee_pose_ik",
      "grasp_close_contact", "joint_space_transport", "grasp_hold",
      "object_pose_tracking", "scene_geometry_tracking", "contact_sensing",
      "grasp_state_tracking", "safety_limit_monitor"],
     "explicit pose + height -> constrained approach around the obstacle -> "
     "grasp -> transport with hold"),
]


def cli(*args):
    p = subprocess.run([sys.executable, CLI, "--db", DB] + list(args),
                       capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError("graph_cli failed: %s\nstdout=%s\nstderr=%s"
                           % (" ".join(args), p.stdout, p.stderr))
    out = json.loads(p.stdout)
    if not out.get("ok"):
        raise RuntimeError("graph_cli error: %s" % p.stdout)
    return out["result"]


def export_graph():
    return cli("export")


def main():
    ex = export_graph()
    names = {n["name"]: n for n in ex["nodes"]}
    cap_names = {n["name"] for n in ex["nodes"] if n["type"] == "CAPABILITY"}
    strat_names = {n["name"] for n in ex["nodes"] if n["type"] == "STRATEGY"}

    created_caps = 0
    for c in CAPS:
        if c["name"] in cap_names:
            continue
        spec = {k: v for k, v in c.items() if k not in ("name", "contract")}
        spec["contract"] = c["contract"]
        cli("propose_capability", "--name", c["name"],
            "--description", c["contract"],
            "--spec", json.dumps(spec, ensure_ascii=True),
            "--level", c["level"])
        created_caps += 1
    print("capabilities created:", created_caps)

    # L2 -> L1/L2 REQUIRES edges (INSERT OR IGNORE, idempotent)
    for c in CAPS:
        if c["level"] != "L2":
            continue
        cli("add_requires", "--capability", c["name"],
            "--requires", ",".join(c["requires"]))
    print("L2 requires edges ensured")

    created_strats = 0
    replaced_strats = 0
    for task, name, caps, desc in STRATEGIES:
        if name in strat_names:
            cli("set_strategy_capabilities", "--strategy", name,
                "--capabilities", ",".join(caps))
            replaced_strats += 1
        else:
            cli("propose_strategy", "--task", task, "--name", name,
                "--description", desc, "--capabilities", ",".join(caps))
            created_strats += 1
    print("strategies created:", created_strats,
          "replaced:", replaced_strats)
    print("seed complete")


if __name__ == "__main__":
    main()
