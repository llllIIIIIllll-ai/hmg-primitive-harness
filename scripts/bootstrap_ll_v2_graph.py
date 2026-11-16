#!/usr/bin/env python3
"""Bootstrap the ll_v2 capability graph: 9 tasks, >=2 strategies each.

Capability-first design.  The six warm-up substrate capabilities keep the exact
names from `experiments/ll_v2/config.yaml`.  Every L2 declares its L1
dependencies; a leaf L2 is a graph defect, so the REQUIRES closure is checked
after the bootstrap.

Usage:
  python scripts/bootstrap_ll_v2_graph.py [--db experiments/ll_v2/skill_system.db]
"""
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(ROOT, "graph_cli.py")

# name -> (level, description)
CAPS = {
    # --- warm-up substrate (config.warmup.substrate) ---
    "base_navigate_to_standoff": ("L1", "Measured-standoff base approach from the live object AABB with fingertip/tilt guards."),
    "ee_pose_ik": ("L1", "Two palm poses (12-D) over the 14 arm joints; measured waist exogenous; mink QP default."),
    "grasp_close_contact": ("L1", "Contact-triggered finger close inside the hand joint range."),
    "grasp_open_release": ("L1", "Contact-triggered finger open with post-open contact/disturbance report."),
    "object_pose_tracking": ("L1", "Live object pose/AABB read with tracked pose error and gap."),
    "safety_limit_monitor": ("L1", "Joint margin, base tilt and penetration monitor; aborts before a limit."),
    # --- discovery-loop L1 candidates ---
    "ee_position_servo": ("L1", "Closed-loop Cartesian point servo for one end effector."),
    "ee_orientation_align": ("L1", "Closed-loop palm-axis alignment to a world/body direction."),
    "base_navigate_to_pose": ("L1", "Closed-loop base servo to a world x/y/yaw pose."),
    "base_height_setpoint": ("L1", "Closed-loop base height setpoint through the WBC."),
    "joint_trajectory_tracking": ("L1", "Ramp a joint vector to a target following the measured lag curve."),
    "object_lift_hold": ("L1", "Raise and hold a grasped object with the grip vector frozen."),
    "contact_force_push": ("L1", "Sustained contact push with contact-duty and target-progress measurement."),
    "object_place_on_surface": ("L1", "Lower a held object onto a target surface with static/upright/contact checks."),
    # --- L2 compositions ---
    "bimanual_grasp_object": ("L2", "Place two pads and close to bilateral contact."),
    "lift_and_hold_object": ("L2", "Lift a grasped object and hold it without slip."),
    "carry_object_between_poses": ("L2", "Carry a held object between base poses."),
    "place_object_on_target": ("L2", "Carry to a target pose and place with contact/upright checks."),
    "push_object_to_target": ("L2", "Push an object to a live target zone and stop on the true gap."),
    "press_surface_to_stop": ("L2", "Press a control surface until its measured displacement stops."),
    "reach_along_axis_and_extract": ("L2", "Reach along an object axis, grasp, and extract it."),
    "obstacle_aware_approach": ("L2", "Approach with a collision-free corridor around an obstacle."),
    "collision_free_reach_trajectory": ("L2", "Reach a pose without link/object interpenetration."),
}

L2_DEPS = {
    "bimanual_grasp_object": ["ee_pose_ik", "grasp_close_contact", "object_pose_tracking"],
    "lift_and_hold_object": ["object_lift_hold", "grasp_close_contact"],
    "carry_object_between_poses": ["object_lift_hold", "base_navigate_to_pose", "safety_limit_monitor"],
    "place_object_on_target": ["carry_object_between_poses", "object_place_on_surface", "object_pose_tracking"],
    "push_object_to_target": ["contact_force_push", "base_navigate_to_standoff", "object_pose_tracking"],
    "press_surface_to_stop": ["ee_position_servo", "ee_orientation_align", "object_pose_tracking"],
    "reach_along_axis_and_extract": ["ee_pose_ik", "grasp_close_contact", "base_navigate_to_standoff"],
    "obstacle_aware_approach": ["base_navigate_to_standoff", "object_pose_tracking", "safety_limit_monitor"],
    "collision_free_reach_trajectory": ["ee_pose_ik", "safety_limit_monitor"],
}

STRATEGIES = {
    "01_box_lift_floor": [
        ("S1_bimanual_pad_lift", ["base_navigate_to_standoff", "ee_pose_ik", "grasp_close_contact", "object_lift_hold", "safety_limit_monitor"]),
        ("S2_one_arm_side_lift", ["ee_position_servo", "ee_orientation_align", "grasp_close_contact", "object_lift_hold"]),
    ],
    "02_push_button": [
        ("S1_index_press", ["ee_position_servo", "ee_orientation_align", "press_surface_to_stop", "object_pose_tracking"]),
        ("S2_contact_duty_press", ["contact_force_push", "ee_position_servo", "press_surface_to_stop"]),
    ],
    "03_box_lift": [
        ("S1_opposed_pad_lift", ["base_navigate_to_standoff", "ee_pose_ik", "grasp_close_contact", "object_lift_hold"]),
        ("S2_top_grasp_lift", ["ee_pose_ik", "ee_orientation_align", "grasp_close_contact", "object_lift_hold"]),
    ],
    "04_push_shelf_forward": [
        ("S1_shoulder_push", ["contact_force_push", "base_navigate_to_standoff", "object_pose_tracking", "push_object_to_target"]),
        ("S2_hand_push", ["ee_position_servo", "contact_force_push", "push_object_to_target"]),
    ],
    "05_drill_lift": [
        ("S1_side_pinch_lift", ["base_navigate_to_standoff", "ee_pose_ik", "grasp_close_contact", "object_lift_hold", "safety_limit_monitor"]),
        ("S2_axial_extract", ["reach_along_axis_and_extract", "grasp_close_contact", "object_lift_hold"]),
    ],
    "06_drill_pnp": [
        ("S1_pick_carry_place", ["bimanual_grasp_object", "carry_object_between_poses", "place_object_on_target", "safety_limit_monitor"]),
        ("S2_axial_place", ["reach_along_axis_and_extract", "object_lift_hold", "place_object_on_target"]),
    ],
    "07_box_table_to_shelf": [
        ("S1_pick_carry_place", ["bimanual_grasp_object", "carry_object_between_poses", "place_object_on_target"]),
        ("S2_push_place", ["push_object_to_target", "place_object_on_target"]),
    ],
    "08_pick_drill_from_holder": [
        ("S1_axial_extract", ["reach_along_axis_and_extract", "grasp_close_contact", "object_lift_hold"]),
        ("S2_side_grasp_extract", ["base_navigate_to_standoff", "ee_pose_ik", "grasp_close_contact", "object_lift_hold"]),
    ],
    "09_obstacle_aware_pick_drill": [
        ("S1_obstacle_avoid_extract", ["obstacle_aware_approach", "collision_free_reach_trajectory", "bimanual_grasp_object", "object_lift_hold"]),
        ("S2_side_reach_extract", ["base_navigate_to_standoff", "collision_free_reach_trajectory", "ee_pose_ik", "grasp_close_contact", "object_lift_hold"]),
    ],
}


def call(db, *args):
    cmd = [sys.executable, CLI, "--db", db] + [str(a) for a in args]
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    try:
        out = json.loads(p.stdout)
    except ValueError:
        raise SystemExit("graph_cli failed: %s\n%s\n%s"
                         % (" ".join(cmd), p.stdout, p.stderr))
    if not out.get("ok"):
        raise SystemExit("graph_cli error: %s\n%s" % (" ".join(cmd), out))
    return out["result"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", default="experiments/ll_v2/skill_system.db")
    args = ap.parse_args()
    db = os.path.join(ROOT, args.db) if not os.path.isabs(args.db) else args.db
    os.makedirs(os.path.dirname(db), exist_ok=True)

    init = call(db, "init")
    for name, (level, desc) in CAPS.items():
        call(db, "propose_capability", "--name", name,
             "--description", desc, "--level", level)

    for name, deps in L2_DEPS.items():
        call(db, "add_requires", "--capability", name,
             "--requires", ",".join(deps))

    for task, strategies in STRATEGIES.items():
        for sname, caps in strategies:
            call(db, "propose_strategy", "--task", task, "--name", sname,
                 "--description", "bootstrap strategy for %s" % task,
                 "--capabilities", ",".join(caps))

    cov = call(db, "coverage")
    out = {"db": db, "tasks": init.get("tasks"),
           "capabilities": len(CAPS), "strategies": sum(
               len(v) for v in STRATEGIES.values()),
           "uncovered_union": cov.get("uncovered_union"),
           "complete": cov.get("complete")}
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
