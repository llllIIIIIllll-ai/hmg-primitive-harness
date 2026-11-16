#!/usr/bin/env python3
"""Link the W1 substrate skills and record the available Validator verdicts."""
import hashlib
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DB = os.path.join(ROOT, "experiments", "ll_v3", "skill_system.db")
CLI = os.path.join(ROOT, "graph_cli.py")
SUB = os.path.join(HERE, "substrate")

LINKS = [
    ("base_navigate_to_standoff", "base_standoff_servo",
     "base_standoff_servo.py",
     "measured standoff from the live object AABB, closed-loop bursts, "
     "fingertip guard, tilt guard; reports standoff error, final xy/yaw error, "
     "base tilt, guard status"),
    ("ee_pose_ik", "arm_pose_ik", "arm_pose_ik.py",
     "two palm poses (12-D), measured waist exogenous, mink QP default (DLS "
     "fallback), joint margin >= 0.05 rad, reports pad position/normal error "
     "and binding constraint"),
    ("grasp_close_contact", "hand_close_contact", "hand_close_contact.py",
     "contact-triggered close, reports contact pairs, grip stability, object "
     "disturbance"),
    ("grasp_open_release", "hand_open", "hand_open.py",
     "open to the neutral target, reports contacts after open and object "
     "disturbance"),
    ("object_pose_tracking", "object_pose_track", "object_pose_track.py",
     "live pose/AABB from the scene, reports pose error and tracking gap"),
    ("safety_limit_monitor", "safety_guard", "safety_guard.py",
     "joint margin, base tilt, penetration; aborts or flags before a limit, "
     "never commands"),
    ("failure_visualization", "debug_visual", "debug_visual.py",
     "find the failure onset/end steps in the trace, render the frames, report "
     "the onset/end steps plus the visual finding; never reads the predicate"),
    ("control_authority_handover", "authority_handover",
     "authority_handover.py",
     "runtime handover and re-handover of the waist channel; verifies the "
     "achieved gain before and after, records HANDOFF events, restores homie, "
     "requires balance in bounds"),
]

VERDICTS = {
    "base_standoff_servo": {
        "verdict": "accept",
        "metrics": {
            "S": [0.25, 0.20, 0.15],
            "standoff_error": [0.01565, 0.01517, 0.01601],
            "tilt_deg": [4.19, 1.87, 1.60],
            "guards_trip_when_forced": ["tilt_guard", "fingertip_guard"],
        },
        "binding": None,
    },
    "arm_pose_ik": {
        "verdict": "accept",
        "metrics": {
            "solver": "mink-qp", "waist_owner": "homie",
            "waist_exogenous": True,
            "joint_margin_rad": [0.3705, 0.975],
            "pad_error_m": [0.0945, 0.0781],
            "ramp_s": 0.35, "hold_s": 0.2,
            "pad_0p015_gate_met": False,
            "note": "contract pass; the 15 mm pad gate is the W2 gate and the "
                    "base drift during the arm ramp is its binding constraint",
        },
        "binding": "pad_error (W2 gate; base drift)",
    },
    "hand_close_contact": {
        "verdict": "restrict",
        "metrics": {
            "closed_angle": 0.8, "steps": 9,
            "bilateral_at_close": False,
            "contacts": {"left": 0, "right": 1},
            "object_disturbance_m": 0.06838,
            "upstream_pregrasp_pad_error_m": 0.07268,
        },
        "binding": "object_disturbance; restrict to pregrasp pad error <= "
                   "0.015 m (upstream arm_pose_ik)",
    },
    "object_pose_track": {
        "verdict": "accept",
        "metrics": {
            "pose_error_static_m": 0.00052,
            "pose_error_post_motion_m": 0.00066,
            "tracking_gap_m": 0.03367,
            "aabb_match_m": {"pre": 0.001, "post": 0.0001},
        },
        "binding": None,
    },
    "authority_handover": {
        "verdict": "accept",
        "metrics": {
            "roll_gains": {"before": 0.326, "ik": 0.987, "after": -0.256},
            "owner_restored": "homie",
            "max_tilt_deg": 7.17,
            "handoff_events": ["before", "handoff", "restore"],
            "lower_owner_switched": False,
        },
        "binding": None,
    },
}


def cli(*args):
    p = subprocess.run([sys.executable, CLI, "--db", DB] + list(args),
                       capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError("graph_cli failed: %s\n%s\n%s"
                           % (" ".join(args), p.stdout, p.stderr))
    out = json.loads(p.stdout)
    if not out.get("ok"):
        raise RuntimeError("graph_cli error: %s" % p.stdout)
    return out["result"]


def main():
    export = cli("export")
    names = {n["name"]: n for n in export["nodes"]}
    for cap, skill, f, contract in LINKS:
        path = os.path.join(SUB, f)
        h = hashlib.sha256(open(path, "rb").read()).hexdigest()
        if skill in names:
            print("skill exists", skill)
            continue
        cli("link_skill", "--capability", cap, "--name", skill,
            "--description", contract, "--code-path",
            os.path.relpath(path, ROOT), "--source-hash", h,
            "--kwargs", "{}", "--contract",
            json.dumps({"contract": contract, "version": "v1"}),
            "--version", "v1")
        print("linked", skill, "->", cap, h[:12])
    export = cli("export")
    names = {n["name"]: n for n in export["nodes"]}
    for skill, v in VERDICTS.items():
        if skill not in names:
            print("missing skill", skill)
            continue
        ev = os.path.join(HERE, "reports", "w1_validation_%s.json" % skill)
        cli("submit_validation", "--skill", skill, "--verdict", v["verdict"],
            "--metrics", json.dumps({"metrics": v["metrics"],
                                     "binding_constraint": v["binding"]}),
            "--evidence", os.path.relpath(ev, ROOT),
            "--scenario-set", "w1_micro",
            "--seed-domain", "05_drill_lift seed 0",
            "--version", "v1")
        print("validated", skill, v["verdict"])
    print("done")


if __name__ == "__main__":
    main()
