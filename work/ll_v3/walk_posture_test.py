#!/usr/bin/env python3
"""Walk gain vs arm posture: tucked (standoff) vs extended down pre-grasp."""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
TASK = "05_drill_lift"


def qtool(req, timeout=600):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run(["./scripts/qtool.sh", json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def prep():
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    qtool({"tool": "call", "name": "base_standoff_servo_v3",
           "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                      "object_match": "bottle"}})


def walk():
    r = qtool({"tool": "call", "name": "move_base_probe",
               "kwargs": {"x": 0.12, "dur": 2.0}})
    res = r.get("result") or {}
    return res.get("moved_xy")


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3", "arm_untuck",
               "arm_pose_ik_live", "move_base_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    prep()
    a = walk()
    print("walk gain TUCKED:", a)
    prep()
    qtool({"tool": "call", "name": "move_base_probe",
           "kwargs": {"x": -0.12, "dur": 4.5}})
    u = qtool({"tool": "call", "name": "arm_untuck",
               "kwargs": {"dx": -0.126, "dy": 0.07, "z": 1.16,
                          "stop_err": 0.12, "max_steps": 250,
                          "max_delta": 0.04, "orient_mode": "down",
                          "orientation": "down", "joint_mode": True,
                          "deadband_comp": 0.012, "wrist_guard": True,
                          "wrist_guard_dist": 0.01, "seed_mode": "dls",
                          "sides": ("left",), "base_gain": 0.3,
                          "base_deadband": 0.005, "base_vmax": 0.05}})
    ur = u.get("result") or {}
    print("down pre-grasp ok", ur.get("ok"), "worst", ur.get("worst_pad_error"))
    b = walk()
    print("walk gain DOWN-PRE-GRASP:", b)


if __name__ == "__main__":
    main()
