#!/usr/bin/env python3
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
TASK = "05_drill_lift"


def qtool(req, timeout=900):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run(["./scripts/qtool.sh", json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def setup(pos, posture):
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    qtool({"tool": "call", "name": "base_standoff_servo_v3",
           "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                      "object_match": "bottle"}})
    if pos == "backoff":
        qtool({"tool": "call", "name": "move_base_probe",
               "kwargs": {"x": -0.12, "dur": 7.0}})
    if posture == "down":
        qtool({"tool": "call", "name": "arm_untuck",
               "kwargs": {"dx": -0.126, "dy": 0.07, "z": 1.16,
                          "stop_err": 0.12, "max_steps": 250,
                          "max_delta": 0.04, "orient_mode": "down",
                          "orientation": "down", "joint_mode": True,
                          "deadband_comp": 0.012, "wrist_guard": True,
                          "wrist_guard_dist": 0.01, "seed_mode": "dls",
                          "sides": ("left",), "base_gain": 0.3,
                          "base_deadband": 0.005, "base_vmax": 0.05}})


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3", "arm_untuck",
               "arm_pose_ik_live", "move_base_probe", "walk_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    rows = []
    for pos in ("standoff", "backoff"):
        for posture in ("tucked", "down"):
            for dur in (2.0, 5.0):
                setup(pos, posture)
                r = qtool({"tool": "call", "name": "walk_probe",
                           "kwargs": {"x": 0.12, "dur": dur}})
                res = r.get("result") or {}
                rows.append({"pos": pos, "posture": posture, "dur": dur,
                             "dist": res.get("dist"),
                             "moved": res.get("moved"),
                             "b0": res.get("b0"), "b1": res.get("b1"),
                             "robot_contacts": res.get("robot_contacts")})
                b1 = res.get("b1") or {}
                print("%s %s dur %.0f: dist %s moved %s cmd %.2f speed %s tilt %s yaw %s contacts %d"
                      % (pos, posture, dur,
                         None if res.get("dist") is None
                         else round(res["dist"], 4),
                         res.get("moved"), 0.12 * dur,
                         None if res.get("dist") is None
                         else round(res["dist"] / dur, 4),
                         b1.get("tilt_deg"), b1.get("yaw_deg"),
                         len(res.get("robot_contacts") or [])))
    json.dump(rows, open(os.path.join(HERE, "reports", "w1_raw",
                                      "walk_grid.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
