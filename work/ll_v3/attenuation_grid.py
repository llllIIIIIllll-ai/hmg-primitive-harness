#!/usr/bin/env python3
"""3-factor WBC attenuation grid: position x joint(direction) x step size."""
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


def setup(pos):
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    if pos in ("standoff", "backoff"):
        qtool({"tool": "call", "name": "base_standoff_servo_v3",
               "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                          "object_match": "bottle"}})
    if pos == "backoff":
        qtool({"tool": "call", "name": "move_base_probe",
               "kwargs": {"x": -0.12, "dur": 4.5}})


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3", "attenuation_probe",
               "move_base_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    sizes = (0.005, 0.01, 0.02, 0.04, 0.08)
    joints = (("left_shoulder_roll_joint", 1.0),
              ("left_shoulder_roll_joint", -1.0),
              ("left_shoulder_pitch_joint", 1.0),
              ("left_elbow_joint", 1.0),
              ("left_wrist_pitch_joint", 1.0))
    out = {}
    for pos in ("spawn", "backoff", "standoff"):
        setup(pos)
        rows = []
        for joint, sign in joints:
            for sz in sizes:
                r = qtool({"tool": "call", "name": "attenuation_probe",
                           "kwargs": {"joint": joint, "step": sz,
                                      "steps": 5, "sign": sign}})
                res = r.get("result") or {}
                rows.append({"joint": joint, "sign": sign, "step": sz,
                             "gain": res.get("gain_mean"),
                             "q0": res.get("q_start"),
                             "q1": res.get("q_end")})
                print(pos, joint, "+" if sign > 0 else "-", sz, "gain",
                      None if res.get("gain_mean") is None
                      else round(res["gain_mean"], 3))
        out[pos] = rows
    json.dump(out, open(os.path.join(HERE, "reports", "w1_raw",
                                     "attenuation_grid.json"), "w"),
              indent=1)


if __name__ == "__main__":
    main()
