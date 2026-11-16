#!/usr/bin/env python3
"""Round 0 deliverable 2: ramp-lag curve (time to 95% of a step command)."""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "round0_raw")
SRC = open(os.path.join(HERE, "primitives", "calib_ramp.py"),
           encoding="utf-8").read()
TASK = "05_drill_lift"
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")


def qtool(req, timeout=180):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 20)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def main():
    qtool({"tool": "write", "name": "calib_ramp", "source": SRC})
    out = {"task": TASK, "seed": 0, "n_steps": 250,
           "primitive": "calib_ramp", "measurements": {}}
    cases = []
    for joint in ("right_elbow_joint", "left_shoulder_pitch_joint"):
        for delta in (0.1, 0.3, 0.5, 1.0):
            cases.append((joint, delta))
    cases.append(("right_elbow_joint", -0.5))
    for joint, delta in cases:
        qtool({"tool": "reset", "seed": 0})
        r = qtool({"tool": "call", "name": "calib_ramp",
                   "kwargs": {"joint": joint, "delta": delta,
                              "n_steps": 250}})
        key = "%s:%+.1f" % (joint, delta)
        out["measurements"][key] = r["result"]
        print(key, "t95", r["result"].get("t95_s"),
              "frac", r["result"].get("achieved_frac"))
    os.makedirs(RAW, exist_ok=True)
    with open(os.path.join(RAW, "ramp_all.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print("wrote ramp_all.json")


if __name__ == "__main__":
    main()
