#!/usr/bin/env python3
"""Primitive smoke test: arm_pose_ik_live's first step must complete and
return seed_choice / seed_surface (caught the k-shadowing TypeError)."""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
TASK = "05_drill_lift"


def qtool(req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run(["./scripts/qtool.sh", json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def main():
    qtool({"tool": "write", "name": "arm_pose_ik_live",
           "source": open(os.path.join(SUB, "arm_pose_ik_live.py")).read()})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    r = qtool({"tool": "call", "name": "arm_pose_ik_live",
               "kwargs": {"grasp": {"dx": -0.20, "dy": 0.14, "z": 0.94},
                          "max_steps": 1, "budget": False,
                          "seed_mode": "dls", "wrist_guard": True,
                          "wrist_guard_dist": 10.0}})
    res = r.get("result") or {}
    fs = res.get("first_step") or {}
    ok = ('error' not in r) and ('seed_choice' in fs) and (
        'seed_surface' in fs) and res.get("steps") is not None
    print("smoke arm_pose_ik_live:", "PASS" if ok else "FAIL")
    print("  error", r.get("error"))
    print("  seed_choice", fs.get("seed_choice"),
          "seed_surface", fs.get("seed_surface"))
    print("  steps", res.get("steps"), "t0/t1", res.get("t0"),
          res.get("t1"))


if __name__ == "__main__":
    main()
