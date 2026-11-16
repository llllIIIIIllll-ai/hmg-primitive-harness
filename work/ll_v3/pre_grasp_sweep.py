#!/usr/bin/env python3
"""Read-only pre-grasp grid at z=0.94 from the tucked standoff pose."""
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
    for nm in ("arm_tuck", "base_standoff_servo_v3", "arm_pose_ik_live"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    r = qtool({"tool": "call", "name": "base_standoff_servo_v3",
               "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                          "object_match": "bottle"}})
    print("standoff", (r.get("result") or {}).get("ok"))
    rows = []
    for dx in (-0.16, -0.20, -0.24):
        for dy in (0.14, 0.18, 0.22):
            r = qtool({"tool": "call", "name": "arm_pose_ik_live",
                       "kwargs": {"grasp": {"dx": dx, "dy": dy, "z": 0.94},
                                  "max_steps": 1, "budget": False,
                                  "seed_mode": "dls", "wrist_guard": True,
                                  "wrist_guard_dist": 10.0}})
            res = r.get("result") or {}
            fs = res.get("first_step") or {}
            tr = res.get("trace") or []
            pred = tr[0].get("wrist_pred") if tr else None
            surf = fs.get("seed_surface") or {}
            ik = fs.get("ik_err") or {}
            print("dx %.2f dy %.2f | ik_err L %s R %s | surf U %s S %s | pred %s"
                  % (dx, dy, ik.get("left"), ik.get("right"),
                     surf.get("unseeded"), surf.get("seeded"), pred))
            rows.append({"dx": dx, "dy": dy, "ik_err": ik,
                         "seed_surface": surf, "wrist_pred": pred,
                         "seed_choice": fs.get("seed_choice")})
    json.dump(rows, open(os.path.join(HERE, "reports", "w1_raw",
                                      "pre_grasp_sweep.json"), "w"),
              indent=1)


if __name__ == "__main__":
    main()
