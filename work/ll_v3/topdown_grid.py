#!/usr/bin/env python3
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


def grid(tag):
    rows = []
    for z in (1.00, 1.03, 1.06, 1.10):
        for dy in (0.06, 0.08, 0.10):
            for roll in (0, -15, 15, -30, 30):
                r = qtool({"tool": "call", "name": "topdown_grid_probe",
                           "kwargs": {"z": z, "dy": dy, "roll": roll}})
                res = r.get("result") or {}
                rows.append({"z": z, "dy": dy, "roll": roll,
                             "ik": res.get("ik_err"),
                             "m": res.get("joint_margin_min"),
                             "j": res.get("joint_margin_joint"),
                             "base": res.get("base")})
    feas = []
    best = None
    for row in rows:
        ik = row["ik"] or {}
        l = ik.get("left")
        rr = ik.get("right")
        if l is None or rr is None:
            continue
        w = l if l > rr else rr
        if best is None or w < best[0]:
            best = (w, row)
        if l <= 0.02 and rr <= 0.02 and row["m"] is not None and row["m"] >= 0.05:
            feas.append((w, row))
    feas.sort(key=lambda t: t[0])
    print("GRID", tag, "cells", len(rows), "feasible", len(feas),
          "best_worst_ik", None if best is None else round(best[0], 4))
    for w, row in feas[:5]:
        print("  FEAS z %.2f dy %.2f roll %+d worst_ik %.4f margin %.3f joint %s base %s"
              % (row["z"], row["dy"], row["roll"], w, row["m"], row["j"],
                 [round(x, 3) for x in (row["base"] or [])]))
    return rows


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3", "move_base_probe",
               "topdown_grid_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    r = qtool({"tool": "call", "name": "base_standoff_servo_v3",
               "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                          "object_match": "bottle"}})
    print("standoff", (r.get("result") or {}).get("standoff_aabb"))
    a = grid("A_current_standoff")
    mv = qtool({"tool": "call", "name": "move_base_probe",
                "kwargs": {"x": -0.12, "dur": 7.0}})
    print("backoff moved", (mv.get("result") or {}).get("moved_xy"))
    b = grid("B_backed_off")
    json.dump({"A": a, "B": b}, open(os.path.join(HERE, "reports", "w1_raw",
                                                  "topdown_grid.json"), "w"),
              indent=1)


if __name__ == "__main__":
    main()
