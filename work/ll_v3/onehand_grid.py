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


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3",
               "onehand_grid_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    r = qtool({"tool": "call", "name": "base_standoff_servo_v3",
               "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                          "object_match": "bottle"}})
    print("standoff", (r.get("result") or {}).get("standoff_aabb"))
    rows = []
    for side in ("left", "right"):
        for dx in (-0.12, -0.08, -0.04):
            for dy in (-0.02, 0.0, 0.02):
                for z in (0.88, 0.92, 0.96):
                    for orient in ("front", "down", "inward"):
                        r = qtool({"tool": "call",
                                   "name": "onehand_grid_probe",
                                   "kwargs": {"side": side, "dx": dx,
                                              "dy": dy, "z": z,
                                              "orient": orient}})
                        res = r.get("result") or {}
                        rows.append(res)
    feas = []
    for row in rows:
        ik = row.get("ik_err")
        m = row.get("joint_margin_min")
        w = row.get("wrist_surface")
        if ik is None or m is None or w is None:
            continue
        if ik <= 0.02 and m >= 0.05 and w >= 0.02:
            feas.append(row)
    feas.sort(key=lambda r: -r["joint_margin_min"])
    print("cells", len(rows), "feasible", len(feas))
    for r in feas[:5]:
        print("  FEAS %s dx %.2f dy %+.2f z %.2f %s ik %.4f margin %.3f joint %s wrist %.3f"
              % (r["side"], r["dx"], r["dy"], r["z"], r["orient"],
                 r["ik_err"], r["joint_margin_min"],
                 r["joint_margin_joint"], r["wrist_surface"]))
    best = []
    for row in rows:
        ik = row.get("ik_err")
        m = row.get("joint_margin_min")
        if ik is None or m is None:
            continue
        best.append((ik, -m, row))
    best.sort(key=lambda t: (t[0], t[1]))
    print("best by residual:")
    for ik, nm, r in best[:5]:
        print("  %s dx %.2f dy %+.2f z %.2f %s ik %.4f margin %.3f joint %s wrist %s"
              % (r["side"], r["dx"], r["dy"], r["z"], r["orient"], ik,
                 r["joint_margin_min"], r["joint_margin_joint"],
                 None if r.get("wrist_surface") is None
                 else round(r["wrist_surface"], 3)))
    json.dump(rows, open(os.path.join(HERE, "reports", "w1_raw",
                                      "onehand_grid.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
