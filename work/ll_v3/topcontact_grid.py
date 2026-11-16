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
    for nm in ("arm_tuck", "base_standoff_servo_v3", "topcontact_probe"):
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
        for dx in (-0.06, -0.03, 0.0, 0.03):
            for dy in (-0.03, 0.0, 0.03):
                for z in (1.02, 1.05, 1.08, 1.12):
                    for roll in (0, 90):
                        r = qtool({"tool": "call",
                                   "name": "topcontact_probe",
                                   "kwargs": {"side": side, "dx": dx,
                                              "dy": dy, "z": z,
                                              "roll": roll}})
                        rows.append(r.get("result") or {})
    feas = []
    for row in rows:
        ps = row.get("pad_surface_dist")
        w = row.get("wrist_surface")
        m = row.get("margin")
        if ps is None or w is None or m is None:
            continue
        if ps <= 0.005 and w >= 0.02 and m >= 0.05:
            feas.append(row)
    feas.sort(key=lambda r: (r["pad_surface_dist"], -r["margin"]))
    print("cells", len(rows), "feasible", len(feas))
    for r in feas[:5]:
        print("  FEAS %s dx %.2f dy %+.2f z %.2f roll %d pad_surf %.4f wrist %.3f margin %.3f joint %s normal %s faces %s"
              % (r["side"], r["dx"], r["dy"], r["z"], r["roll"],
                 r["pad_surface_dist"], r["wrist_surface"], r["margin"],
                 r["joint"], [round(x, 2) for x in r["pad_normal"]],
                 r.get("faces")))
    best = []
    for row in rows:
        ps = row.get("pad_surface_dist")
        if ps is None:
            continue
        best.append((ps, row))
    best.sort(key=lambda t: t[0])
    print("closest pad surfaces:")
    for ps, r in best[:5]:
        print("  %s dx %.2f dy %+.2f z %.2f roll %d pad_surf %.4f wrist %s margin %s normal %s"
              % (r["side"], r["dx"], r["dy"], r["z"], r["roll"], ps,
                 None if r.get("wrist_surface") is None
                 else round(r["wrist_surface"], 3),
                 None if r.get("margin") is None else round(r["margin"], 3),
                 [round(x, 2) for x in (r.get("pad_normal") or [])]))
    json.dump(rows, open(os.path.join(HERE, "reports", "w1_raw",
                                      "topcontact_grid.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
