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
    for nm in ("arm_tuck", "base_standoff_servo_v3", "handle_grid_probe"):
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
        for frac in (0.2, 0.5, 0.8):
            for roll in (0, 90):
                for zoff in (0.05, 0.07, 0.09):
                    r = qtool({"tool": "call", "name": "handle_grid_probe",
                               "kwargs": {"side": side, "frac": frac,
                                          "roll": roll, "zoff": zoff}})
                    rows.append(r.get("result") or {})
    feas = []
    for row in rows:
        ps = row.get("pad_surface")
        w = row.get("wrist_min")
        m = row.get("margin")
        ik = row.get("ik")
        if ps is None or w is None or m is None or ik is None:
            continue
        if ps <= 0.005 and w >= 0.02 and m >= 0.05 and ik <= 0.02:
            feas.append(row)
    feas.sort(key=lambda r: (-r["margin"], r["pad_surface"]))
    print("cells", len(rows), "feasible", len(feas))
    for r in feas[:5]:
        print("  FEAS %s frac %.1f roll %d pad_surf %.4f wrist %.3f margin %.3f joint %s ik %.4f normal %s"
              % (r["side"], r["frac"], r["roll"], r["pad_surface"],
                 r["wrist_min"], r["margin"], r["joint"], r["ik"],
                 [round(x, 2) for x in (r.get("pad_normal") or [])]))
    best = [r for r in rows if r.get("pad_surface") is not None]
    best.sort(key=lambda r: r["pad_surface"])
    print("closest pad surfaces:")
    for r in best[:5]:
        print("  %s frac %.1f roll %d pad_surf %.4f wrist %s margin %s ik %s"
              % (r["side"], r["frac"], r["roll"], r["pad_surface"],
                 None if r.get("wrist_min") is None
                 else round(r["wrist_min"], 3),
                 None if r.get("margin") is None else round(r["margin"], 3),
                 None if r.get("ik") is None else round(r["ik"], 3)))
    json.dump(rows, open(os.path.join(HERE, "reports", "w1_raw",
                                      "handle_grid.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
