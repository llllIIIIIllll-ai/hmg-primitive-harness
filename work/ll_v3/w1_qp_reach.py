#!/usr/bin/env python3
"""Re-measure the 05 reach envelope with the executed mink-QP path.

Reduced grid (10 cells: 5 z values per side at xo=0, gap=0.155) at the 05
standoff placed by the base_standoff_servo.  The full 20-cell-per-side DLS
preview remains the screen; this executed set is the authoritative check.
"""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "w1_raw")
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
TASK = "05_drill_lift"


def qtool(req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def save(name, obj):
    os.makedirs(RAW, exist_ok=True)
    with open(os.path.join(RAW, name + ".json"), "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1)


def main():
    sub = os.path.join(HERE, "substrate")
    for name, f in (("base_standoff_servo", "base_standoff_servo.py"),
                    ("arm_pose_ik", "arm_pose_ik.py")):
        src = open(os.path.join(sub, f), encoding="utf-8").read()
        qtool({"tool": "write", "name": name, "source": src})
    qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    rows = []
    for side, sign in (("left", 1.0), ("right", -1.0)):
        for z in (0.46, 0.52, 0.58, 0.64, 0.70):
            qtool({"tool": "reset", "seed": 0})
            sg = qtool({"tool": "call", "name": "base_standoff_servo",
                        "kwargs": {"standoff": 0.15}})
            st = qtool({"tool": "probe", "what": "state"})["result"]
            obj = st["object"]["pos"]
            tgt = [obj[0], obj[1] + sign * 0.155, obj[2] + z]
            r = qtool({"tool": "qp_reach",
                       "kwargs": {"task_dim": 12, "sides": [side],
                                  "targets": {side: tgt},
                                  "orientation": "down", "dur": 1.0,
                                  "iters": 80}})
            res = r.get("result") or {}
            pr = (res.get("palm_residual") or {}).get(side)
            err = None
            if pr is not None:
                err = (pr[0] ** 2 + pr[1] ** 2 + pr[2] ** 2) ** 0.5
            rows.append({"side": side, "xo": 0.0, "gap": 0.155, "z": z,
                         "target": tgt, "pad_error": err,
                         "solver": res.get("solver"),
                         "joint_margin_min": res.get("joint_margin_min"),
                         "waist_owner": res.get("waist_owner"),
                         "standoff_ok": (sg.get("result") or {}).get("ok"),
                         "standoff_aabb": (sg.get("result") or {}).get(
                             "standoff_aabb")})
            print(side, z, "err", None if err is None else round(err, 4),
                  "margin", res.get("joint_margin_min"),
                  "solver", res.get("solver"))
    ok = [x for x in rows if x["pad_error"] is not None
          and x["pad_error"] <= 0.015]
    out = {"task": TASK, "pose_standoff_aabb": 0.15,
           "method": "executed mink-QP (qp_reach, task_dim=12, dur=1.0)",
           "rows": rows, "n": len(rows), "n_le_0p015": len(ok),
           "note": "The DLS preview reported 0/20 <= 15 mm; this executed "
                   "set is the authoritative reach check."}
    save("qp_reach_envelope_dur1p0", out)
    print("executed cells", len(rows), "<=15mm", len(ok))


if __name__ == "__main__":
    main()
