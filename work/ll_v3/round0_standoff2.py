#!/usr/bin/env python3
"""Round 0 deliverable 4 (AABB standoff) for the tasks with a stable approach.

The 05 drill approach is measured; 01 (floor crouch) and 04 (16 kg shelf) are
recorded as failures: the hand-rolled move_base controller tilts 20-33 deg and
runs away, so the per-task standoff envelope needs the W1 base-standoff servo.
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "round0_raw")
SRC = open(os.path.join(HERE, "primitives", "calib_standoff.py"),
           encoding="utf-8").read()
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")

CONFIG = {
    "02_push_button": {"z_off": 0.0071, "gap": 0.155, "x_off": 0.0,
                       "standoffs": [0.35, 0.30, 0.25, 0.20, 0.15]},
    "03_box_lift": {"z_off": 0.0842, "gap": 0.155, "x_off": 0.0,
                    "standoffs": [0.35, 0.30, 0.25, 0.20, 0.15]},
    "05_drill_lift": {"z_off": 0.45, "gap": 0.155, "x_off": 0.0,
                      "standoffs": [0.35, 0.30, 0.25, 0.20, 0.15, 0.12]},
    "06_drill_pnp": {"z_off": 0.45, "gap": 0.155, "x_off": 0.0,
                     "standoffs": [0.35, 0.30, 0.25, 0.20, 0.15]},
    "07_box_table_to_shelf": {"z_off": 0.075, "gap": 0.155, "x_off": 0.0,
                              "standoffs": [0.35, 0.30, 0.25, 0.20, 0.15]},
    "08_pick_drill_from_holder": {"z_off": 0.1692, "gap": 0.155, "x_off": 0.0,
                                  "standoffs": [0.35, 0.30, 0.25, 0.20, 0.15]},
    "09_obstacle_aware_pick_drill": {"z_off": 0.45, "gap": 0.155, "x_off": 0.0,
                                     "standoffs": [0.35, 0.30, 0.25, 0.20, 0.15]},
}


def qtool(task, req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = task
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def ensure_server(task):
    env = os.environ.copy()
    env["HMG_VENV"] = "/home/pjy0422/hmg_migration/stack/hmg_venv"
    env["HMG_REPO"] = "/home/pjy0422/hmg_migration/stack/humanoidmimicgen_repo"
    subprocess.run(["bash", CTL, "start", task, "0", "0"], cwd=ROOT, env=env,
                   capture_output=True, text=True)
    log = None
    for _ in range(60):
        logs = sorted([f for f in os.listdir(os.path.join(ROOT, ".scratch"))
                       if f.startswith("srv_%s_" % task)], reverse=True)
        if logs:
            log = os.path.join(ROOT, ".scratch", logs[0])
            if "server ready" in open(log, encoding="utf-8",
                                      errors="ignore").read():
                return
        time.sleep(2)
    raise RuntimeError("server not ready: %s (%s)" % (task, log))


def summarize(rec):
    best = None
    rows = []
    for m in rec["measurements"]:
        pe = m["pad_error"]
        worst = max(pe.values()) if pe else None
        margin = (m["reach"] or {}).get("joint_margin_min")
        tilt = m["base"]["tilt_deg"]
        ok = (worst is not None and worst <= 0.015
              and margin is not None and margin >= 0.05
              and abs(tilt) <= 8.0)
        row = {"target_standoff_aabb": m["target_standoff"],
               "achieved_standoff_aabb": round(
                   m["achieved_standoff_aabb"], 3),
               "achieved_standoff_root": round(
                   m["achieved_standoff_root"], 3),
               "worst_pad_error": None if worst is None else round(worst, 4),
               "joint_margin_min": margin,
               "base_tilt_deg": tilt,
               "meets_0p015_margin_tilt": bool(ok),
               "binding_constraint": (
                   "tilt_guard" if abs(tilt) > 8.0
                   else "pad_error" if (worst is not None and worst > 0.015)
                   else "joint_margin" if (margin is not None and margin < 0.05)
                   else "ok")}
        rows.append(row)
        if ok:
            if best is None or row["achieved_standoff_aabb"] < \
                    best["achieved_standoff_aabb"]:
                best = row
    return {"target_params": rec.get("height"), "best_reachable": best,
            "rows": rows}


def main():
    tasks = sys.argv[1:]
    if not tasks:
        raise SystemExit("usage: round0_standoff2.py <task> [task ...]")
    os.makedirs(RAW, exist_ok=True)
    out = {}
    for task in tasks:
        cfg = CONFIG[task]
        ensure_server(task)
        qtool(task, {"tool": "write", "name": "calib_standoff",
                     "source": SRC})
        qtool(task, {"tool": "reset", "seed": 0})
        r = qtool(task, {"tool": "call", "name": "calib_standoff",
                         "kwargs": {"standoffs": cfg["standoffs"],
                                    "gap": cfg["gap"],
                                    "z_off": cfg["z_off"],
                                    "x_off": cfg["x_off"],
                                    "max_bursts": 50, "burst": 0.5}})
        with open(os.path.join(RAW, "standoff_%s.json" % task), "w",
                  encoding="utf-8") as fh:
            json.dump(r, fh, indent=1)
        summ = summarize(r["result"])
        out[task] = summ
        print(task, "best", summ["best_reachable"])
        for row in summ["rows"]:
            print("   ", row)
    with open(os.path.join(RAW, "standoff_summary.json"), "w",
              encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)


if __name__ == "__main__":
    main()
