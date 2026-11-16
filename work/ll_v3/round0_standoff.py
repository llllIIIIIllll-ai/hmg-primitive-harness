#!/usr/bin/env python3
"""Round 0 deliverable 4: standoff envelope per task."""
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
FINE = {"05_drill_lift": [0.30, 0.28, 0.26, 0.24, 0.23, 0.22, 0.20, 0.18]}
COARSE = [0.45, 0.40, 0.35, 0.30, 0.25, 0.20]


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
            txt = open(log, encoding="utf-8", errors="ignore").read()
            if "server ready" in txt:
                return
        time.sleep(2)
    raise RuntimeError("server not ready: %s (%s)" % (task, log))


def summarize(rec):
    best = None
    for m in rec["measurements"]:
        pe = m["pad_error"]
        worst = max(pe.values()) if pe else None
        margin = (m["reach"] or {}).get("joint_margin_min")
        ok = (worst is not None and worst <= 0.015
              and margin is not None and margin >= 0.05)
        row = {"target_standoff": m["target_standoff"],
               "achieved_standoff": round(m["achieved_standoff"], 3),
               "worst_pad_error": None if worst is None else round(worst, 4),
               "joint_margin_min": margin,
               "meets_0p015_and_margin": bool(ok),
               "base_yaw_deg": m["base"]["yaw_deg"],
               "base_tilt_deg": m["base"]["tilt_deg"],
               "binding_constraint": (
                   "pad_error" if (worst is not None and worst > 0.015)
                   else "joint_margin" if (margin is not None and margin < 0.05)
                   else "ok")}
        if ok:
            if best is None or row["achieved_standoff"] < best["achieved_standoff"]:
                best = row
    return {"best_reachable": best, "rows": [
        {"target_standoff": m["target_standoff"],
         "achieved_standoff": round(m["achieved_standoff"], 3),
         "worst_pad_error": None if not m["pad_error"] else round(
             max(m["pad_error"].values()), 4),
         "joint_margin_min": (m["reach"] or {}).get("joint_margin_min")}
        for m in rec["measurements"]]}


def main():
    tasks = sys.argv[1:]
    if not tasks:
        raise SystemExit("usage: round0_standoff.py <task> [task ...]")
    os.makedirs(RAW, exist_ok=True)
    out = {}
    for task in tasks:
        ensure_server(task)
        qtool(task, {"tool": "write", "name": "calib_standoff",
                     "source": SRC})
        qtool(task, {"tool": "reset", "seed": 0})
        grid = FINE.get(task, COARSE)
        r = qtool(task, {"tool": "call", "name": "calib_standoff",
                       "kwargs": {"standoffs": grid, "max_bursts": 50,
                                  "burst": 0.5}})
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
