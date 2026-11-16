#!/usr/bin/env python3
"""Round 0 deliverable 6: grasp_state_tracking proxy vs the benchmark outcome.

The exact IsGrasped sub-condition is not exposed (success_detail only has
'task'), so the benchmark outcome is the full 05_drill_lift task success.  Three
conditions separate the proxy from the task-success threshold:
  good   target on the drill, lift 0.15 m  -> grasp and task success
  low    target on the drill, lift 0.05 m  -> grasp but task fails (lift < 0.1)
  miss   target above the drill, lift 0.15  -> no grasp and task fails
"""
import glob
import json
import os
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "round0_raw")
SRC = open(os.path.join(HERE, "primitives", "calib_grasp_proxy.py"),
           encoding="utf-8").read()
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
TASK = "05_drill_lift"
CONDS = {
    "good": {"lift_dz": 0.15},
    "low": {"lift_dz": 0.05},
    "miss": {"lift_dz": 0.15, "grasp_z": 1.15},
}


def qtool(req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def summaries(since):
    out = {}
    for f in glob.glob(os.path.join(ROOT, ".coord",
                                    "episode_%s_*.json" % TASK)):
        if os.path.getmtime(f) < since:
            continue
        try:
            s = json.load(open(f, encoding="utf-8"))
        except Exception:
            continue
        if s.get("primitive") != "calib_grasp_proxy":
            continue
        out[(s.get("seed"), json.dumps(s.get("cfg"), sort_keys=True))] = s
    return out


def main():
    qtool({"tool": "write", "name": "calib_grasp_proxy", "source": SRC})
    t0 = time.time()
    runs = {}
    for cond, kw in CONDS.items():
        r = qtool({"tool": "run_seeds", "name": "calib_grasp_proxy",
                   "seeds": [0, 1, 2, 3], "kwargs": kw,
                   "results": "full"})
        runs[cond] = r
        with open(os.path.join(RAW, "grasp_proxy_%s.json" % cond), "w",
                  encoding="utf-8") as fh:
            json.dump(r, fh, indent=1)
        print(cond, [(e["seed"], e["ok"],
                      (e.get("result") or {}).get("proxy_grasped"),
                      None if not e.get("result") else round(
                          e["result"].get("object_rise", 0), 3))
                     for e in r["result"]["episodes"]])
    sums = summaries(t0)
    rows = []
    for cond, r in runs.items():
        for e in r["result"]["episodes"]:
            key = (e["seed"], json.dumps(CONDS[cond], sort_keys=True))
            s = sums.get(key, {})
            res = e.get("result") or {}
            rows.append({
                "condition": cond, "seed": e["seed"], "ok": e["ok"],
                "proxy_grasped": res.get("proxy_grasped"),
                "close_phase_bilateral": res.get("close_phase_bilateral"),
                "lift_phase_bilateral": res.get("lift_phase_bilateral"),
                "object_rise": res.get("object_rise"),
                "contacts_before": res.get("contacts_before"),
                "contacts_after": res.get("contacts_after"),
                "benchmark_task_success": s.get("success"),
                "benchmark_success_detail": s.get("success_detail"),
            })
    tp = sum(1 for x in rows if x["proxy_grasped"] and
             x["benchmark_task_success"])
    fp = sum(1 for x in rows if x["proxy_grasped"] and
             not x["benchmark_task_success"])
    fn = sum(1 for x in rows if not x["proxy_grasped"] and
             x["benchmark_task_success"])
    tn = sum(1 for x in rows if not x["proxy_grasped"] and
             not x["benchmark_task_success"])
    out = {"episodes": rows,
           "confusion": {"tp": tp, "fp": fp, "fn": fn, "tn": tn},
           "n": len(rows), "conditions": CONDS,
           "note": "benchmark outcome = full 05 task success; the exact "
                   "IsGrasped sub-condition is not exposed "
                   "(success_detail only has 'task')"}
    with open(os.path.join(RAW, "grasp_proxy_summary.json"), "w",
              encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print("confusion", out["confusion"], "n", out["n"])


if __name__ == "__main__":
    main()
