#!/usr/bin/env python3
"""debug_visual substrate: find a failure window in the recorded trace and
render the onset/end frames.  Never reads the success predicate.

Usage:
  python3 work/ll_v3/substrate/debug_visual.py --task 05_drill_lift \
      --signal obj_z --rule below --threshold 0.85 --pad 0.1
"""
import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")


def qtool(task, req, timeout=120):
    env = os.environ.copy()
    env["HMG_TASK"] = task
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 20)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--signal", default="obj_z")
    ap.add_argument("--rule", default="below")
    ap.add_argument("--threshold", type=float, default=None)
    ap.add_argument("--delta", type=float, default=None)
    ap.add_argument("--pad", type=float, default=0.1)
    ap.add_argument("--lookat", default="1.7,0.0,0.9")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    fw = qtool(args.task, {"tool": "failure_window",
                           "signal": args.signal, "rule": args.rule,
                           "threshold": args.threshold, "delta": args.delta,
                           "pad": args.pad})
    res = fw.get("result") or {}
    onset_t = res.get("onset_t")
    end_t = res.get("end_t")
    out = {"failure_window": res, "onset_step": res.get("onset_step"),
           "onset_t": onset_t, "end_step": res.get("end_step"),
           "end_t": end_t, "terminal": res.get("terminal"),
           "stills": None, "visual_finding": None}
    if onset_t is not None and end_t is not None:
        lookat = [float(x) for x in args.lookat.split(",")]
        st = qtool(args.task, {"tool": "stills", "t": [onset_t, end_t],
                               "lookat": lookat, "n": 1,
                               "prefix": "debug_visual"})
        out["stills"] = st.get("result")
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
