#!/usr/bin/env python3
"""Probe: does a slower untuck keep the WBC base from walking away?"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
TASK = "05_drill_lift"
SRCS = {"arm_tuck": "arm_tuck.py",
        "base_standoff_servo_v3": "base_standoff_servo_v3.py",
        "arm_untuck": "arm_untuck.py",
        "arm_pose_ik_live": "arm_pose_ik_live.py"}


def qtool(req, timeout=900):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)], cwd=ROOT,
                       env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def main():
    max_delta = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03
    max_steps = int(sys.argv[2]) if len(sys.argv) > 2 else 120
    dx = float(sys.argv[3]) if len(sys.argv) > 3 else -0.16
    dy = float(sys.argv[4]) if len(sys.argv) > 4 else 0.11
    z = float(sys.argv[5]) if len(sys.argv) > 5 else 0.94
    for name, f in SRCS.items():
        qtool({"tool": "write", "name": name,
               "source": open(os.path.join(SUB, f), encoding="utf-8").read()})
    qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    qtool({"tool": "reset", "seed": 0})
    print("tuck", qtool({"tool": "call", "name": "arm_tuck"})
          .get("result", {}).get("ok"))
    r = qtool({"tool": "call", "name": "base_standoff_servo_v3",
               "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                          "object_match": "drill"}})
    print("standoff", (r.get("result") or {}).get("ok"))
    r = qtool({"tool": "call", "name": "arm_untuck",
               "kwargs": {"dx": dx, "dy": dy, "z": z,
                          "stop_err": 0.015, "max_steps": max_steps,
                          "max_delta": max_delta}})
    u = r.get("result") or {}
    print("untuck ok", u.get("ok"), "worst", u.get("worst_pad_error"),
          "steps", u.get("steps"), "resets", u.get("resets"),
          "nans", u.get("nans"), "sim_t", u.get("sim_time"),
          "margin", u.get("joint_margin_min"),
          "base_drift", u.get("base_drift"))
    print("pad_error", u.get("pad_error"))
    tr = u.get("trace") or []
    for e in tr[::10] + tr[-1:]:
        print("  k%03d t %.2f->%.2f worst %s basej %.4f nav %s" % (
            e.get("k", -1), e.get("t_before", 0), e.get("t_after", 0),
            ("%.4f" % e["worst"]) if e.get("worst") is not None else None,
            e.get("base_jump", 0), [round(x, 3) for x in (e.get("nav") or [])]))
    json.dump(r, open(os.path.join(HERE, "reports", "w1_raw",
                                   "w2e_probe_%.3f.json" % max_delta), "w"),
              indent=1)


if __name__ == "__main__":
    main()
