#!/usr/bin/env python3
"""Measure WBC drift with a still upper body, nav=0, and with the corrected
deadbanded base-hold law (2 s each), from the post-standoff pose."""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
TASK = "05_drill_lift"


def qtool(req, timeout=900):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)], cwd=ROOT,
                       env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def prepare():
    qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    r = qtool({"tool": "call", "name": "base_standoff_servo_v3",
               "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                          "object_match": "drill"}})
    return (r.get("result") or {}).get("ok")


def probe(hold, steps=50):
    r = qtool({"tool": "call", "name": "base_still_probe",
               "kwargs": {"steps": steps, "hold": hold}})
    res = r.get("result") or {}
    print("hold", hold, "drift", res.get("drift"), "dyaw",
          res.get("dyaw"), "b0", res.get("b0"), "b1", res.get("b1"))
    tr = res.get("trace") or []
    for e in tr[::10] + tr[-1:]:
        print("  k%02d t %.2f pos %s yaw %s nav %s" % (
            e.get("k", -1), e.get("t", 0),
            [round(x, 4) for x in e.get("pos", [])], e.get("yaw_deg"),
            [round(x, 4) for x in (e.get("nav") or [])]))
    return r


def main():
    for name, f in (("arm_tuck", "arm_tuck.py"),
                    ("base_standoff_servo_v3",
                     "base_standoff_servo_v3.py"),
                    ("base_still_probe", "base_still_probe.py")):
        qtool({"tool": "write", "name": name,
               "source": open(os.path.join(SUB, f), encoding="utf-8").read()})
    out = {}
    print("standoff", prepare())
    out["nav0"] = probe(False)
    print("standoff", prepare())
    out["hold"] = probe(True)
    json.dump(out, open(os.path.join(HERE, "reports", "w1_raw",
                                     "w2e_still_probe.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
