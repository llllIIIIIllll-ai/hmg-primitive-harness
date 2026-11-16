#!/usr/bin/env python3
"""Steady-state WBC gain (40-step window) at backoff and standoff."""
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


def setup(pos):
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    qtool({"tool": "call", "name": "base_standoff_servo_v3",
           "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                      "object_match": "bottle"}})
    if pos == "backoff":
        qtool({"tool": "call", "name": "move_base_probe",
               "kwargs": {"x": -0.12, "dur": 4.5}})


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3", "attenuation_probe",
               "move_base_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    cells = (("backoff", "left_shoulder_roll_joint"),
             ("backoff", "left_elbow_joint"),
             ("backoff", "left_wrist_pitch_joint"),
             ("standoff", "left_elbow_joint"),
             ("standoff", "left_wrist_pitch_joint"))
    out = []
    for pos in ("backoff", "standoff"):
        setup(pos)
        for pos2, joint in cells:
            if pos2 != pos:
                continue
            for sz in (0.02, 0.04):
                r = qtool({"tool": "call", "name": "attenuation_probe",
                           "kwargs": {"joint": joint, "step": sz,
                                      "steps": 40, "sign": 1.0}})
                res = r.get("result") or {}
                ds = res.get("deltas") or []
                qs = res.get("q_start")
                qe = res.get("q_end")
                total = None
                if qs is not None and qe is not None:
                    total = qe - qs
                g40 = None
                if total is not None:
                    g40 = total / (40.0 * sz)
                first = ds[0]["delta"] / sz if ds else None
                early = None
                late = None
                if ds:
                    s1 = 0.0
                    s2 = 0.0
                    i = 0
                    while i < 10 and i < len(ds):
                        s1 = s1 + ds[i]["delta"]
                        i = i + 1
                    j = len(ds) - 10
                    while j < len(ds):
                        s2 = s2 + ds[j]["delta"]
                        j = j + 1
                    early = s1 / (10.0 * sz)
                    late = s2 / (10.0 * sz)
                out.append({"pos": pos, "joint": joint, "step": sz,
                            "gain_40": g40, "gain_first": first,
                            "gain_first10": early,
                            "gain_last10": late, "q_start": qs,
                            "q_end": qe})
                print(pos, joint, sz, "g40", None if g40 is None else round(g40, 3),
                      "first", None if first is None else round(first, 3),
                      "f10", None if early is None else round(early, 3),
                      "l10", None if late is None else round(late, 3),
                      "q", qs, "->", qe)
    json.dump(out, open(os.path.join(HERE, "reports", "w1_raw",
                                     "steady_gain.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
