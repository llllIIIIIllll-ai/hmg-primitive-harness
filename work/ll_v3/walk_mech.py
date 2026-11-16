#!/usr/bin/env python3
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
TASK = "05_drill_lift"


def qtool(req, timeout=900):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run(["./scripts/qtool.sh", json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def prep(pos):
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    qtool({"tool": "call", "name": "base_standoff_servo_v3",
           "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                      "object_match": "bottle"}})
    if pos == "backoff":
        qtool({"tool": "call", "name": "move_base_probe",
               "kwargs": {"x": -0.12, "dur": 7.0}})


def run_case(label, pos, dur=5.0):
    prep(pos)
    r = qtool({"tool": "call", "name": "walk_mech_probe",
               "kwargs": {"x": 0.12, "dur": dur, "settle": 1.5}})
    res = r.get("result") or {}
    tr = res.get("trace") or []
    print("==", label, "dist", None if res.get("dist") is None
          else round(res["dist"], 4))
    for e in tr[:4] + tr[-2:]:
        print("  k%03d pos %s tilt %s yaw %s vel %s nscene %d feet %s"
              % (e.get("k"), [round(x, 3) for x in (e.get("pos") or [])],
                 e.get("tilt"), e.get("yaw"),
                 [round(v, 3) for v in (e.get("vel") or [])],
                 e.get("nscene"), str(e.get("feet"))[:100]))
        for c in (e.get("scene") or [])[:2]:
            print("     scene", c)
    return res


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3", "move_base_probe",
               "walk_mech_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    run_case("standoff", "standoff")
    run_case("backoff", "backoff")


if __name__ == "__main__":
    main()
