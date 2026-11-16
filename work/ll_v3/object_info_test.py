#!/usr/bin/env python3
"""Validator for the read-only object_info tool: drill (05) and box (03)."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")


def qtool(task, req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = task
    p = subprocess.run(["./scripts/qtool.sh", json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def state(task):
    r = qtool(task, {"tool": "call", "name": "state_probe", "kwargs": {}})
    return r.get("result") or {}


def check(task):
    qtool(task, {"tool": "write", "name": "state_probe",
                 "source": open(os.path.join(SUB, "state_probe.py")).read()})
    qtool(task, {"tool": "reset", "seed": 0})
    s0 = state(task)
    r = qtool(task, {"tool": "object_info"})
    info = r if 'object' in r else (r.get('result') or {})
    s1 = state(task)
    same = (s0.get("qpos") == s1.get("qpos")
            and s0.get("qvel") == s1.get("qvel")
            and s0.get("t") == s1.get("t"))
    print("==", task, "error", r.get("error"), "state_unchanged", same)
    print("  object", info.get("object"), "summary", info.get("summary"))
    for p in (info.get("parts") or []):
        print("  part %-8s %-32s dims %s cross %s fits %s" % (
            p.get("part"), p.get("evidence_geom"), p.get("dims"),
            p.get("cross_section"), p.get("fits_hand_opening")))
    print("  grasp", info.get("candidate_grasp_parts"))
    print("  aabb_note", (info.get("aabb_note") or "")[:60])
    return info


def main():
    for task in ("05_drill_lift", "03_box_lift"):
        check(task)


if __name__ == "__main__":
    main()
