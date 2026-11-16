#!/usr/bin/env python3
"""W2d driver: phase-resolved trace and lateral-offset approach trials."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "w1_raw")
SUB = os.path.join(HERE, "substrate")
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")
TASK = "05_drill_lift"
OFFSETS = [0.0, 0.12, -0.12]
SRCS = {
    "base_standoff_servo_v3": "base_standoff_servo_v3.py",
    "arm_pose_ik_live": "arm_pose_ik_live.py",
    "arm_tuck": "arm_tuck.py",
    "hand_close_contact": "hand_close_contact.py",
    "safety_guard": "safety_guard.py",
    "w2d_reach_close_lift": "w2d_reach_close_lift.py",
}


def qtool(req, timeout=600):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def ensure_server():
    env = os.environ.copy()
    env["HMG_VENV"] = "/home/pjy0422/hmg_migration/stack/hmg_venv"
    env["HMG_REPO"] = "/home/pjy0422/hmg_migration/stack/humanoidmimicgen_repo"
    subprocess.run(["bash", CTL, "start", TASK, "0", "0"], cwd=ROOT, env=env,
                   capture_output=True, text=True)
    for _ in range(60):
        logs = sorted([f for f in os.listdir(os.path.join(ROOT, ".scratch"))
                       if f.startswith("srv_%s_" % TASK)], reverse=True)
        if logs:
            p = os.path.join(ROOT, ".scratch", logs[0])
            if "server ready" in open(p, encoding="utf-8",
                                      errors="ignore").read():
                return
        import time
        time.sleep(2)
    raise RuntimeError("server not ready")


def main():
    ensure_server()
    for name, f in SRCS.items():
        src = open(os.path.join(SUB, f), encoding="utf-8").read()
        qtool({"tool": "write", "name": name, "source": src})
    for off in OFFSETS:
        qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
        qtool({"tool": "reset", "seed": 0})
        r = qtool({"tool": "call", "name": "w2d_reach_close_lift",
                   "kwargs": {"standoff": 0.15, "lift_dz": 0.10,
                              "lateral_offset": off}})
        res = r.get("result") or {}
        with open(os.path.join(RAW, "w2d_offset_%+.2f.json" % off), "w",
                  encoding="utf-8") as fh:
            json.dump(r, fh, indent=1)
        ph = res.get("phase") or {}
        print("offset", off, "ok", res.get("ok"), "stage", res.get("stage"),
              "binding", res.get("binding_constraint"))
        print("  displacement", ph.get("displacements"))
        print("  pad", res.get("pad_error"), "normal", res.get(
            "normal_error_deg"), "contacts", res.get("contacts"),
              "rise", res.get("object_rise"),
              "guard", (res.get("guard") or {}).get("ok"))
        tr = res.get("standoff_trace") or []
        hits = []
        for e in tr:
            cs = e.get("contacts") or []
            if cs:
                hits.append({"phase": e.get("phase"), "burst": e.get(
                    "burst"), "t": e.get("t"), "object": e.get("object"),
                    "contacts": [{"b1": c.get("b1"), "b2": c.get("b2"),
                                  "g1": c.get("g1"), "g2": c.get("g2"),
                                  "dist": c.get("dist")} for c in cs]})
        print("  servo contact bursts", len(hits))
        for h in hits[:5]:
            print("   ", h)
        if res.get("ok"):
            print("W2d PASS")
            break
    print("w2d driver done")


if __name__ == "__main__":
    main()
