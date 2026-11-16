#!/usr/bin/env python3
"""W1 smoke chain: register the substrates and run a basic live sequence."""
import json
import os
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
RAW = os.path.join(HERE, "reports", "w1_raw")
TASK = "05_drill_lift"
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")
SRCS = {
    "base_standoff_servo": "base_standoff_servo.py",
    "arm_pose_ik": "arm_pose_ik.py",
    "hand_close_contact": "hand_close_contact.py",
    "hand_open": "hand_open.py",
    "object_pose_track": "object_pose_track.py",
    "safety_guard": "safety_guard.py",
    "authority_handover": "authority_handover.py",
    "w2_reach_close_lift": "w2_reach_close_lift.py",
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
        time.sleep(2)
    raise RuntimeError("server not ready")


def save(name, obj):
    os.makedirs(RAW, exist_ok=True)
    with open(os.path.join(RAW, name + ".json"), "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1)


def run(name, kwargs=None):
    r = qtool({"tool": "call", "name": name,
               "kwargs": kwargs or {}})
    save(name, r)
    res = r.get("result") or {}
    print(name, "ok", r.get("ok"), "inner_ok", res.get("ok"),
          "binding", res.get("binding_constraint"))
    return r


def main():
    ensure_server()
    for name, f in SRCS.items():
        src = open(os.path.join(SUB, f), encoding="utf-8").read()
        w = qtool({"tool": "write", "name": name, "source": src})
        save("register_" + name, w)
        print("registered", name)
    qtool({"tool": "reset", "seed": 0})
    run("base_standoff_servo", {"standoff": 0.15})
    run("arm_pose_ik", {})
    run("hand_close_contact", {})
    run("safety_guard", {})
    run("object_pose_track", {})
    run("object_pose_track", {})
    run("hand_open", {})
    run("authority_handover", {})
    qtool({"tool": "reset", "seed": 0})
    run("w2_reach_close_lift", {"standoff": 0.15})
    print("smoke done")


if __name__ == "__main__":
    main()
