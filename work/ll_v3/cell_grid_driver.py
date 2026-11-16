#!/usr/bin/env python3
"""Stage-1 cell grid driver: tuck -> standoff -> stage-1 down -> read-only cell grid."""
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
OFFSETS = ([float(sys.argv[1])] if len(sys.argv) > 1
           else [0.0, 0.12])
SRCS = {
    "arm_tuck": "arm_tuck.py",
    "base_standoff_servo_v3": "base_standoff_servo_v3.py",
    "arm_untuck": "arm_untuck.py",
    "arm_pose_ik_live": "arm_pose_ik_live.py",
    "hand_close_contact": "hand_close_contact.py",
    "safety_guard": "safety_guard.py",
    "cell_grid_probe": "cell_grid_probe.py",
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
    import time
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


def main():
    ensure_server()
    for name, f in SRCS.items():
        src = open(os.path.join(SUB, f), encoding="utf-8").read()
        qtool({"tool": "write", "name": name, "source": src})
    qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck", "kwargs": {}})
    r1 = qtool({"tool": "call", "name": "base_standoff_servo_v3",
                "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                           "object_match": "bottle"}})
    print("standoff", (r1.get("result") or {}).get("ok"))
    u1 = qtool({"tool": "call", "name": "arm_untuck",
                "kwargs": {"dx": -0.126, "dy": 0.07, "z": 1.16,
                           "stop_err": 0.12, "max_steps": 250,
                           "max_delta": 0.04, "orient_mode": "down",
                           "orientation": "down", "joint_mode": True,
                           "deadband_comp": 0.012, "wrist_guard": True,
                           "wrist_guard_dist": 0.02, "seed_mode": "dls",
                           "sides": ("left",), "base_gain": 0.3,
                           "base_deadband": 0.005, "base_vmax": 0.05,
                           "base_yaw_deadband": 0.003, "base_yaw_gain": 0.6,
                           "base_wmax": 0.1, "trace_contacts": True,
                           "contact_match": "bottle"}})
    print("stage1 ok", (u1.get("result") or {}).get("ok"),
          "worst", (u1.get("result") or {}).get("worst_pad_error"))
    g = qtool({"tool": "call", "name": "cell_grid_probe",
               "kwargs": {"side": "left"}}, timeout=1800)
    res = g.get("result") or {}
    with open(os.path.join(RAW, "cell_grid_stage1.json"), "w",
              encoding="utf-8") as fh:
        json.dump(g, fh, indent=1)
    print("grid err", g.get("error"), "rows", len(res.get("rows") or []))
    for row in (res.get("rows") or []):
        b = row.get("best") or {}
        print("%-5s frac %.1f roll %3d | best %s err %s | margin %s @ %s | bounds %s | wrist %s @ %s | yaw %.3f" % (
            row.get("approach"), row.get("frac"), row.get("roll"),
            b.get("branch_name"),
            None if b.get("err") is None else round(b["err"], 4),
            b.get("margin"), b.get("margin_joint"),
            [x.replace("_joint", "") for x in (b.get("bounds") or [])],
            None if b.get("wrist_min") is None else round(b["wrist_min"], 4),
            (b.get("wrist_collider") or "").replace("collider_", ""),
            b.get("yaw_delta") or 0.0))


if __name__ == "__main__":
    main()
