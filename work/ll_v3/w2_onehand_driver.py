#!/usr/bin/env python3
"""W2e driver: tuck -> standoff -> untuck -> true closed-loop reach -> close -> lift."""
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
    "arm_stage_reach": "arm_stage_reach.py",
    "w2_onehand": "w2_onehand.py",
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
    for off in OFFSETS:
        qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
        qtool({"tool": "reset", "seed": 0})
        r = qtool({"tool": "call", "name": "w2_onehand",
                   "kwargs": {"standoff": 0.15, "lift_dz": 0.10,
                              "lateral_offset": off,
                              "target_site": [1.62683, 0.02184, 0.88450],
                              "n_stages": 5}})
        res = r.get("result") or {}
        with open(os.path.join(RAW, "w2oh_offset_%+.2f.json" % off), "w",
                  encoding="utf-8") as fh:
            json.dump(r, fh, indent=1)
        print("offset", off, "ok", res.get("ok"), "stage", res.get("stage"),
              "binding", res.get("binding_constraint"), "err", r.get("error"))
        ph = res.get("phase") or {}
        print("  displacements", ph.get("displacements"))
        print("  resets/nans", ph.get("resets"), ph.get("nans"),
              "sim_time", ph.get("reach_sim_time"))
        print("  first_step", json.dumps(ph.get("first_step"))[:900])
        sr = res.get("stage_reach") or {}
        for st_ in (sr.get("stages") or []):
            print("  stage", st_.get("stage"), "frac", st_.get("frac"),
                  "ok", st_.get("ok"), "pe", st_.get("pad_error"),
                  "margin", st_.get("margin_min"), st_.get("margin_joint"),
                  "wrist", st_.get("wrist_dist_min"), "abort", st_.get("wrist_abort"))
        print("  site", res.get("pad_error"), "normal", res.get(
            "normal_error_deg"), "contacts", res.get("contacts"),
              "rise", res.get("object_rise"),
              "guard", (res.get("guard") or {}).get("ok"))
        print("  budget", json.dumps(res.get("budget"))[:900])
        tr = ph.get("reach_trace") or []
        print("  reach trace n", len(tr))
        for e in tr[:3] + tr[-3:]:
            print("   ", json.dumps(e)[:500])
        if res.get("ok"):
            print("W2e PASS")
            break
    print("w2e driver done")


if __name__ == "__main__":
    main()
