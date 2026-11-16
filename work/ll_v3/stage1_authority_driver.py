#!/usr/bin/env python3
"""Stage-1 authority: tuck -> standoff -> per-joint attenuation for the down target."""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "w1_raw")
SUB = os.path.join(HERE, "substrate")
TASK = "05_drill_lift"


def qtool(req, timeout=900):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([os.path.join(ROOT, "scripts", "qtool.sh"),
                        json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def main():
    for nm, f in (("arm_tuck", "arm_tuck.py"),
                  ("base_standoff_servo_v3", "base_standoff_servo_v3.py"),
                  ("arm_untuck", "arm_untuck.py"),
                  ("arm_pose_ik_live", "arm_pose_ik_live.py"),
                  ("attenuation_probe", "attenuation_probe.py"),
                  ("down_seed_probe", "down_seed_probe.py"),
                  ("pose_limits_probe", "pose_limits_probe.py")):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, f)).read()})
    qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck", "kwargs": {}})
    r1 = qtool({"tool": "call", "name": "base_standoff_servo_v3",
                "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                           "object_match": "bottle"}})
    print("standoff", (r1.get("result") or {}).get("ok"))
    s = qtool({"tool": "call", "name": "down_seed_probe",
               "kwargs": {"dx": -0.126, "dy": 0.07, "z": 1.16}})
    sres = s.get("result") or {}
    print("down seed target", sres.get("target"))
    for row in (sres.get("rows") or []):
        print("  seed dq", row.get("dq"), "err", row.get("err"))
        for nm, v in (row.get("delta") or {}).items():
            print("    %-30s now %7.3f seed %7.3f delta %7.3f"
                  % (nm, v["q_now"], v["q_seed"], v["delta"]))
    for joint in ("left_shoulder_pitch_joint", "left_shoulder_roll_joint",
                  "left_shoulder_yaw_joint", "left_elbow_joint",
                  "left_wrist_pitch_joint"):
        for step in (0.02, 0.04, 0.08):
            r = qtool({"tool": "call", "name": "attenuation_probe",
                       "kwargs": {"joint": joint, "step": step,
                                  "steps": 40, "sign": 1.0}})
            res = r.get("result") or {}
            if r.get("error"):
                print(joint, step, "ERR", r.get("error"))
                continue
            print("%-28s step %.2f gain %.2f q %.3f -> %.3f"
                  % (joint, step, res["gain_mean"], res["q_start"],
                     res["q_end"]))
    print("(post-probe arm is displaced; stage-1 itself not run)")


if __name__ == "__main__":
    main()
