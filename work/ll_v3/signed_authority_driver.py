#!/usr/bin/env python3
"""Signed authority map: elbow / shoulder-yaw at the standoff tuck, both
directions, with the posture RESTORED before every probe."""
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
                  ("attenuation_probe", "attenuation_probe.py"),
                  ("pose_limits_probe", "pose_limits_probe.py"),
                  ("restore_contact_probe", "restore_contact_probe.py")):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, f)).read()})
    qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck", "kwargs": {}})
    r1 = qtool({"tool": "call", "name": "base_standoff_servo_v3",
                "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                           "object_match": "bottle"}})
    print("standoff", (r1.get("result") or {}).get("ok"))
    q0 = (qtool({"tool": "call", "name": "pose_limits_probe",
                 "kwargs": {}}).get("result") or {}).get("upper")
    jr = (qtool({"tool": "call", "name": "pose_limits_probe",
                 "kwargs": {}}).get("result") or {}).get("joints") or {}
    for nm in ("left_elbow_joint", "left_shoulder_yaw_joint",
               "left_shoulder_pitch_joint", "left_wrist_pitch_joint"):
        if nm in jr:
            print("start %-28s q %.3f [%.3f, %.3f]"
                  % (nm, jr[nm]["q"], jr[nm]["lo"], jr[nm]["hi"]))
    out = {}
    for joint in ("left_elbow_joint", "left_shoulder_yaw_joint",
                  "left_shoulder_pitch_joint", "left_wrist_pitch_joint"):
        out[joint] = {}
        for sign in (1.0, -1.0):
            for step in (0.01, 0.02, 0.04, 0.08):
                qtool({"tool": "call", "name": "restore_contact_probe",
                       "kwargs": {"q": q0, "steps": 50}})
                r = qtool({"tool": "call", "name": "attenuation_probe",
                           "kwargs": {"joint": joint, "step": step,
                                      "steps": 40, "sign": sign}})
                res = r.get("result") or {}
                if r.get("error"):
                    print(joint, sign, step, "ERR", r.get("error"))
                    continue
                out[joint][sign] = out[joint].get(sign) or []
                out[joint][sign].append({"step": step,
                                         "gain": res["gain_mean"],
                                         "q_start": res["q_start"],
                                         "q_end": res["q_end"]})
                print("%-27s sign %+.0f step %.2f gain %+.3f  q %.3f -> %.3f"
                      % (joint, sign, step, res["gain_mean"],
                         res["q_start"], res["q_end"]))
    json.dump(out, open(os.path.join(RAW, "signed_authority_standoff.json"),
                        "w"), indent=1)


if __name__ == "__main__":
    main()
