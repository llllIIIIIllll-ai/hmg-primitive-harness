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


def main():
    for nm in ("arm_tuck", "base_standoff_servo_v3", "arm_untuck",
               "arm_pose_ik_live", "attenuation_probe"):
        qtool({"tool": "write", "name": nm,
               "source": open(os.path.join(SUB, nm + ".py")).read()})
    qtool({"tool": "reset", "seed": 0})
    qtool({"tool": "call", "name": "arm_tuck"})
    qtool({"tool": "call", "name": "base_standoff_servo_v3",
           "kwargs": {"standoff": 0.15, "lateral_target": 0.03,
                      "object_match": "bottle"}})
    q = {"orient_mode": "down", "orientation": "down", "joint_mode": True,
         "deadband_comp": 0.012, "wrist_guard": True,
         "wrist_guard_dist": 0.01, "seed_mode": "dls",
         "sides": ("left",), "base_gain": 0.3, "base_deadband": 0.005,
         "base_vmax": 0.05}
    u = qtool({"tool": "call", "name": "arm_untuck",
               "kwargs": dict(q, dx=-0.126, dy=0.07, z=1.16,
                              stop_err=0.12, max_steps=250,
                              max_delta=0.04)})
    print("stage1", (u.get("result") or {}).get("worst_pad_error"))
    q2 = {"orient_mode": "side", "orientation": "keep", "joint_mode": True,
          "deadband_comp": 0.012, "wrist_guard": True,
          "wrist_guard_dist": 0.01, "seed_mode": "dls",
          "sides": ("left",), "base_gain": 0.3, "base_deadband": 0.005,
          "base_vmax": 0.05, "max_steps": 300, "max_delta": 0.04,
          "stop_err": 0.12}
    u2 = qtool({"tool": "call", "name": "arm_pose_ik_live",
                "kwargs": dict(q2, grasp={"dx": -0.205, "dy": 0.0,
                                          "z": 0.954})})
    print("stage2", (u2.get("result") or {}).get("worst_pad_error"))
    for joint in ("left_shoulder_pitch_joint", "left_elbow_joint",
                  "left_wrist_pitch_joint"):
        for step in (0.005, 0.01, 0.02, 0.04, 0.08):
            r = qtool({"tool": "call", "name": "attenuation_probe",
                       "kwargs": {"joint": joint, "step": step,
                                  "steps": 40, "sign": 1.0}})
            res = r.get("result") or {}
            ds = res.get("deltas") or []
            qs = res.get("q_start")
            qe = res.get("q_end")
            g = None
            if qs is not None and qe is not None:
                g = (qe - qs) / (40.0 * step)
            print("%-28s s%.3f steady_gain %s q %s -> %s"
                  % (joint, step, None if g is None else round(g, 3),
                     None if qs is None else round(qs, 3),
                     None if qe is None else round(qe, 3)))


if __name__ == "__main__":
    main()
