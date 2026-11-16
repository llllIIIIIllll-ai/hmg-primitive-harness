#!/usr/bin/env python3
"""Round 0 deliverables 3 and 5 at the stable 05 drill pose.

3. Reach envelope per arm: ik_preview grid at the minimum reachable standoff.
5. IK dimension A/B: 12/14 read-only under homie; 12/17 read-only + executed
   under an authorized ik handover, then restore homie.
"""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "round0_raw")
SRC = open(os.path.join(HERE, "primitives", "calib_standoff.py"),
           encoding="utf-8").read()
SRC_RP = open(os.path.join(HERE, "primitives", "calib_reach_preview.py"),
              encoding="utf-8").read()
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
TASK = "05_drill_lift"


def qtool(req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def save(name, obj):
    os.makedirs(RAW, exist_ok=True)
    with open(os.path.join(RAW, name + ".json"), "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1)


def main():
    qtool({"tool": "write", "name": "calib_standoff", "source": SRC})
    qtool({"tool": "reset", "seed": 0})
    pose = qtool({"tool": "call", "name": "calib_standoff",
                  "kwargs": {"standoffs": [0.15], "gap": 0.155,
                             "z_off": 0.45, "x_off": 0.0,
                             "max_bursts": 50, "burst": 0.5}})
    save("reach_pose", pose)
    m = pose["result"]["measurements"][0]
    base = m["base"]
    obj = m["object"]
    print("pose standoff aabb", round(m["achieved_standoff_aabb"], 3),
          "root", round(m["achieved_standoff_root"], 3),
          "base", base["pos"], "yaw", base["yaw_deg"], "tilt",
          base["tilt_deg"], "object", obj)

    # ---- deliverable 3: reach envelope at this pose (read-only) ----
    qtool({"tool": "write", "name": "calib_reach_preview",
           "source": SRC_RP})
    rp = qtool({"tool": "call", "name": "calib_reach_preview",
                "kwargs": {"xs": [-0.04, 0.02], "gaps": [0.155, 0.175],
                           "zs": [0.46, 0.52, 0.58, 0.64, 0.70],
                           "orients": ["down"], "iters": 150}}, timeout=600)
    save("reach_envelope", rp)
    rows = rp["result"]["rows"]
    print("ik_preview rows", len(rows))

    # ---- deliverable 5: targets from the live object pose ----
    targets = {
        "left": [obj[0], obj[1] + 0.155, obj[2] + 0.45],
        "right": [obj[0], obj[1] - 0.155, obj[2] + 0.45],
    }
    save("ik_targets", targets)

    # homie: 12 and 14 only (17 is not authorized under homie)
    cmp_h = qtool({"tool": "ik_compare",
                   "kwargs": {"targets": targets,
                              "task_dim_candidates": [12, 14],
                              "iters": 80}}, timeout=600)
    save("ik_compare_homie", cmp_h)

    # executed 12-D under homie
    q12 = qtool({"tool": "qp_reach",
                 "kwargs": {"task_dim": 12, "dur": 1.0, "iters": 80}},
                timeout=300)
    save("qp_reach_12d_homie", q12)

    # authorized handover to ik
    a1 = qtool({"tool": "authority", "kwargs": {"waist_owner": "ik"}})
    save("authority_ik", a1)
    assert a1["result"]["waist_owner"] == "ik"
    cmp_i = qtool({"tool": "ik_compare",
                   "kwargs": {"targets": targets,
                              "task_dim_candidates": [12, 17],
                              "iters": 80}}, timeout=600)
    save("ik_compare_ik", cmp_i)
    q17 = qtool({"tool": "qp_reach",
                 "kwargs": {"task_dim": 17, "dur": 1.0, "iters": 80}},
                timeout=300)
    save("qp_reach_17d_ik", q17)

    # restore the default owner and verify
    a2 = qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    save("authority_homie_restore", a2)
    assert a2["result"]["waist_owner"] == "homie"
    q12b = qtool({"tool": "qp_reach",
                  "kwargs": {"task_dim": 12, "dur": 1.0, "iters": 80}},
                 timeout=300)
    save("qp_reach_12d_restore", q12b)
    st = qtool({"tool": "probe", "what": "state"})
    save("state_after_restore", st)
    print("handover restored", a2["result"]["waist_owner"],
          "tilt", st["result"].get("base_tilt_deg"))
    print("wrote reach/IK raw data")


if __name__ == "__main__":
    main()
