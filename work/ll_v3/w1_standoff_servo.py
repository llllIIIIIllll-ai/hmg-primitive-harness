#!/usr/bin/env python3
"""Re-run the per-task standoff envelope with the W1 base_standoff_servo.

For each task: servo to 0.35/0.25/0.20 m AABB standoff (task height/via where
needed), then arm_pose_ik with the explicit task target; record the standoff,
pad error, joint margin and binding constraint.
"""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "w1_raw")
SUB = os.path.join(HERE, "substrate")
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
TASKS = ["01_box_lift_floor", "02_push_button", "03_box_lift",
         "04_push_shelf_forward", "06_drill_pnp", "07_box_table_to_shelf",
         "08_pick_drill_from_holder", "09_obstacle_aware_pick_drill"]
GRID = [0.35, 0.25, 0.20]


def qtool(task, req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = task
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def task_cfg(task, st, objprobe, button):
    o = st["object"]["pos"]
    a = objprobe.get("aabb_lo"), objprobe.get("aabb_hi")
    if task == "01_box_lift_floor":
        return {"height": 0.55, "via": None,
                "targets": {"left": [o[0], o[1] + 0.155, 0.3275],
                            "right": [o[0], o[1] - 0.155, 0.3275]}}
    if task == "02_push_button":
        b = button or [o[0], o[1], 0.8455]
        return {"height": None, "via": None,
                "targets": {"right": [b[0] - 0.0838, b[1], b[2]]}}
    if task == "03_box_lift":
        return {"height": None, "via": None,
                "targets": {"left": [o[0], o[1] + 0.155, 0.8838],
                            "right": [o[0], o[1] - 0.155, 0.8838]}}
    if task == "04_push_shelf_forward":
        lo = a[0]
        return {"height": None, "via": None,
                "targets": {"left": [lo[0] - 0.0838, o[1] + 0.20, 0.7294],
                            "right": [lo[0] - 0.0838, o[1] - 0.20, 0.7294]}}
    if task == "06_drill_pnp":
        return {"height": None, "via": None,
                "targets": {"left": [o[0], o[1] + 0.155, 0.94],
                            "right": [o[0], o[1] - 0.155, 0.94]}}
    if task == "07_box_table_to_shelf":
        return {"height": None, "via": None,
                "targets": {"left": [o[0], o[1] + 0.155, 0.76],
                            "right": [o[0], o[1] - 0.155, 0.76]}}
    if task == "08_pick_drill_from_holder":
        return {"height": None, "via": None,
                "targets": {"left": [o[0], o[1] + 0.07, 0.8966],
                            "right": [o[0], o[1] - 0.07, 0.8966]}}
    if task == "09_obstacle_aware_pick_drill":
        return {"height": None, "via": [[0.5, -0.9]],
                "targets": {"left": [o[0], o[1] + 0.155, 0.94],
                            "right": [o[0], o[1] - 0.155, 0.94]}}
    return {}


def main():
    os.makedirs(RAW, exist_ok=True)
    out = {}
    for task in TASKS:
        for name, f in (("base_standoff_servo", "base_standoff_servo.py"),
                        ("arm_pose_ik", "arm_pose_ik.py")):
            src = open(os.path.join(SUB, f), encoding="utf-8").read()
            qtool(task, {"tool": "write", "name": name, "source": src})
        qtool(task, {"tool": "authority", "kwargs": {"waist_owner": "homie"}})
        rows = []
        for so in GRID:
            qtool(task, {"tool": "reset", "seed": 0})
            kw = {"standoff": so}
            st = qtool(task, {"tool": "probe", "what": "state"})["result"]
            objp = qtool(task, {"tool": "probe", "what": "object"})["result"]
            button = None
            if task == "02_push_button":
                bb = qtool(task, {"tool": "probe", "what": "bodies",
                                  "match": "button"})["result"]
                for b in bb.get("bodies") or []:
                    if b["body"] == "control_panel_button":
                        button = b["pos"]
            cfg = task_cfg(task, st, objp.get("object") or {}, button)
            for k in ("height", "via"):
                if cfg.get(k) is not None:
                    kw[k] = cfg[k]
            sg = qtool(task, {"tool": "call", "name": "base_standoff_servo",
                              "kwargs": kw})
            sr = sg.get("result") or {}
            row = {"target_standoff": so,
                   "servo_ok": sr.get("ok"),
                   "standoff_aabb": sr.get("standoff_aabb"),
                   "xy_error": sr.get("final_xy_error"),
                   "tilt_deg": sr.get("tilt_deg"),
                   "servo_binding": sr.get("binding_constraint")}
            if sr.get("ok"):
                ar = qtool(task, {"tool": "call", "name": "arm_pose_ik",
                                  "kwargs": {"targets": cfg["targets"],
                                             "orientation": "keep"}})
                aa = ar.get("result") or {}
                row.update({
                    "arm_ok": aa.get("ok"),
                    "pad_error": aa.get("pad_error"),
                    "worst_pad_error": aa.get("worst_pad_error"),
                    "joint_margin_min": aa.get("joint_margin_min"),
                    "arm_binding": aa.get("binding_constraint"),
                })
            rows.append(row)
            print(task, so, "servo", row["servo_ok"],
                  "arm", row.get("arm_ok"), "pad",
                  None if row.get("worst_pad_error") is None else
                  round(row["worst_pad_error"], 4),
                  "binding", row.get("arm_binding") or row["servo_binding"])
        out[task] = rows
        with open(os.path.join(RAW, "standoff_servo_%s.json" % task), "w",
                  encoding="utf-8") as fh:
            json.dump({"task": task, "grid": GRID, "rows": rows}, fh, indent=1)
    with open(os.path.join(RAW, "standoff_servo_summary.json"), "w",
              encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print("per-task servo envelope done")


if __name__ == "__main__":
    main()
