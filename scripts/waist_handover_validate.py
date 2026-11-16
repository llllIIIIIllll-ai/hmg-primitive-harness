#!/usr/bin/env python3
"""Validate the user-authorized waist handover.

1. handover + re-handover gains on seeds 0-3 (homie -> ik -> homie)
2. lower_owner policy <-> hold behaviour change
3. balance: 1.0 m walk and a 45 deg turn under homie vs ik (tilt, fall)
4. 17-D vs 12-D: qp_reach with the waist handed to IK vs HOMIE-owned
"""
import json
import os
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
SERVER_CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")
VENV = "/home/pjy0422/hmg_migration/stack/hmg_venv"
TASK = "05_drill_lift"

DRIVE = """
def run(api, **kw):
    tx = kw.get('x')
    tyaw = kw.get('yaw', 0.0)
    vmax = 0.25
    n = 0
    while n < 120:
        st = api.observe()
        x = st['base'][0]
        err = (tx - x) if tx is not None else 0.0
        yaw = st.get('base_yaw_deg', 0.0)
        yerr = tyaw - yaw
        if err < 0.04 and err > -0.04 and yerr < 4.0 and yerr > -4.0:
            break
        v = err * 2.0
        if v > vmax: v = vmax
        if v < -vmax: v = -vmax
        w = yerr * 0.02
        if w > 0.4: w = 0.4
        if w < -0.4: w = -0.4
        api.step_goal(nav=[v, 0.0, w], dur=0.2)
        n = n + 1
    api.step_goal(nav=[0.0, 0.0, 0.0], dur=0.6)
    st = api.observe()
    return {'x': st['base'][0], 'yaw': st.get('base_yaw_deg'), 'iters': n}
"""


def qtool(queue, req, timeout=180):
    env = os.environ.copy()
    env["HMG_TASK"] = queue
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True)
    try:
        return json.loads(p.stdout)
    except ValueError:
        return {"ok": False, "error": p.stdout.strip() or p.stderr.strip()}


def gain(queue, seed, delta=0.10):
    qtool(queue, {"tool": "reset", "seed": seed})
    r = qtool(queue, {"tool": "track_gain",
                      "kwargs": {"joints": ["waist_pitch_joint"],
                                 "delta": delta, "dur": 1.5, "hold": 1.0}})
    return ((r.get("result") or {}).get("joints") or {}).get(
        "waist_pitch_joint", {}).get("gain")


def main():
    slot = "ho1"
    queue = TASK + "__" + slot
    env = os.environ.copy()
    env["HMG_VENV"] = VENV
    subprocess.run(["bash", SERVER_CTL, "start", TASK, "0", slot], cwd=ROOT,
                   env=env, capture_output=True, text=True)
    import glob
    import time
    logs = sorted(glob.glob(os.path.join(ROOT, ".scratch", "srv_%s_*.log" % queue)))
    deadline = time.time() + 120
    while time.time() < deadline:
        if logs and "server ready" in open(logs[-1], errors="replace").read():
            break
        time.sleep(0.5)
    qtool(queue, {"tool": "write", "name": "drive", "source": DRIVE})

    out = {"handover": [], "lower_owner": {}, "balance": {}, "dim": {}}
    for seed in (0, 1, 2, 3):
        qtool(queue, {"tool": "authority", "kwargs": {"waist_owner": "homie"}})
        g_h = gain(queue, seed)
        qtool(queue, {"tool": "authority", "kwargs": {"waist_owner": "ik"}})
        g_i = gain(queue, seed)
        qtool(queue, {"tool": "authority", "kwargs": {"waist_owner": "homie"}})
        g_b = gain(queue, seed)
        out["handover"].append({"seed": seed, "homie": g_h, "ik": g_i,
                                "homie_again": g_b})

    # lower owner: policy vs hold, measured as walking displacement
    qtool(queue, {"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    for mode in ("policy", "hold"):
        qtool(queue, {"tool": "reset", "seed": 0})
        qtool(queue, {"tool": "authority", "kwargs": {"lower_owner": mode}})
        x0 = ((qtool(queue, {"tool": "observe"}).get("result") or {})
              .get("base") or [0.0])[0]
        d = qtool(queue, {"tool": "call", "name": "drive",
                          "kwargs": {"x": x0 + 0.5}})
        x1 = ((qtool(queue, {"tool": "observe"}).get("result") or {})
              .get("base") or [0.0])[0]
        out["lower_owner"][mode] = {"achieved_dx": round(x1 - x0, 3),
                                    "ok": d.get("ok")}
    qtool(queue, {"tool": "authority", "kwargs": {"lower_owner": "policy"}})

    # balance: 1.0 m walk + 45 deg turn under homie vs ik
    for mode in ("homie", "ik"):
        qtool(queue, {"tool": "authority", "kwargs": {"waist_owner": mode}})
        qtool(queue, {"tool": "reset", "seed": 0})
        st0 = (qtool(queue, {"tool": "observe"}).get("result") or {})
        x0 = (st0.get("base") or [0.0])[0]
        d = qtool(queue, {"tool": "call", "name": "drive",
                          "kwargs": {"x": x0 + 1.0, "yaw": 45.0}})
        st1 = (qtool(queue, {"tool": "observe"}).get("result") or {})
        out["balance"][mode] = {
            "achieved_dx": round(((st1.get("base") or [0.0])[0]) - x0, 3),
            "end_tilt_deg": st1.get("base_tilt_deg"),
            "end_base_z": (st1.get("base") or [None, None, None])[2],
            "ok": d.get("ok"),
            "error": d.get("error"),
        }

    # 12-D vs 17-D: qp_reach with waist homie vs ik
    qtool(queue, {"tool": "reset", "seed": 0})
    obj = ((qtool(queue, {"tool": "observe"}).get("result") or {})
           .get("object") or {}).get("pos")
    if obj:
        tg = {"left": [obj[0] - 0.06, obj[1] + 0.11, obj[2] + 0.02],
              "right": [obj[0] - 0.06, obj[1] - 0.11, obj[2] + 0.02]}
        for mode in ("homie", "ik"):
            qtool(queue, {"tool": "authority", "kwargs": {"waist_owner": mode}})
            qtool(queue, {"tool": "reset", "seed": 0})
            qtool(queue, {"tool": "call", "name": "drive",
                          "kwargs": {"x": obj[0] - 0.25}})
            r = qtool(queue, {"tool": "qp_reach",
                              "kwargs": {"targets": tg, "orientation": "keep",
                                         "iters": 60, "dur": 2.0}})
            res = r.get("result") or {}
            out["dim"][mode] = {"ik_err": res.get("ik_err"),
                                "palm": res.get("palm_residual"),
                                "ok": r.get("ok"), "error": r.get("error")}
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
