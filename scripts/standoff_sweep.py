#!/usr/bin/env python3
"""Experiment 1: base-standoff sweep on 05_drill_lift.

For each base standoff, reset the episode, drive the base to that standoff from
the drill, then run the arms-only DLS reach (reach_box) to the drill side faces
and measure the executed pad error, the model residual and the base drift.

Decision rule: if any standoff the base can actually hold gives a palm residual
below ~12 mm, the missing piece is an approach/standoff primitive; if none does,
the reach is structurally short at every reachable standoff.
"""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
SERVER_CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")
TASK = "05_drill_lift"
SLOT = "sweep1"
QUEUE = TASK + "__" + SLOT


def qtool(req, timeout=120):
    env = os.environ.copy()
    env["HMG_TASK"] = QUEUE
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True)
    try:
        return json.loads(p.stdout)
    except ValueError:
        return {"ok": False, "error": p.stdout.strip() or p.stderr.strip()}


DRIVE = """
def run(api, **kw):
    tx = kw['x']
    vmax = kw.get('vmax', 0.25)
    n = 0
    while n < 80:
        st = api.observe()
        x = st['base'][0]
        err = tx - x
        if err < 0.03 and err > -0.03:
            break
        v = err * 2.0
        if v > vmax:
            v = vmax
        if v < -vmax:
            v = -vmax
        api.step_goal(nav=[v, 0.0, 0.0], dur=0.2)
        n = n + 1
    api.step_goal(nav=[0.0, 0.0, 0.0], dur=0.6)
    st = api.observe()
    return {'x': st['base'][0], 'target_x': tx, 'iters': n}
"""

REACH = """
def run(api, **kw):
    b0 = api.observe()['base']
    r = api.reach_box(targets=kw['targets'], orientation=kw.get('orientation', 'keep'), dur=1.6)
    b1 = api.observe()['base']
    return {'reach': r, 'base0': b0, 'base1': b1}
"""


def main():
    standoffs = [float(x) for x in (sys.argv[1:] or
                                    ["0.30", "0.35", "0.40", "0.45", "0.50", "0.55", "0.60"])]
    env = os.environ.copy()
    env["HMG_VENV"] = "/home/pjy0422/hmg_migration/stack/hmg_venv"
    subprocess.run(["bash", SERVER_CTL, "start", TASK, "0", SLOT], cwd=ROOT,
                   env=env, capture_output=True, text=True)
    import glob
    import time
    logs = sorted(glob.glob(os.path.join(ROOT, ".scratch", "srv_%s_*.log" % QUEUE)))
    deadline = time.time() + 120
    while time.time() < deadline:
        if logs and "server ready" in open(logs[-1], errors="replace").read():
            break
        time.sleep(0.5)
    qtool({"tool": "write", "name": "drive", "source": DRIVE})
    qtool({"tool": "write", "name": "reach_test", "source": REACH})
    out = []
    for d in standoffs:
        qtool({"tool": "reset", "seed": 0})
        st = qtool({"tool": "observe"})
        obj = st.get("result", {}).get("object", {}).get("pos")
        if not obj:
            out.append({"standoff": d, "error": "no object"})
            continue
        tx = obj[0] - d
        drv = qtool({"tool": "call", "name": "drive", "kwargs": {"x": tx}})
        targets = {
            "left": [obj[0], obj[1] + 0.04, obj[2] + 0.02],
            "right": [obj[0], obj[1] - 0.04, obj[2] + 0.02],
        }
        rr = qtool({"tool": "call", "name": "reach_test",
                    "kwargs": {"targets": targets, "orientation": "keep"}})
        res = rr.get("result") or {}
        reach = res.get("reach") or {}
        b0, b1 = res.get("base0"), res.get("base1")
        drift = None
        if b0 and b1:
            drift = round(float(((b1[0] - b0[0]) ** 2 + (b1[1] - b0[1]) ** 2) ** 0.5), 4)
        out.append({
            "standoff_cmd": d,
            "base_x": (drv.get("result") or {}).get("x"),
            "standoff_achieved": round(float(obj[0] - ((drv.get("result") or {}).get("x") or obj[0])), 3),
            "palm_residual": reach.get("palm_residual"),
            "ik_err": reach.get("ik_err"),
            "pad": reach.get("pad"),
            "base_drift_m": drift,
            "ok": rr.get("ok"),
            "error": rr.get("error"),
        })
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
