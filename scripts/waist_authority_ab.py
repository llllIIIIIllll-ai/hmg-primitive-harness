#!/usr/bin/env python3
"""Waist authority A/B: HOMIE-owned vs IK-owned vs blend.

For each mode, start a fresh 05_drill_lift server with HMG_WAIST_OWNER set,
then measure:
  - waist pitch tracking gain at +0.10 and +0.30 rad
  - standing base tilt after the waist command
  - walking: closed-loop forward 0.50 m; report achieved displacement,
    end tilt and whether the base dropped (fall)

Usage:
  python scripts/waist_authority_ab.py --task 05_drill_lift
"""
import argparse
import glob
import json
import os
import subprocess
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
SERVER_CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")
VENV = "/home/pjy0422/hmg_migration/stack/hmg_venv"

DRIVE = """
def run(api, **kw):
    tx = kw['x']
    vmax = 0.25
    n = 0
    while n < 80:
        x = api.observe()['base'][0]
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
    return {'x': api.observe()['base'][0], 'iters': n}
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


def run_mode(task, mode, blend):
    slot = "wa_" + mode
    queue = task + "__" + slot
    env = os.environ.copy()
    env["HMG_VENV"] = VENV
    env["HMG_WAIST_OWNER"] = mode
    env["HMG_WAIST_BLEND"] = str(blend)
    subprocess.run(["bash", SERVER_CTL, "start", task, "0", slot],
                   cwd=ROOT, env=env, capture_output=True, text=True)
    logs = sorted(glob.glob(os.path.join(ROOT, ".scratch",
                                         "srv_%s_*.log" % queue)))
    deadline = time.time() + 120
    while time.time() < deadline:
        if logs and "server ready" in open(logs[-1], errors="replace").read():
            break
        time.sleep(0.5)
    qtool(queue, {"tool": "write", "name": "drive", "source": DRIVE})
    out = {"mode": mode, "blend": blend}

    # waist tracking gain (standing)
    qtool(queue, {"tool": "reset", "seed": 0})
    gains = {}
    for delta in (0.10, 0.30):
        qtool(queue, {"tool": "reset", "seed": 0})
        r = qtool(queue, {"tool": "track_gain",
                          "kwargs": {"joints": ["waist_pitch_joint"],
                                     "delta": delta, "dur": 1.5, "hold": 1.0}})
        g = ((r.get("result") or {}).get("joints") or {}).get(
            "waist_pitch_joint", {})
        gains[str(delta)] = g
    out["waist_gain"] = gains
    st = (qtool(queue, {"tool": "observe"}).get("result") or {})
    out["stand_tilt_deg"] = st.get("base_tilt_deg")
    out["stand_base_z"] = (st.get("base") or [None, None, None])[2]

    # walking stability
    qtool(queue, {"tool": "reset", "seed": 0})
    x0 = ((qtool(queue, {"tool": "observe"}).get("result") or {})
          .get("base") or [None])[0]
    d = qtool(queue, {"tool": "call", "name": "drive",
                      "kwargs": {"x": (x0 or 0.0) + 0.5}})
    st = (qtool(queue, {"tool": "observe"}).get("result") or {})
    x1 = (st.get("base") or [None, None, None])[0]
    out["walk"] = {
        "cmd_dx": 0.5,
        "achieved_dx": None if (x0 is None or x1 is None) else round(x1 - x0, 3),
        "end_tilt_deg": st.get("base_tilt_deg"),
        "end_base_z": (st.get("base") or [None, None, None])[2],
        "drive_ok": d.get("ok"),
        "drive_err": d.get("error"),
    }
    out["walk"]["fell"] = bool(
        (out["walk"]["end_base_z"] is not None and out["walk"]["end_base_z"] < 0.55)
        or (out["walk"]["end_tilt_deg"] is not None
            and out["walk"]["end_tilt_deg"] > 30))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="05_drill_lift")
    ap.add_argument("--modes", default="homie,ik,blend")
    ap.add_argument("--blend", type=float, default=0.5)
    args = ap.parse_args()
    rows = []
    for mode in [m.strip() for m in args.modes.split(",") if m.strip()]:
        rows.append(run_mode(args.task, mode, args.blend))
    print(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
