#!/usr/bin/env python3
"""Round 0 calibration for ll_v2.

Establishes the control contract by measurement, before any capability work:

  1. authority + measured gain for every commandable upper-body joint
  2. ramp-lag curve (achieved gain vs ramp duration for several joint deltas)
  3. reach envelope per arm (read-only ik_preview grid)
  4. standoff envelope on one task (minimum reachable base standoff + pad error)

Writes experiments/ll_v2/calibration.json and docs/control_contract.md.

Usage:
  python scripts/calibrate.py --task 05_drill_lift \
      --out experiments/ll_v2/calibration.json
"""
import argparse
import json
import os
import subprocess
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
SERVER_CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")
SLOT = "cal0"

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
    return {'x': api.observe()['base'][0]}
"""

PREVIEW = """
def run(api, **kw):
    return api.ik_preview(targets=kw['targets'], iters=200,
                          sides=kw.get('sides', ('left', 'right')))
"""

REACH = """
def run(api, **kw):
    return api.reach_box(targets=kw['targets'], orientation='keep',
                         dur=kw.get('dur', 1.6), sides=kw.get('sides', ('left', 'right')))
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="05_drill_lift")
    ap.add_argument("--out", default="experiments/ll_v2/calibration.json")
    ap.add_argument("--skip-gain", action="store_true")
    args = ap.parse_args()
    queue = args.task + "__" + SLOT

    env = os.environ.copy()
    env["HMG_VENV"] = "/home/pjy0422/hmg_migration/stack/hmg_venv"
    subprocess.run(["bash", SERVER_CTL, "start", args.task, "0", SLOT],
                   cwd=ROOT, env=env, capture_output=True, text=True)
    import glob
    logs = sorted(glob.glob(os.path.join(
        ROOT, ".scratch", "srv_%s_*.log" % queue)))
    deadline = time.time() + 120
    while time.time() < deadline:
        if logs and "server ready" in open(logs[-1], errors="replace").read():
            break
        time.sleep(0.5)

    qtool(queue, {"tool": "write", "name": "drive", "source": DRIVE})
    qtool(queue, {"tool": "write", "name": "preview", "source": PREVIEW})
    qtool(queue, {"tool": "write", "name": "reach", "source": REACH})

    cal = {"task": args.task, "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
           "gains": {}, "ramp_curve": [], "reach_envelope": {},
           "standoff": []}

    # 1) authority + gain for the 31 commandable joints
    qtool(queue, {"tool": "reset", "seed": 0})
    jr = (qtool(queue, {"tool": "call", "name": "jr"}) or {}).get("result")
    if jr is None:
        qtool(queue, {"tool": "write", "name": "jr",
                      "source": "def run(api, **kw):\n    return api.joint_ranges()\n"})
        jr = (qtool(queue, {"tool": "call", "name": "jr"}) or {}).get("result") or {}
    names = [n for n in jr]
    if not args.skip_gain:
        for nm in names:
            r = qtool(queue, {"tool": "track_gain",
                              "kwargs": {"joints": [nm], "delta": 0.10,
                                         "dur": 1.5, "hold": 1.0}})
            res = (r.get("result") or {}).get("joints") or {}
            g = res.get(nm, {}).get("gain")
            cal["gains"][nm] = g
            qtool(queue, {"tool": "reset", "seed": 0})

    # 2) ramp-lag curve on one arm joint
    for delta in (0.10, 0.30, 0.50):
        for dur in (0.5, 1.0, 2.0):
            qtool(queue, {"tool": "reset", "seed": 0})
            r = qtool(queue, {"tool": "track_gain",
                              "kwargs": {"joints": ["left_shoulder_pitch_joint"],
                                         "delta": delta, "dur": dur, "hold": 1.0}})
            g = ((r.get("result") or {}).get("joints") or {}).get(
                "left_shoulder_pitch_joint", {}).get("gain")
            cal["ramp_curve"].append({"delta": delta, "dur": dur, "gain": g})

    # 3) reach envelope per arm at the minimum standoff
    qtool(queue, {"tool": "reset", "seed": 0})
    obj = (qtool(queue, {"tool": "observe"}).get("result") or {}).get(
        "object", {}).get("pos")
    if obj:
        qtool(queue, {"tool": "call", "name": "drive",
                      "kwargs": {"x": obj[0] - 0.25}})
        env_rows = {"left": [], "right": []}
        for side, sign in (("left", 1.0), ("right", -1.0)):
            for dx in (-0.06, -0.03, 0.0, 0.03):
                for dy in (0.03, 0.07, 0.11):
                    tgt = {side: [obj[0] + dx, obj[1] + sign * dy, obj[2] + 0.02]}
                    r = qtool(queue, {"tool": "call", "name": "preview",
                                      "kwargs": {"targets": tgt, "sides": (side,)}})
                    rows = (r.get("result") or {}).get("rows") or []
                    if rows:
                        env_rows[side].append({"dx": dx, "dy": sign * dy,
                                               "err": rows[0].get("err"),
                                               "seed": rows[0].get("seed")})
        cal["reach_envelope"] = env_rows

        # 4) standoff envelope
        for d in (0.25, 0.30, 0.40, 0.50):
            qtool(queue, {"tool": "reset", "seed": 0})
            drv = qtool(queue, {"tool": "call", "name": "drive",
                                "kwargs": {"x": obj[0] - d}})
            bx = (drv.get("result") or {}).get("x")
            tg = {"left": [obj[0], obj[1] + 0.07, obj[2] + 0.02],
                  "right": [obj[0], obj[1] - 0.07, obj[2] + 0.02]}
            rr = qtool(queue, {"tool": "call", "name": "reach",
                               "kwargs": {"targets": tg}})
            reach = (rr.get("result") or {}) or {}
            cal["standoff"].append({
                "cmd": d,
                "base_x": bx,
                "achieved": None if bx is None else round(obj[0] - bx, 3),
                "ik_err": reach.get("ik_err"),
                "palm": reach.get("palm_residual"),
            })

    os.makedirs(os.path.dirname(os.path.join(ROOT, args.out)), exist_ok=True)
    with open(os.path.join(ROOT, args.out), "w", encoding="utf-8") as fh:
        json.dump(cal, fh, indent=1)
    print(json.dumps(cal, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
