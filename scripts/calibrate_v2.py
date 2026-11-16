#!/usr/bin/env python3
"""Round 0 calibration for ll_v2 (completes and verifies the control contract).

Measures, on the live simulator:

  A. direction-aware track gain for every commandable upper-body joint
     (+delta and -delta where the target stays inside the model range), so a
     one-sided direction with gain below min_gain_for_commands is visible;
  B. ramp-lag curve: achieved gain at the END of the ramp (hold=0) for joint
     deltas 0.1/0.3/0.5/1.0 over several ramp durations -> ramp_s(delta);
  C. dimension A/B: the IK candidates 12/14/17 on the same live targets at the
     measured standoff (read-only ik_compare + an executed 17/12 authority
     pair on the waist);
  D. standoff envelope per task: the base standoff (object-body x minus base x)
     at which the arms reach, the achieved pad residual and the object
     disturbance caused by the approach.

Output: experiments/ll_v2/calibration.json and docs/control_contract.md.

Usage:
  python scripts/calibrate_v2.py [--skip-gains] [--skip-standoff]
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
SERVER_CTL = os.path.join(ROOT, "scripts", "server_ctl.sh")
TASKS = ["01_box_lift_floor", "02_push_button", "03_box_lift",
         "04_push_shelf_forward", "05_drill_lift", "06_drill_pnp",
         "07_box_table_to_shelf", "08_pick_drill_from_holder",
         "09_obstacle_aware_pick_drill"]
CAL_TASK = "05_drill_lift"
CAL_SLOT = "cal0"
VENV = os.environ.get("HMG_VENV", "/home/pjy0422/hmg_migration/stack/hmg_venv")

DRIVE = '''
def run(api, **kw):
    tx = kw["x"]
    vmax = kw.get("vmax", 0.20)
    n = 0
    while n < 100:
        st = api.observe()
        err = tx - st["base"][0]
        if -0.02 < err < 0.02:
            break
        v = err * 2.0
        if v > vmax:
            v = vmax
        if v < -vmax:
            v = -vmax
        api.step_goal(nav=[v, 0.0, 0.0], dur=0.2)
        n = n + 1
    api.step_goal(nav=[0.0, 0.0, 0.0], dur=0.6)
    return {"x": api.observe()["base"][0], "iters": n}
'''

JR = '''
def run(api, **kw):
    return api.joint_ranges()
'''


def qtool(queue, req, timeout=180):
    env = os.environ.copy()
    env["HMG_TASK"] = queue
    proc = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                          cwd=ROOT, env=env, capture_output=True, text=True)
    try:
        return json.loads(proc.stdout)
    except ValueError:
        return {"ok": False, "error": proc.stdout.strip() or proc.stderr.strip()}


def result(resp):
    return resp.get("result") if isinstance(resp, dict) and resp.get("ok") else None


def start(task, slot, seed=0):
    env = os.environ.copy()
    env["HMG_VENV"] = VENV
    subprocess.run(["bash", SERVER_CTL, "start", task, str(seed), slot],
                   cwd=ROOT, env=env, capture_output=True, text=True)
    queue = "%s__%s" % (task, slot)
    deadline = time.time() + 180
    while time.time() < deadline:
        hits = sorted(glob.glob(os.path.join(
            ROOT, ".scratch", "srv_%s_*.log" % queue)))
        if hits:
            try:
                if "server ready" in open(hits[-1], errors="replace").read():
                    return queue
            except OSError:
                pass
        time.sleep(0.3)
    raise RuntimeError("server not ready: %s" % queue)


def ensure_registered(queue):
    qtool(queue, {"tool": "write", "name": "drive", "source": DRIVE})
    qtool(queue, {"tool": "write", "name": "jr", "source": JR})


def measure_gains(queue):
    """Direction-aware gain per joint at delta +/-0.2 (legal direction only)."""
    qtool(queue, {"tool": "reset", "seed": 0})
    ranges = result(qtool(queue, {"tool": "call", "name": "jr"})) or {}
    out = {}
    for name in ranges:
        if name in ("t0", "t1"):
            continue
        rec = ranges[name]
        q0, lo, hi = float(rec["q"]), float(rec["lo"]), float(rec["hi"])
        rec_out = {"q0": q0, "lo": lo, "hi": hi, "gain_pos": None,
                   "gain_neg": None}
        for sign, key in ((1.0, "gain_pos"), (-1.0, "gain_neg")):
            tgt = q0 + sign * 0.2
            if tgt < lo - 1e-9 or tgt > hi + 1e-9:
                continue
            qtool(queue, {"tool": "reset", "seed": 0})
            r = result(qtool(queue, {"tool": "track_gain", "kwargs": {
                "joints": [name], "delta": sign * 0.2,
                "dur": 1.5, "hold": 1.0}})) or {}
            g = ((r.get("joints") or {}).get(name) or {}).get("gain")
            rec_out[key] = g
        # the usable direction is the larger measured gain
        gp, gn = rec_out["gain_pos"], rec_out["gain_neg"]
        if gp is None:
            usable, best = "neg", gn
        elif gn is None:
            usable, best = "pos", gp
        else:
            usable, best = (("pos", gp) if gp >= gn else ("neg", gn))
        rec_out["usable_direction"] = usable
        rec_out["usable_gain"] = best
        rec_out["commandable"] = (best is not None and best >= 0.8)
        out[name] = rec_out
    return out


def measure_ramp(queue):
    """Gain at ramp end (hold=0) -> ramp_s(delta) at the 95% point."""
    joints = ("left_shoulder_pitch_joint", "left_elbow_joint",
              "right_shoulder_roll_joint")
    rows = []
    for delta in (0.1, 0.3, 0.5, 1.0):
        for dur in (0.1, 0.2, 0.3, 0.5, 0.8, 1.2, 2.0):
            gains = {}
            for j in joints:
                qtool(queue, {"tool": "reset", "seed": 0})
                d = delta if j != "right_shoulder_roll_joint" else -delta
                r = result(qtool(queue, {"tool": "track_gain", "kwargs": {
                    "joints": [j], "delta": d, "dur": dur, "hold": 0.0}})) or {}
                gains[j] = ((r.get("joints") or {}).get(j) or {}).get("gain")
            rows.append({"delta": delta, "dur": dur, "gains": gains})
    curve = []
    for delta in (0.1, 0.3, 0.5, 1.0):
        sub = [r for r in rows if r["delta"] == delta]
        hit = None
        for r in sorted(sub, key=lambda z: z["dur"]):
            gs = [g for g in r["gains"].values() if g is not None]
            if gs and min(gs) >= 0.95:
                hit = r
                break
        curve.append({
            "delta": delta,
            "ramp_s_95": None if hit is None else hit["dur"],
            "sample": [{"dur": r["dur"], "gains": r["gains"]} for r in sub],
        })
    return curve


def pad_targets(api_aabb):
    """Desired pad contact points from the live object AABB (side faces)."""
    lo, hi = api_aabb["lo"], api_aabb["hi"]
    bx = api_aabb["body_pos"][0]
    hw = 0.0217
    z = lo[2] + 0.25 * (hi[2] - lo[2])
    return {"left": [bx, hi[1] + hw, z], "right": [bx, lo[1] - hw, z]}, z


def measure_dim(queue):
    """C: live-target dimension comparison plus an executed 12 vs 17 pair."""
    qtool(queue, {"tool": "reset", "seed": 0})
    obj0 = (result(qtool(queue, {"tool": "probe", "what": "object"})) or {})
    aabb0 = obj0.get("object") or {}
    obj_x = (aabb0.get("body_pos") or [None])[0]
    if obj_x is None:
        return {"error": "no object at reset"}
    qtool(queue, {"tool": "call", "name": "drive", "kwargs": {"x": obj_x - 0.25}})
    st = result(qtool(queue, {"tool": "observe"})) or {}
    obj = (st.get("object") or {}).get("pos") or [obj_x, 0.0, 0.0]
    tgt = {"left": [obj[0], obj[1] + 0.07, obj[2] + 0.02],
           "right": [obj[0], obj[1] - 0.07, obj[2] + 0.02]}
    bounds = {"right_shoulder_roll_joint": [-2.252, 0.05]}
    cmp12 = result(qtool(queue, {"tool": "ik_compare", "kwargs": {
        "targets": tgt, "orientation": "inward_down",
        "task_dim_candidates": [12, 14, 17], "iters": 120,
        "joint_bounds": bounds}}, timeout=300))
    out = {"targets": tgt, "standoff_body_x": round(obj[0] - (st.get("base") or [0])[0], 3),
           "ik_compare": cmp12}
    # executed authority pair: 17 commands the waist; 12 does not
    pair = {}
    for dim in (17, 12):
        qtool(queue, {"tool": "reset", "seed": 0})
        qtool(queue, {"tool": "call", "name": "drive",
                      "kwargs": {"x": obj_x - 0.25}})
        r = result(qtool(queue, {"tool": "qp_reach", "kwargs": {
            "targets": tgt, "orientation": "inward_down", "dur": 1.6,
            "iters": 120, "task_dim": dim, "joint_bounds": bounds}},
            timeout=300)) or {}
        a = result(qtool(queue, {"tool": "audit"})) or {}
        waist = {}
        for nm, rec in (a.get("joints") or {}).items():
            if nm.startswith("waist"):
                waist[nm] = {"cmd": rec["cmd"], "meas": rec["meas"],
                             "err": rec["err"]}
        pair[str(dim)] = {
            "ik_err": r.get("ik_err"), "palm_residual": r.get("palm_residual"),
            "joint_margin_min": r.get("joint_margin_min"),
            "solver": r.get("solver"), "waist": waist,
            "state": r.get("state"),
        }
    out["executed_pair"] = pair
    return out


def measure_standoff_task(task, slot):
    queue = start(task, slot)
    ensure_registered(queue)
    qtool(queue, {"tool": "reset", "seed": 0})
    obj0 = result(qtool(queue, {"tool": "probe", "what": "object"})) or {}
    a0 = obj0.get("object") or {}
    p0 = a0.get("body_pos")
    if not p0:
        return {"task": task, "error": "no object"}
    rows = []
    for standoff in (0.25, 0.35, 0.50):
        qtool(queue, {"tool": "reset", "seed": 0})
        qtool(queue, {"tool": "call", "name": "drive",
                      "kwargs": {"x": p0[0] - standoff}})
        st = result(qtool(queue, {"tool": "observe"})) or {}
        obj = (st.get("object") or {}).get("pos") or p0
        drift = ((obj[0] - p0[0]) ** 2 + (obj[1] - p0[1]) ** 2
                 + (obj[2] - p0[2]) ** 2) ** 0.5
        tgt = {"left": [obj[0], obj[1] + 0.07, obj[2] + 0.02],
               "right": [obj[0], obj[1] - 0.07, obj[2] + 0.02]}
        r = result(qtool(queue, {"tool": "qp_reach", "kwargs": {
            "targets": tgt, "orientation": "keep", "dur": 1.6, "iters": 80,
            "task_dim": 12,
            "joint_bounds": {"right_shoulder_roll_joint": [-2.252, 0.05]}}},
            timeout=300)) or {}
        rows.append({
            "standoff_cmd": standoff,
            "base_x": (st.get("base") or [None])[0],
            "achieved_standoff": round(obj[0] - (st.get("base") or [0])[0], 3),
            "object_drift_m": round(drift, 4),
            "ik_err": r.get("ik_err"),
            "palm_residual": r.get("palm_residual"),
            "joint_margin_min": r.get("joint_margin_min"),
            "solver": r.get("solver"),
            "base_tilt_deg": st.get("base_tilt_deg"),
        })
    return {"task": task, "rows": rows}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-gains", action="store_true")
    ap.add_argument("--skip-standoff", action="store_true")
    ap.add_argument("--out", default="experiments/ll_v2/calibration.json")
    args = ap.parse_args()

    queue = start(CAL_TASK, CAL_SLOT)
    ensure_registered(queue)
    cal = {"task": CAL_TASK,
           "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
           "control": {"dof_total": 43, "upper_commands": 31,
                       "exogenous_joints": ["waist_yaw_joint",
                                            "waist_roll_joint",
                                            "waist_pitch_joint"],
                       "min_gain_for_commands": 0.8},
           "gains": {}, "ramp_curve": [], "dimension_ab": {},
           "standoff": {}}

    if not args.skip_gains:
        print("[A] direction-aware gains", flush=True)
        cal["gains"] = measure_gains(queue)
        print("[B] ramp curve", flush=True)
        cal["ramp_curve"] = measure_ramp(queue)

    print("[C] dimension comparison", flush=True)
    cal["dimension_ab"] = measure_dim(queue)

    if not args.skip_standoff:
        print("[D] standoff envelope per task", flush=True)
        results = {}
        lock = threading.Lock()

        def one(task, slot):
            try:
                rec = measure_standoff_task(task, slot)
            except Exception as exc:
                rec = {"task": task, "error": "%s: %s"
                       % (type(exc).__name__, exc)}
            with lock:
                results[task] = rec

        threads = [threading.Thread(target=one, args=(t, "cal9"))
                   for t in TASKS]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        cal["standoff"] = results

    path = os.path.join(ROOT, args.out)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(cal, fh, indent=1)
    print(json.dumps(cal, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
