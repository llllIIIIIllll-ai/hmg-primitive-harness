#!/usr/bin/env python3
"""Round 0 deliverable 1: authority + measured gain per upper joint.

Measures the 31 upper joints with track_gain (direction-aware), then the
waist handover sequence homie -> ik -> homie with the achieved-gain check and
the default-owner restore.  Saves raw responses under
work/ll_v3/reports/round0_raw/gain/ and a combined gain_all.json.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "round0_raw", "gain")
TASK = "05_drill_lift"
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")
AUTH = os.path.join(ROOT, "docs", "joint_authority.json")

UPPER = [
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "left_hand_index_0_joint", "left_hand_index_1_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "right_hand_index_0_joint", "right_hand_index_1_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
]
WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]


def qtool(req, timeout=180):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 20)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def save(name, obj):
    os.makedirs(RAW, exist_ok=True)
    with open(os.path.join(RAW, name + ".json"), "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1)


def main():
    reg = json.load(open(AUTH, encoding="utf-8"))
    by_name = {j["joint"]: j for j in reg["joints"]}
    # direction-aware delta: use the registry's usable direction; one-sided
    # hand joints and shoulder-roll are the cases this protects.
    direction = {}
    for nm in UPPER:
        d = by_name[nm].get("measured_gain_direction")
        direction[nm] = -1.0 if d == "neg" else 1.0

    out = {"task": TASK, "seed": 0, "delta_rad": 0.2,
           "dur_s": 1.5, "hold_s": 1.0,
           "authority_registry": "docs/joint_authority.json",
           "homie": {}, "ik": {}, "restore": {},
           "waist_handover": {}}

    # Phase A: all 31 upper joints under the default owner homie.
    for nm in UPPER:
        qtool({"tool": "reset", "seed": 0})
        r = qtool({"tool": "track_gain",
                   "kwargs": {"joints": [nm], "delta": 0.2 * direction[nm],
                              "dur": 1.5, "hold": 1.0}})
        st = qtool({"tool": "probe", "what": "state"})
        save("homie_" + nm, {"track_gain": r, "state": st})
        out["homie"][nm] = {
            "final_writer_default": by_name[nm]["groups_enable_waist_true"]
            ["final_writer"],
            "authority_note": by_name[nm].get("measured_gain_note"),
            "delta_cmd": 0.2 * direction[nm],
            "measured_gain": r["result"]["joints"][nm]["gain"],
            "delta_meas": r["result"]["joints"][nm]["delta_meas"],
            "waist_owner": r["result"]["waist_owner"],
            "base_tilt_deg": st["result"].get("base_tilt_deg"),
        }
        print("homie", nm, out["homie"][nm]["measured_gain"])

    # Phase B: authorized handover to ik; waist gains after the switch.
    a = qtool({"tool": "authority", "kwargs": {"waist_owner": "ik"}})
    out["waist_handover"]["switch_to_ik"] = a["result"]
    assert a["result"]["waist_owner"] == "ik"
    for nm in WAIST:
        qtool({"tool": "reset", "seed": 0})
        r = qtool({"tool": "track_gain",
                   "kwargs": {"joints": [nm], "delta": 0.2 * direction[nm],
                              "dur": 1.5, "hold": 1.0}})
        st = qtool({"tool": "probe", "what": "state"})
        save("ik_" + nm, {"track_gain": r, "state": st})
        out["ik"][nm] = {
            "final_writer": by_name[nm]["final_writer_owners"]["ik"],
            "delta_cmd": 0.2 * direction[nm],
            "measured_gain": r["result"]["joints"][nm]["gain"],
            "delta_meas": r["result"]["joints"][nm]["delta_meas"],
            "waist_owner": r["result"]["waist_owner"],
            "base_tilt_deg": st["result"].get("base_tilt_deg"),
        }
        print("ik", nm, out["ik"][nm]["measured_gain"])

    # Phase C: restore the default owner homie; verify the gain returns.
    a = qtool({"tool": "authority", "kwargs": {"waist_owner": "homie"}})
    out["waist_handover"]["restore_homie"] = a["result"]
    assert a["result"]["waist_owner"] == "homie"
    for nm in WAIST:
        qtool({"tool": "reset", "seed": 0})
        r = qtool({"tool": "track_gain",
                   "kwargs": {"joints": [nm], "delta": 0.2 * direction[nm],
                              "dur": 1.5, "hold": 1.0}})
        st = qtool({"tool": "probe", "what": "state"})
        save("restore_" + nm, {"track_gain": r, "state": st})
        out["restore"][nm] = {
            "final_writer": by_name[nm]["final_writer_owners"]["homie"],
            "delta_cmd": 0.2 * direction[nm],
            "measured_gain": r["result"]["joints"][nm]["gain"],
            "delta_meas": r["result"]["joints"][nm]["delta_meas"],
            "waist_owner": r["result"]["waist_owner"],
            "base_tilt_deg": st["result"].get("base_tilt_deg"),
        }
        print("restore", nm, out["restore"][nm]["measured_gain"])

    out["waist_handover"]["measured_gain"] = {
        "homie_before": {nm: out["homie"][nm]["measured_gain"] for nm in WAIST},
        "ik_after": {nm: out["ik"][nm]["measured_gain"] for nm in WAIST},
        "homie_again": {nm: out["restore"][nm]["measured_gain"] for nm in WAIST},
    }
    out["waist_handover"]["restore_ok"] = all(
        out["restore"][nm]["waist_owner"] == "homie" for nm in WAIST)
    out["waist_handover"]["balance_ok"] = all(
        abs(out["homie"][nm]["base_tilt_deg"] or 0) <= 8.0
        and abs(out["ik"][nm]["base_tilt_deg"] or 0) <= 8.0
        and abs(out["restore"][nm]["base_tilt_deg"] or 0) <= 8.0
        for nm in WAIST)
    with open(os.path.join(HERE, "reports", "round0_raw", "gain_all.json"),
              "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print("wrote gain_all.json")


if __name__ == "__main__":
    main()
