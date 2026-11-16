#!/usr/bin/env python3
"""Build experiments/ll_v3/calibration.json from the round-0 raw measurements."""
import json
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RAW = os.path.join(HERE, "reports", "round0_raw")
OUT = os.path.join(ROOT, "experiments", "ll_v3", "calibration.json")


def load(name):
    with open(os.path.join(RAW, name), encoding="utf-8") as fh:
        return json.load(fh)


def norm(v):
    return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5


def main():
    reg = json.load(open(os.path.join(ROOT, "docs", "joint_authority.json"),
                         encoding="utf-8"))
    gain = load("gain_all.json")
    ramp = load("ramp_all.json")
    reach_pose = load("reach_pose.json")["result"]["measurements"][0]
    reach_env = load("reach_envelope.json")["result"]
    cmp_h = load("ik_compare_homie.json")["result"]
    cmp_i = load("ik_compare_ik.json")["result"]
    q12 = load("qp_reach_12d_homie.json")["result"]
    q17 = load("qp_reach_17d_ik.json")["result"]
    q12b = load("qp_reach_12d_restore.json")["result"]
    auth_ik = load("authority_ik.json")["result"]
    auth_restore = load("authority_homie_restore.json")["result"]
    state_restore = load("state_after_restore.json")["result"]
    proxy = load("grasp_proxy_summary.json")
    fine = load("standoff_05_fine.json")["result"]["measurements"]

    # ---- authority + gain (all 43 final writers; 31 measured) ----
    robot_auth = {}
    for j in reg["joints"]:
        name = j["joint"]
        g = j["groups_enable_waist_true"]
        robot_auth[name] = {
            "final_writer_default": g["final_writer"],
            "upper_body": g["upper_body"],
            "lower_body": g["lower_body"],
            "commandable_upper": g["final_writer"] == "upper_target",
            "measured_gain_registry": j.get("measured_gain"),
        }
        if name in ("waist_yaw_joint", "waist_roll_joint",
                    "waist_pitch_joint"):
            robot_auth[name]["final_writer_owners"] = j.get(
                "final_writer_owners")
            robot_auth[name]["measured_gain_handover_registry"] = j.get(
                "measured_gain_handover")

    measured = {}
    for name, rec in gain["homie"].items():
        measured[name] = {
            "final_writer_default": rec["final_writer_default"],
            "authority_note": rec["authority_note"],
            "delta_cmd": rec["delta_cmd"],
            "measured_gain_homie": rec["measured_gain"],
            "delta_meas": rec["delta_meas"],
            "waist_owner": rec["waist_owner"],
            "base_tilt_deg": rec["base_tilt_deg"],
        }
        if name in gain["ik"]:
            measured[name]["final_writer_ik"] = gain["ik"][name][
                "final_writer"]
            measured[name]["measured_gain_ik"] = gain["ik"][name][
                "measured_gain"]
            measured[name]["measured_gain_restore_homie"] = gain["restore"][
                name]["measured_gain"]

    # ---- ramp curve ----
    t95 = {}
    for key, rec in ramp["measurements"].items():
        t95[key] = {"t95_s": rec.get("t95_s"),
                    "achieved_frac": rec.get("achieved_frac"),
                    "t_end_s": rec.get("t_end")}
    recommended = {}
    for delta in (0.1, 0.3, 0.5, 1.0):
        vals = [t95[k]["t95_s"] for k in t95
                if k.endswith(":%+.1f" % delta) and t95[k]["t95_s"]]
        m = max(vals) if vals else None
        recommended["%.1f" % delta] = (None if m is None
                                       else round((int(m / 0.05) + 1) * 0.05,
                                                  2))
    ramp_out = {
        "method": "step command, per-control-step sampling, time to 95% of "
                  "the commanded delta",
        "deltas_rad": [0.1, 0.3, 0.5, 1.0],
        "measurements": t95,
        "ramp_s_by_delta": recommended,
        "note": "The measured step-response t95 is 0.10-0.32 s, much shorter "
                "than the fixed 1.5 s _ramp used by track_gain.  Use the "
                "recommended ramp_s(delta) instead of a constant ramp.",
    }

    # ---- reach envelope ----
    per_side = {}
    for side in ("left", "right"):
        rows = [r for r in reach_env["rows"] if r["side"] == side]
        errs = [r["err"] for r in rows]
        best = min(rows, key=lambda r: r["err"])
        per_side[side] = {
            "n": len(rows),
            "reachable_le_0p015": sum(1 for e in errs if e <= 0.015),
            "mean_err": round(statistics.mean(errs), 4),
            "min_err": min(errs), "max_err": max(errs),
            "best": {k: best[k] for k in ("xo", "gap", "z", "err", "rot",
                                          "seed", "dq", "pad_travel")},
        }
    lm = {(r["xo"], r["gap"], r["z"]): r["err"]
          for r in reach_env["rows"] if r["side"] == "left"}
    rm = {(r["xo"], r["gap"], r["z"]): r["err"]
          for r in reach_env["rows"] if r["side"] == "right"}
    diffs = [lm[k] - rm[k] for k in lm if k in rm]
    reach_out = {
        "pose": {"base": reach_pose["base"],
                 "achieved_standoff_aabb": reach_pose[
                     "achieved_standoff_aabb"],
                 "achieved_standoff_root": reach_pose[
                     "achieved_standoff_root"]},
        "grid": {"xs": [-0.04, 0.02], "gaps": [0.155, 0.175],
                 "zs": [0.46, 0.52, 0.58, 0.64, 0.70],
                 "orients": ["down"], "iters": 150},
        "solver": "DLS (ik_preview, read-only)",
        "per_side": per_side,
        "asymmetry": {"mean_left_minus_right": round(
            statistics.mean(diffs), 4),
            "max_abs": round(max(abs(x) for x in diffs), 4)},
        "rows": reach_env["rows"],
        "note": "The DLS preview residual (0.017-0.208 m) overstates the "
                "mink-QP executed residual (0.008-0.015 m).  Use the preview "
                "only as a reachability screen; the QP is the adopted solver.",
    }

    # ---- standoff envelope ----
    def fine_row(target_so):
        for m in fine:
            if abs(m["target_standoff"] - target_so) < 1e-9:
                return m
        return None

    def standoff_summary(m):
        pr = (m.get("reach") or {}).get("palm_residual") or {}
        if pr:
            worst = max(norm(v) for v in pr.values())
        else:
            pe = m.get("pad_error") or {}
            worst = max(pe.values()) if pe else None
        return {
            "target_root_standoff": m["target_standoff"],
            "achieved_root_standoff": round(m[
                "achieved_standoff_root"], 3) if "achieved_standoff_root"
                in m else round(m["achieved_standoff"], 3),
            "worst_pad_error": None if worst is None else round(worst, 4),
            "joint_margin_min": (m["reach"] or {}).get("joint_margin_min"),
            "base_tilt_deg": m["base"]["tilt_deg"],
            "base_pos": m["base"]["pos"],
            "binding_constraint": (
                "pad_error" if (worst is not None and worst > 0.015)
                else "joint_margin" if (
                    (m["reach"] or {}).get("joint_margin_min") is not None
                    and (m["reach"] or {})["joint_margin_min"] < 0.05)
                else "ok"),
        }

    fine_rows = [standoff_summary(m) for m in fine]
    valid05 = [r for r in fine_rows if r["binding_constraint"] == "ok"]
    best05 = min(valid05, key=lambda r: r["achieved_root_standoff"]) if \
        valid05 else None
    # AABB distance at the best 05 pose
    if best05 is not None:
        bx = best05["base_pos"][0]
        aabb_lo_x = 1.63726  # live 05 object AABB lo x
        best05["achieved_aabb_standoff"] = round(aabb_lo_x - bx, 3)

    blocked = {
        "01_box_lift_floor": {
            "status": "blocked",
            "binding_constraint": "base_tilt_guard",
            "measured": "set_height(0.46) + move_base approach tilted "
                        "19.8-29.4 deg; base ran backward; pad error 0.98-1.54",
            "gap": "G0/G7"},
        "02_push_button": {
            "status": "blocked",
            "binding_constraint": "target_invalid",
            "measured": "best pad error 0.156 at AABB 0.192 with the panel "
                        "AABB-centre target; the predicate target is the "
                        "button (control_panel_button_z), not the panel centre",
            "gap": "G0/G2"},
        "03_box_lift": {
            "status": "blocked",
            "binding_constraint": "target_invalid",
            "measured": "best pad error 0.466 at AABB 0.362; the generic "
                        "target height is above the box",
            "gap": "G0/G2"},
        "04_push_shelf_forward": {
            "status": "blocked",
            "binding_constraint": "base_tilt_guard",
            "measured": "shelf approach tilted 20.7-33.0 deg and pushed the "
                        "16 kg shelf; the push standoff is a contact "
                        "distance, not a grasp standoff",
            "gap": "G0/G8"},
        "06_drill_pnp": {
            "status": "blocked",
            "binding_constraint": "target_invalid",
            "measured": "best pad error 0.091 at AABB 0.157; the drill target "
                        "height needs the task-specific grasp point",
            "gap": "G0/G2"},
        "07_box_table_to_shelf": {
            "status": "blocked",
            "binding_constraint": "target_invalid",
            "measured": "best pad error 0.440 at AABB 0.306; the box target "
                        "needs the task-specific grasp point",
            "gap": "G0/G2"},
        "08_pick_drill_from_holder": {
            "status": "blocked",
            "binding_constraint": "target_invalid",
            "measured": "best pad error 0.072 at AABB 0.195; the holder "
                        "blocks the generic drill target",
            "gap": "G0/G2"},
        "09_obstacle_aware_pick_drill": {
            "status": "blocked",
            "binding_constraint": "base_tilt_guard",
            "measured": "obstacle approach tilted 90.9-125.1 deg and the base "
                        "ran away; no standoff number",
            "gap": "G0/G9"},
    }
    standoff_out = {
        "definition": "standoff = distance from the base origin to the task "
                      "object body-tree AABB nearest point; the root distance "
                      "is reported too",
        "task_05_drill_lift": {
            "status": "measured",
            "target_params": {"gap": 0.155, "z_off": 0.45, "x_off": 0.0},
            "rows": fine_rows,
            "minimum_reachable": best05,
            "note": "05 root-distance 0.233 m matches the prior ~0.25 m; the "
                    "AABB standoff at that pose is 0.139 m.  Pad error 0.0059, "
                    "joint margin 0.1719, tilt 3.66 deg.",
        },
        "other_tasks": blocked,
        "gap": "G0: the per-task standoff envelope requires the W1 "
               "base_navigate_to_standoff servo and task-specific targets. "
               "The hand-rolled move_base approach is not a calibrated "
               "standoff servo (tilt 20-125 deg on 01/04/09; it cannot hold "
               "the 15 mm pad gate on the others).",
    }

    # ---- IK dimension A/B ----
    def executed(rec):
        pr = rec.get("palm_residual") or {}
        return {
            "task_dim": rec.get("task_dim"),
            "waist_owner": rec.get("waist_owner"),
            "waist_solved": rec.get("waist_solved"),
            "waist_commandable": rec.get("waist_commandable"),
            "solver": rec.get("solver"),
            "palm_residual": pr,
            "worst_palm_error": (None if not pr else round(
                max(norm(v) for v in pr.values()), 4)),
            "joint_margin_min": rec.get("joint_margin_min"),
            "ik_err": rec.get("ik_err"),
            "ik_rot": rec.get("ik_rot"),
        }

    ab_out = {
        "targets": {"left": [1.731, 0.182, 1.25],
                    "right": [1.731, -0.128, 1.25]},
        "read_only_homie": cmp_h["rows"],
        "read_only_ik": cmp_i["rows"],
        "executed": {
            "12d_homie": executed(q12),
            "17d_ik": executed(q17),
            "12d_restore_homie": executed(q12b),
        },
        "handover": {
            "switch_to_ik": auth_ik,
            "restore_homie": auth_restore,
            "restore_tilt_deg": state_restore.get("base_tilt_deg"),
        },
        "solve_times_s": {
            "12d": cmp_h["rows"][0]["solve_s"],
            "14d": cmp_h["rows"][1]["solve_s"],
            "17d": cmp_i["rows"][1]["solve_s"],
        },
        "adopted": {
            "task_dim": 12,
            "waist_owner": "homie",
            "reason": "12-D is the smallest candidate that meets the 15 mm "
                      "pad and 50 mrad margin gates (executed worst side "
                      "0.0146 m, margin 0.1559).  14-D is dominated: same "
                      "read-only residual (0.00013 m) for 0.2447 s vs "
                      "0.0151 s and a (0,0) base search.  17-D under ik is "
                      "slightly better executed (0.0109 m) but requires the "
                      "authorized handover and is not the smallest; it is a "
                      "measured alternative.",
        },
    }

    # ---- grasp proxy ----
    proxy_out = {
        "proxy_definition": "lift-phase bilateral hand-object contact "
                            "(left>0 and right>0) and object rise > 0.02 m",
        "conditions": proxy["conditions"],
        "episodes": proxy["episodes"],
        "confusion": proxy["confusion"],
        "benchmark_outcome": "full 05_drill_lift task success; the exact "
                             "IsGrasped sub-condition is not exposed "
                             "(success_detail only has 'task')",
        "failure_boundary": "The 3 false positives are the low-lift "
                            "condition (object rise 0.045-0.081 m < the task's "
                            "0.1 m threshold): the proxy is a grasp "
                            "indicator, not the task predicate.  Close-phase "
                            "bilateral contact is not sufficient (0/0 on "
                            "successful good seeds 0 and 3); the lift phase is "
                            "the reliable read.  One-sided/swept-object "
                            "episodes (good seed 1 rise 0.185 m; miss seed 2 "
                            "rise 0.155 m) are correctly not called grasps.  "
                            "The lift L2 must add the rise/support terms.",
        "gap": "G1",
    }

    out = {
        "experiment": "ll_v3",
        "round": 0,
        "kind": "calibration",
        "task_for_joint_measurements": gain["task"],
        "seed": gain["seed"],
        "date": "2026-11-16",
        "authority_gain": {
            "source": "docs/joint_authority.json for final_writer; live "
                      "track_gain for measured gain",
            "delta_rad": 0.2, "dur_s": 1.5, "hold_s": 1.0,
            "min_gain_for_commands": 0.8,
            "robot_authority": robot_auth,
            "measured": measured,
            "waist_handover": gain["waist_handover"],
            "summary": {
                "n_measured": len(measured),
                "n_measured_ge_0p8": sum(
                    1 for r in measured.values()
                    if r["measured_gain_homie"] is not None
                    and abs(r["measured_gain_homie"]) >= 0.8),
                "waist_gains_homie": {
                    nm: measured[nm]["measured_gain_homie"]
                    for nm in ("waist_yaw_joint", "waist_roll_joint",
                               "waist_pitch_joint")},
                "waist_gains_ik": {
                    nm: measured[nm]["measured_gain_ik"]
                    for nm in ("waist_yaw_joint", "waist_roll_joint",
                               "waist_pitch_joint")},
                "restore_ok": gain["waist_handover"]["restore_ok"],
                "balance_ok": gain["waist_handover"]["balance_ok"],
            },
        },
        "ramp_lag": ramp_out,
        "reach_envelope": reach_out,
        "standoff_envelope": standoff_out,
        "ik_dimension_ab": ab_out,
        "grasp_proxy": proxy_out,
        "adopted": {
            "ik_task_dim": 12,
            "waist_owner": "homie",
            "standoff_05": (None if best05 is None else {
                "root_standoff_m": best05["achieved_root_standoff"],
                "aabb_standoff_m": best05.get("achieved_aabb_standoff"),
                "pad_error_m": best05["worst_pad_error"],
                "joint_margin_rad": best05["joint_margin_min"],
            }),
            "ramp_s_by_delta": recommended,
            "grasp_proxy_boundary": proxy_out["failure_boundary"],
        },
        "gaps": [
            {"id": "G0", "title": "per-task standoff envelope blocked",
             "evidence": "hand-rolled move_base approach tilts 20-125 deg on "
                         "01/04/09 and cannot hold the 15 mm gate on the "
                         "others; only 05 is measured",
             "design_change": "run W1 base_navigate_to_standoff before the "
                              "per-task envelope, or defer the envelope to W1"},
            {"id": "G1", "title": "exact IsGrasped not exposed",
             "evidence": "success_detail only has 'task'; proxy vs full task "
                         "success = 4 TP / 3 FP / 0 FN / 5 TN, FP = low lift",
             "design_change": "grasp_state_tracking is a grasp indicator; "
                              "lift_object/task programs add the rise/support "
                              "terms; never read or imitate the predicate"},
            {"id": "G2", "title": "task-specific targets",
             "evidence": "02 button vs panel centre; 03/07 box centre; 08 "
                         "holder drill; 04 shelf side; generic target invalid",
             "design_change": "the target belongs to the task strategy / "
                              "press-push-place skill, not the generic "
                              "standoff calibration"},
            {"id": "G3", "title": "14-D dominated / executed path blocked",
             "evidence": "14-D same read-only residual 0.00013 m, 0.2447 s vs "
                         "0.0151 s, base search (0,0); executed 14-D needs the "
                         "base-placement servo",
             "design_change": "adopt 12-D; retire the 14-D candidate with this "
                              "measurement"},
            {"id": "G4", "title": "17-D alternative",
             "evidence": "17-D under ik executed worst 0.0109 m vs 12-D "
                         "0.0146 m; both meet 15 mm; 17-D requires handover",
             "design_change": "keep 12-D as the adopted smallest; record 17-D "
                              "as a measured alternative, not the default"},
            {"id": "G5", "title": "DLS preview overstates the QP residual",
             "evidence": "ik_preview DLS 0.017-0.208 m vs QP executed "
                         "0.008-0.015 m",
             "design_change": "use ik_preview only as a reachability screen; "
                              "execute with the mink QP"},
            {"id": "G6", "title": "ramp curve shorter than the fixed ramp",
             "evidence": "step t95 0.10-0.32 s vs the 1.5 s _ramp used by "
                         "track_gain",
             "design_change": "use ramp_s(delta) = 0.15/0.15/0.15/0.35 s "
                              "instead of a constant ramp"},
            {"id": "G7", "title": "01 floor crouch unstable",
             "evidence": "set_height(0.46) + approach tilt 19.8-29.4 deg; pad "
                         "error 0.98-1.54",
             "design_change": "01 requires base_height_setpoint + a stable "
                              "crouch/approach servo; defer the floor "
                              "standoff to W1"},
            {"id": "G8", "title": "04 shelf push is a contact problem",
             "evidence": "16 kg shelf pushed; approach tilt 20.7-33.0 deg",
             "design_change": "the push standoff is a contact distance "
                              "measured by push_object_to_target, not a "
                              "grasp standoff"},
            {"id": "G9", "title": "09 obstacle approach unstable",
             "evidence": "approach tilt 90.9-125.1 deg; base ran away",
             "design_change": "09 requires collision_free_reach_trajectory / "
                              "an obstacle-aware standoff servo"},
            {"id": "G10", "title": "lift-phase joint margin at 0.15 m lift",
             "evidence": "good seed 0 reach_lift joint margin 0.0406 rad "
                         "(below 0.05) at a 0.15 m lift",
             "design_change": "cap the transport lift or add a posture "
                              "adjustment; the task's 0.1 m lift is safer"},
        ],
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print("wrote", OUT)
    print("n measured", len(measured), "n>=0.8",
          out["authority_gain"]["summary"]["n_measured_ge_0p8"])
    print("ramp", recommended)
    print("05 best", best05)
    print("proxy confusion", proxy["confusion"])


if __name__ == "__main__":
    main()
