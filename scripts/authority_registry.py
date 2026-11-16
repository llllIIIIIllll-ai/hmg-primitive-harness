#!/usr/bin/env python3
"""Generate the Joint Authority Registry for the G1 (43 DoF).

Authority is a STRUCTURAL property: which controller writes the final joint
command last.  In HumanoidMimicGen, G1DecoupledWholeBodyPolicy.get_action
writes q[upper_body_indices] first and q[lower_body_indices] afterwards, so a
joint present in both groups is finally written by the lower-body (HOMIE v2)
policy.  With enable_waist=True the waist appears in both groups, so its final
writer stays HOMIE by default.

The user authorized a RUNTIME handover for the waist (2026-11-15, recorded in
experiments/ll_v2/config.yaml control.waist_owner_modes): `api.authority` may
set waist_owner to `homie` (default), `ik` (upper_target) or `blend`; the
default owner is restored on exit.  `homie` means the waist is
observed_exogenous (read, never command); `ik`/`blend` make it a command target.
The handover is measured in `authority_handover` (see gate_status).  A high
tracking gain still never confers authority: final_writer decides it.

Tracking gain is a DYNAMIC property measured at runtime (`track_gain` in
hmg_tools.py); it is stored separately and never used to infer authority.
Regenerating this file PRESERVES the measured fields and the `measurement`
block already present in an existing output file.

Usage:
  python scripts/authority_registry.py --out docs/joint_authority.json
"""
import argparse
import json
import os

MEASURED_FIELDS = (
    "measured_gain",
    "measured_gain_channel",
    "measured_gain_direction",
    "measured_gain_pos",
    "measured_gain_neg",
    "measured_gain_note",
    "effective_bounds",
)

WAIST_JOINTS = ["waist_pitch_joint", "waist_roll_joint", "waist_yaw_joint"]

# Measured gain handover / re-handover, track_gain(delta=0.1, dur=1.5,
# hold=1.0) with 2 s pre-settle, 05_drill_lift seeds 0-3 (bit-identical across
# seeds).  Source: work/ll_v2/reports/gate_status.md (W1 authority_handover).
WAIST_HANDOVER = {
    "waist_pitch_joint": {
        "homie": 0.137, "ik": 1.000, "rehandover": 0.139,
        "restore_error_rad": 0.002,
    },
    "waist_roll_joint": {
        "homie": -0.026, "ik": 0.998, "rehandover": -0.062,
        "restore_error_rad": 0.036,
    },
    "waist_yaw_joint": {
        "homie": 1.137, "ik": 1.000, "rehandover": 1.135,
        "restore_error_rad": 0.002,
    },
}

WAIST_OWNER_WRITERS = {
    "homie": "homie_v2",
    "ik": "upper_target",
    "blend": "blend",
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="docs/joint_authority.json")
    p.add_argument("--preserve-from", default="docs/joint_authority.json",
                   help="existing registry whose measured fields and "
                        "measurement block are carried over")
    args = p.parse_args()

    old = {}
    old_joints = {}
    seed_path = args.preserve_from
    if not os.path.exists(seed_path) and os.path.exists(args.out):
        seed_path = args.out
    if os.path.exists(seed_path):
        with open(seed_path, encoding="utf-8") as fh:
            old = json.load(fh)
        for rec in old.get("joints") or []:
            old_joints[rec.get("joint")] = rec

    from humanoidmimicgen.wbc.robot_model.instantiation import (
        get_robot_type_and_model,
    )

    _, rm0 = get_robot_type_and_model("g1", enable_waist_ik=False)
    _, rm1 = get_robot_type_and_model("g1", enable_waist_ik=True)
    names = list(rm0.joint_names)

    def group_set(rm, group):
        return {names[i] for i in rm.get_joint_group_indices(group)}

    joints = []
    for nm in names:
        up0 = nm in group_set(rm0, "upper_body")
        low0 = nm in group_set(rm0, "lower_body")
        up1 = nm in group_set(rm1, "upper_body")
        low1 = nm in group_set(rm1, "lower_body")
        # lower_body is written after upper_body
        writer0 = "homie_v2" if low0 else ("upper_target" if up0 else "none")
        writer1 = "homie_v2" if low1 else ("upper_target" if up1 else "none")
        rec = {
            "joint": nm,
            "groups_enable_waist_false": {
                "upper_body": up0, "lower_body": low0,
                "final_writer": writer0,
            },
            "groups_enable_waist_true": {
                "upper_body": up1, "lower_body": low1,
                "final_writer": writer1,
            },
        }
        prev = old_joints.get(nm) or {}
        for f in MEASURED_FIELDS:
            if f in prev:
                rec[f] = prev[f]
            else:
                rec.setdefault(f, None)
        if nm in WAIST_JOINTS:
            rec["final_writer_owners"] = dict(WAIST_OWNER_WRITERS)
            rec["measured_gain_handover"] = {
                "homie": WAIST_HANDOVER[nm]["homie"],
                "ik": WAIST_HANDOVER[nm]["ik"],
                "rehandover": WAIST_HANDOVER[nm]["rehandover"],
            }
        joints.append(rec)

    waist_handover = {
        "channel": "waist",
        "joints": list(WAIST_JOINTS),
        "default_owner": "homie",
        "modes": dict(WAIST_OWNER_WRITERS),
        "authorized": "user-authorized 2026-11-15; recorded in "
                      "experiments/ll_v2/config.yaml "
                      "control.waist_owner_modes",
        "verified_by": "authority_handover",
        "restore_default_on_exit": True,
        "gain_probe": "track_gain(delta=0.1, dur=1.5, hold=1.0), 2 s "
                      "pre-settle, 05_drill_lift seeds 0-3 (bit-identical)",
        "measured_gain": {
            "homie_before": {j: WAIST_HANDOVER[j]["homie"]
                             for j in WAIST_JOINTS},
            "ik_after": {j: WAIST_HANDOVER[j]["ik"] for j in WAIST_JOINTS},
            "homie_again": {j: WAIST_HANDOVER[j]["rehandover"]
                            for j in WAIST_JOINTS},
        },
        "restore_error_rad": {j: WAIST_HANDOVER[j]["restore_error_rad"]
                              for j in WAIST_JOINTS},
        "restore_error_tol_rad": 0.05,
        "null_control_noise_floor": 0.020,
        "pitch_rise": {"measured": 0.863, "min_required": 0.5},
        "note": "A high measured gain never confers authority: authority is "
                "the structural final_writer. With owner=homie the waist is "
                "observed_exogenous: read every control step, never command "
                "it. Under owner=ik (upper_target) or blend the waist is a "
                "command target; verify achieved gain before and after and "
                "restore homie on exit.",
    }

    out = {
        "robot": "g1",
        "dof": len(names),
        "authority_rule": "Default: G1DecoupledWholeBodyPolicy.get_action "
                          "writes upper_body first, lower_body last, so a "
                          "joint in both groups is finally written by HOMIE "
                          "v2 (lower wins). A user-authorized runtime "
                          "handover (api.authority) may change the waist's "
                          "final writer to upper_target (owner=ik) or blend; "
                          "the default owner is homie and is restored on exit.",
        "gain_note": "measured_gain is dynamic, filled by track_gain; it never "
                     "confers authority. final_writer decides authority. With "
                     "owner=homie the waist is observed_exogenous (read, never "
                     "command).",
        "joints": joints,
        "waist_handover": waist_handover,
    }
    if "measurement" in old:
        out["measurement"] = old["measurement"]
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("wrote %s (%d joints)" % (args.out, len(joints)))


if __name__ == "__main__":
    main()
