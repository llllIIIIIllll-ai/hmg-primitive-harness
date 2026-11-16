#!/usr/bin/env python3
"""Build the shared proposal packet for one ll_v2 batch.

The EXPLORE and EXPLOIT proposal actors receive the SAME packet: mode/batch,
coverage, verified skills, measured failures, calibration summary and the
resource limits.  No full DB, no history dump.

Usage:
  python scripts/ll_v2_packet.py --batch 1 --out work/ll_v2/packets/batch_1.md \
      [--notes work/ll_v2/reports/batch_1_notes.md] \
      [--sweep work/ll_v2/reports/sweep_0.json]
"""
import argparse
import json
import os
import sqlite3
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(ROOT, "graph_cli.py")
DB = os.path.join(ROOT, "experiments/ll_v2/skill_system.db")


def cov():
    p = subprocess.run([sys.executable, CLI, "--db", DB, "coverage"],
                       cwd=ROOT, capture_output=True, text=True)
    return json.loads(p.stdout)["result"]


def verified_skills():
    conn = sqlite3.connect(DB)
    rows = conn.execute(
        "SELECT n.name, n.status, n.spec_json, c.name AS cap "
        "FROM nodes n "
        "JOIN edges e ON e.source=n.id AND e.relation='IMPLEMENTS' "
        "JOIN nodes c ON c.id=e.target "
        "WHERE n.type='SKILL' AND n.status='verified' ORDER BY c.name").fetchall()
    out = []
    for name, status, spec, cap in rows:
        spec = json.loads(spec or "{}")
        vrow = conn.execute(
            "SELECT verdict, seed_domain, metrics_json FROM validations "
            "WHERE skill_id=(SELECT id FROM nodes WHERE type='SKILL' AND name=?) "
            "ORDER BY created_at DESC LIMIT 1", (name,)).fetchone()
        out.append({"capability": cap, "skill": name,
                    "scope": "" if vrow is None else vrow[1],
                    "metrics": {} if vrow is None else json.loads(vrow[2] or "{}"),
                    "source": spec.get("code_path")})
    conn.close()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--notes", default="")
    ap.add_argument("--sweep", default="")
    ap.add_argument("--extra", default="")
    args = ap.parse_args()

    c = cov()
    cal = json.load(open(os.path.join(ROOT, "experiments/ll_v2/calibration.json")))
    lines = []

    def add(s=""):
        lines.append(s)

    add("# Proposal packet — ll_v2 batch %d" % args.batch)
    add("")
    add("Mode: `warmup`. Max batches: 8. The warm-up substrate primitives are "
        "verified (W1 8/8). The W2 integration gate is a BLOCKING gate and is "
        "NOT closed (0/4): the batch counter stays 0 and no discovery batch "
        "starts until W2 is re-run and passes. The gate predicate in "
        "`experiments/ll_v2/config.yaml` is authoritative.")
    add("")
    add("## Uncovered capabilities (portfolio union)")
    add("")
    for u in c["uncovered_union"]:
        add("- `%s` (%s)" % (u["name"], u["level"]))
    add("")
    add("## Per-task coverage")
    add("")
    for t in c["tasks"]:
        add("- **%s** %d/%d; uncovered: %s" % (
            t["task"], t["covered"], t["total"],
            ", ".join(t["uncovered"]) or "none"))
    add("")
    add("## Verified skills")
    add("")
    vs = verified_skills()
    if not vs:
        add("- none yet")
    for s in vs:
        add("- `%s` → capability `%s`; scope: %s; metrics: %s" % (
            s["skill"], s["capability"], s["scope"] or "n/a",
            json.dumps(s["metrics"])[:300]))
    add("")
    add("## Calibration summary (measured)")
    add("")
    add("- Lower body 12 are WBC/HOMIE-owned. The waist 3 default to HOMIE v2 "
        "and are **observed_exogenous** while `waist_owner=homie`: read every "
        "control step, never command them. A user-authorized runtime handover "
        "(`api.authority(waist_owner='ik'|'blend')`) makes the waist a command "
        "target; verify the achieved gain before and after and restore `homie` "
        "on exit. Arms and hands are upper_target.")
    add("- Shoulder roll is commandable only OUTWARD: "
        "`{'left_shoulder_roll_joint': [-0.05, 2.252], "
        "'right_shoulder_roll_joint': [-2.252, 0.05]}`.")
    add("- Finger closing directions are one-sided (left index/middle negative, "
        "right index/middle positive).")
    ramp = cal.get("ramp_curve") or []
    add("- Ramp lag: ramp_s reaching 95%% = %s." % ", ".join(
        "%s rad → %s s" % (r["delta"], r["ramp_s_95"]) for r in ramp))
    add("- IK dimension: **12-D** adopted (two palm poses over the 14 arm "
        "joints, waist measured exogenous). 17-D (adds the three waist joints) "
        "is legal only under an authorized `ik`/`blend` handover. Measured on "
        "`05_drill_lift` seeds 0-3: 17-D/ik is authority-compliant but its "
        "EXECUTED pad error is worse than 12-D/homie on every seed "
        "(0.0119-0.1342 m vs 0.0091-0.0389 m), so selection is by executed "
        "pad/contract, not model residual. 17-D/homie is the "
        "authority-negative control.")
    add("- W1 substrate: COMPLETE (8/8), including `authority_handover` "
        "(measured waist-pitch gain 0.137 under homie -> 1.000 under ik, "
        "restored 0.139; all three joints in `docs/control_contract.md` §1b).")
    add("- W2 integration: **BLOCKED (0/4)** on `05_drill_lift` seeds 0-3 with "
        "12-D/homie (binding `pad_position_error` x3, `bilateral_contact` x1). "
        "The missing capability is `collision_free_reach_trajectory`; that work "
        "is a warm-up closure job (label W2b), not batch 1. W2 must be re-run "
        "and pass (pad <= 0.015 m/side, bilateral contact after close, rise "
        ">= 0.05 m, no guard trip) before batch 1 starts.")
    add("- Right-arm reach is the measured asymmetry; the substrate plans pad "
        "contact inside the usable set.")
    stand = cal.get("standoff") or {}
    add("- Standoff (object-body x − base x, measured, open-loop drive): " +
        "; ".join("%s %s m" % (t, ",".join(
            str(r.get("achieved_standoff")) for r in (v.get("rows") or [])))
            for t, v in sorted(stand.items())))
    add("")
    if args.notes and os.path.exists(os.path.join(ROOT, args.notes)):
        add("## Recent failures and open boundaries")
        add("")
        add(open(os.path.join(ROOT, args.notes), encoding="utf-8").read())
        add("")
    if args.sweep and os.path.exists(os.path.join(ROOT, args.sweep)):
        sw = json.load(open(os.path.join(ROOT, args.sweep)))
        add("## Previous evaluation sweep")
        add("")
        add("- total: %s/90" % sw.get("success_total"))
        add("- per task: " + "; ".join(
            "%s %s/10" % (t, v.get("success_count"))
            for t, v in sorted((sw.get("tasks") or {}).items())))
        add("")
    if args.extra and os.path.exists(os.path.join(ROOT, args.extra)):
        add(open(os.path.join(ROOT, args.extra), encoding="utf-8").read())
        add("")
    add("## Resource limits")
    add("")
    add("- `execution.servers_per_task: 2`, `max_parallel_servers: 18`. "
        "One slot per execution actor; never share a queue between actors.")
    add("- Allowed job types: DISCOVER_LOW_LEVEL, COMPOSE_MIDDLE_LEVEL, "
        "REFINE_SKILL, VALIDATE_GENERALIZATION, SOLVE_TASK, EXPLORE_ALTERNATIVE.")
    add("")
    add("## Output")
    add("")
    add("Return only the YAML from `prompts/proposal_actor.md`: `job_type`, "
        "`target_ids`, `hypothesis`, `proposed_experiment`, `expected_benefit`, "
        "`supporting_evidence`, `success_measure`, `main_uncertainty`.")

    path = os.path.join(ROOT, args.out)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w", encoding="utf-8").write("\n".join(lines))
    print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
