#!/usr/bin/env python3
"""Register a validated skill in the ll_v2 graph and record its validation.

Usage:
  python scripts/ll_v2_register_skill.py \
      --capability ee_pose_ik --name arm_pose_ik \
      --source work/ll_v2/substrate/arm_pose_ik.py \
      --verdict accept --scope "05 seeds 0-3; 03 seed 0" \
      --metrics '{"pad_err_max_m":0.011}' \
      --evidence work/ll_v2/substrate/arm_pose_ik.report.md \
      --contract '{"commands":[...],"observed_exogenous":[...]}' \
      --kwargs '{}'
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(ROOT, "graph_cli.py")
DB = "experiments/ll_v2/skill_system.db"


def call(*args):
    p = subprocess.run([sys.executable, CLI, "--db", DB] + [str(a) for a in args],
                       cwd=ROOT, capture_output=True, text=True)
    try:
        out = json.loads(p.stdout)
    except ValueError:
        raise SystemExit("graph_cli failed: %s\n%s" % (p.stdout, p.stderr))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capability", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--source", required=True)
    ap.add_argument("--verdict", required=True,
                    choices=["accept", "restrict", "reject", "retire"])
    ap.add_argument("--scope", default="")
    ap.add_argument("--metrics", default="{}")
    ap.add_argument("--evidence", default="")
    ap.add_argument("--contract", default="{}")
    ap.add_argument("--kwargs", default="{}")
    ap.add_argument("--description", default="")
    ap.add_argument("--version", default="v1")
    args = ap.parse_args()

    src = os.path.join(ROOT, args.source)
    h = hashlib.sha256(open(src, "rb").read()).hexdigest()

    # link_skill is idempotent by name (it errors on an existing name); reuse
    # the existing node when the same skill is re-registered after a rerun.
    import sqlite3
    conn = sqlite3.connect(os.path.join(ROOT, DB))
    row = conn.execute(
        "SELECT id FROM nodes WHERE type='SKILL' AND name=?", (args.name,)
    ).fetchone()
    conn.close()
    existing = None if row is None else {"id": row[0]}
    linked = None
    if existing is None:
        r = call("link_skill", "--capability", args.capability,
                 "--name", args.name, "--description", args.description,
                 "--code-path", args.source, "--source-hash", h,
                 "--kwargs", args.kwargs, "--contract", args.contract,
                 "--version", args.version)
        if not r.get("ok"):
            raise SystemExit("link_skill failed: %s" % r)
        linked = r["result"]["id"]
        sid = linked
    else:
        sid = existing["id"]

    r = call("submit_validation", "--skill", sid, "--verdict", args.verdict,
             "--metrics", args.metrics, "--scope", args.scope,
             "--evidence", args.evidence, "--version", args.version)
    if not r.get("ok"):
        raise SystemExit("submit_validation failed: %s" % r)
    print(json.dumps({"ok": True, "skill_id": sid, "name": args.name,
                      "source_hash": h, "verdict": args.verdict,
                      "status": r["result"]["status"]}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
