#!/usr/bin/env python3
"""Typed capability/skill graph for the bottom-up discovery experiment.

SQLite is the source of truth. Every command prints one JSON object.
Run `python graph_cli.py init` before the first use.
"""
import argparse
import datetime
import json
import os
import sqlite3
import sys
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DB = os.path.join(HERE, "experiments", "ll_v1", "skill_system.db")
SCHEMA = os.path.join(HERE, "schema.sql")

TASKS = [
    "01_box_lift_floor",
    "02_push_button",
    "03_box_lift",
    "04_push_shelf_forward",
    "05_drill_lift",
    "06_drill_pnp",
    "07_box_table_to_shelf",
    "08_pick_drill_from_holder",
    "09_obstacle_aware_pick_drill",
]


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def emit(obj):
    print(json.dumps(obj, ensure_ascii=False, indent=2))


def fail(msg):
    emit({"ok": False, "error": str(msg)})
    sys.exit(1)


def connect(path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys = ON")
    conn.execute("PRAGMA journal_mode = WAL")
    return conn


def init_db(conn):
    with open(SCHEMA, encoding="utf-8") as fh:
        conn.executescript(fh.read())
    for name in TASKS:
        row = conn.execute(
            "SELECT id FROM nodes WHERE type='TASK' AND name=?", (name,)
        ).fetchone()
        if row is None:
            conn.execute(
                "INSERT INTO nodes(id,type,name,description,spec_json,status,created_at)"
                " VALUES(?,?,?,?,?,?,?)",
                (str(uuid.uuid4()), "TASK", name, "", "{}", "active", now()),
            )
    conn.commit()


def checkpoints_dir(db):
    return os.path.join(os.path.dirname(os.path.abspath(db)), "checkpoints")


def ensure_initial(db):
    directory = checkpoints_dir(db)
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, "initial.db")
    if os.path.exists(path):
        return path
    tmp = sqlite3.connect(":memory:")
    init_db(tmp)
    dst = sqlite3.connect(path)
    tmp.backup(dst)
    dst.close()
    tmp.close()
    return path


def resolve(conn, ref, kind=None):
    if not ref:
        fail("empty reference")
    rows = conn.execute("SELECT * FROM nodes WHERE id=?", (ref,)).fetchall()
    if not rows:
        rows = conn.execute("SELECT * FROM nodes WHERE name=?", (ref,)).fetchall()
    if kind:
        rows = [r for r in rows if r["type"] == kind]
    if not rows:
        fail("not found: %r%s" % (ref, " (type %s)" % kind if kind else ""))
    if len(rows) > 1:
        fail("ambiguous reference %r: %s" % (ref, [r["id"] for r in rows]))
    return rows[0]


def node_dict(row):
    return {
        "id": row["id"],
        "type": row["type"],
        "name": row["name"],
        "description": row["description"],
        "spec": json.loads(row["spec_json"] or "{}"),
        "status": row["status"],
        "created_at": row["created_at"],
    }


def parse_json(value, default):
    if value is None or value == "":
        return default
    try:
        return json.loads(value)
    except Exception as exc:
        fail("invalid JSON: %s" % exc)


def cmd_init(conn, args):
    init_db(conn)
    initial = ensure_initial(args.db)
    emit({"ok": True, "result": {"db": args.db, "tasks": TASKS,
                                 "initial_checkpoint": initial}})


def cmd_add_task(conn, args):
    init_db(conn)
    row = conn.execute(
        "SELECT id FROM nodes WHERE type='TASK' AND name=?", (args.name,)
    ).fetchone()
    if row:
        emit({"ok": True, "result": {"id": row["id"], "duplicate": True}})
        return
    nid = str(uuid.uuid4())
    conn.execute(
        "INSERT INTO nodes(id,type,name,description,spec_json,status,created_at)"
        " VALUES(?,?,?,?,?,?,?)",
        (nid, "TASK", args.name, args.description, "{}", "active", now()),
    )
    conn.commit()
    emit({"ok": True, "result": {"id": nid}})


def cmd_search_capabilities(conn, args):
    q = "%" + args.query + "%"
    rows = conn.execute(
        "SELECT * FROM nodes WHERE type='CAPABILITY' AND (name LIKE ? OR description LIKE ?)"
        " ORDER BY created_at",
        (q, q),
    ).fetchall()
    emit({"ok": True, "result": [node_dict(r) for r in rows]})


def cmd_get_task_strategies(conn, args):
    task = resolve(conn, args.task, "TASK")
    srows = conn.execute(
        "SELECT n.* FROM nodes n JOIN edges e ON e.target=n.id"
        " WHERE e.source=? AND e.relation='HAS_STRATEGY'",
        (task["id"],),
    ).fetchall()
    out = []
    for s in srows:
        caps = conn.execute(
            "SELECT n.* FROM nodes n JOIN edges e ON e.target=n.id"
            " WHERE e.source=? AND e.relation='REQUIRES'",
            (s["id"],),
        ).fetchall()
        out.append({"strategy": node_dict(s), "capabilities": [node_dict(c) for c in caps]})
    emit({"ok": True, "result": {"task": node_dict(task), "strategies": out}})


def cmd_get_missing_capabilities(conn, args):
    strategy = resolve(conn, args.strategy, "STRATEGY")
    caps = conn.execute(
        "SELECT n.* FROM nodes n JOIN edges e ON e.target=n.id"
        " WHERE e.source=? AND e.relation='REQUIRES'",
        (strategy["id"],),
    ).fetchall()
    missing = []
    for cap in caps:
        verified = conn.execute(
            "SELECT s.id FROM nodes s JOIN edges e ON e.source=s.id"
            " WHERE e.target=? AND e.relation='IMPLEMENTS' AND s.type='SKILL'"
            " AND s.status='verified'",
            (cap["id"],),
        ).fetchall()
        if not verified:
            missing.append(node_dict(cap))
    emit({"ok": True, "result": {"strategy": node_dict(strategy), "missing": missing}})


def cmd_propose_capability(conn, args):
    init_db(conn)
    row = conn.execute(
        "SELECT * FROM nodes WHERE type='CAPABILITY' AND name=?", (args.name,)
    ).fetchone()
    if row:
        emit({"ok": True, "result": {"id": row["id"], "duplicate": True}})
        return
    spec = parse_json(args.spec, {})
    if args.level:
        spec["level"] = args.level
    nid = str(uuid.uuid4())
    conn.execute(
        "INSERT INTO nodes(id,type,name,description,spec_json,status,created_at)"
        " VALUES(?,?,?,?,?,?,?)",
        (nid, "CAPABILITY", args.name, args.description, json.dumps(spec), "proposed", now()),
    )
    conn.commit()
    emit({"ok": True, "result": {"id": nid, "spec": spec}})


def cmd_propose_strategy(conn, args):
    task = resolve(conn, args.task, "TASK")
    cap_ids = [x.strip() for x in args.capabilities.split(",") if x.strip()]
    caps = [resolve(conn, c, "CAPABILITY") for c in cap_ids]
    nid = str(uuid.uuid4())
    conn.execute(
        "INSERT INTO nodes(id,type,name,description,spec_json,status,created_at)"
        " VALUES(?,?,?,?,?,?,?)",
        (nid, "STRATEGY", args.name, args.description, "{}", "proposed", now()),
    )
    conn.execute(
        "INSERT INTO edges(source,target,relation) VALUES(?,?,?)",
        (task["id"], nid, "HAS_STRATEGY"),
    )
    for cap in caps:
        conn.execute(
            "INSERT OR IGNORE INTO edges(source,target,relation) VALUES(?,?,?)",
            (nid, cap["id"], "REQUIRES"),
        )
    conn.commit()
    emit({"ok": True, "result": {"id": nid, "task": task["id"],
                                 "capabilities": [c["id"] for c in caps]}})


def cmd_link_skill(conn, args):
    cap = resolve(conn, args.capability, "CAPABILITY")
    existing = conn.execute(
        "SELECT * FROM nodes WHERE type='SKILL' AND name=?", (args.name,)
    ).fetchone()
    if existing:
        fail("skill name already exists: %s" % args.name)
    sid = str(uuid.uuid4())
    kwargs = parse_json(args.kwargs, {})
    contract = parse_json(args.contract, {})
    conn.execute(
        "INSERT INTO nodes(id,type,name,description,spec_json,status,created_at)"
        " VALUES(?,?,?,?,?,?,?)",
        (sid, "SKILL", args.name, args.description,
         json.dumps({"contract": contract, "code_path": args.code_path,
                     "source_hash": args.source_hash, "kwargs": kwargs}),
         "candidate", now()),
    )
    conn.execute(
        "INSERT INTO edges(source,target,relation) VALUES(?,?,?)",
        (sid, cap["id"], "IMPLEMENTS"),
    )
    conn.execute(
        "INSERT INTO skill_versions(id,capability_ids,version,code_path,source_hash,"
        "kwargs_json,contract_json,status,created_at) VALUES(?,?,?,?,?,?,?,?,?)",
        (sid, json.dumps([cap["id"]]), args.version, args.code_path,
         args.source_hash, json.dumps(kwargs), json.dumps(contract), "candidate", now()),
    )
    conn.commit()
    emit({"ok": True, "result": {"id": sid, "capability": cap["id"]}})


def cmd_request_discovery(conn, args):
    cap = resolve(conn, args.capability, "CAPABILITY")
    eid = str(uuid.uuid4())
    conn.execute(
        "INSERT INTO experiments(id,job_type,capability_id,actor_id,budget,"
        "measured_cost,result_json,created_at) VALUES(?,?,?,?,?,?,?,?)",
        (eid, args.job_type, cap["id"], args.actor_id, args.budget, 0.0, "{}", now()),
    )
    conn.commit()
    emit({"ok": True, "result": {"id": eid, "capability": cap["id"]}})


def cmd_submit_validation(conn, args):
    skill = resolve(conn, args.skill, "SKILL")
    verdict = args.verdict
    if verdict not in ("accept", "restrict", "reject", "retire"):
        fail("verdict must be accept|restrict|reject|retire")
    metrics = parse_json(args.metrics, {})
    vid = str(uuid.uuid4())
    conn.execute(
        "INSERT INTO validations(id,skill_id,version,scenario_set,seed_domain,"
        "metrics_json,evidence_path,verdict,created_at) VALUES(?,?,?,?,?,?,?,?,?)",
        (vid, skill["id"], args.version, args.scenario_set, args.seed_domain,
         json.dumps(metrics), args.evidence, verdict, now()),
    )
    status = "verified" if verdict in ("accept", "restrict") else (
        "retired" if verdict == "retire" else "rejected"
    )
    conn.execute("UPDATE nodes SET status=? WHERE id=?", (status, skill["id"]))
    conn.execute("UPDATE skill_versions SET status=? WHERE id=?", (status, skill["id"]))
    conn.commit()
    emit({"ok": True, "result": {"id": vid, "skill": skill["id"], "status": status}})


def cmd_retire_skill(conn, args):
    skill = resolve(conn, args.skill, "SKILL")
    spec = json.loads(skill["spec_json"] or "{}")
    spec["retire_reason"] = args.reason
    conn.execute("UPDATE nodes SET status='retired', spec_json=? WHERE id=?",
                 (json.dumps(spec), skill["id"]))
    conn.execute("UPDATE skill_versions SET status='retired' WHERE id=?", (skill["id"],))
    conn.commit()
    emit({"ok": True, "result": {"id": skill["id"], "status": "retired"}})


def cmd_merge_capabilities(conn, args):
    keep = resolve(conn, args.keep, "CAPABILITY")
    merged = [resolve(conn, x, "CAPABILITY") for x in args.merge.split(",") if x.strip()]
    for m in merged:
        if m["id"] == keep["id"]:
            continue
        conn.execute("UPDATE OR IGNORE edges SET target=? WHERE target=?",
                     (keep["id"], m["id"]))
        conn.execute("UPDATE OR IGNORE edges SET source=? WHERE source=?",
                     (keep["id"], m["id"]))
        conn.execute("DELETE FROM edges WHERE source=? OR target=?", (m["id"], m["id"]))
        conn.execute("UPDATE nodes SET status='retired' WHERE id=?", (m["id"],))
    conn.commit()
    emit({"ok": True, "result": {"keep": keep["id"],
                                 "merged": [m["id"] for m in merged]}})


def cmd_checkpoint(conn, args):
    directory = checkpoints_dir(args.db)
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, args.name + ".db")
    dst = sqlite3.connect(path)
    conn.backup(dst)
    dst.close()
    emit({"ok": True, "result": {"checkpoint": path}})


def cmd_restore(conn, args):
    path = os.path.join(checkpoints_dir(args.db), args.name + ".db")
    if not os.path.exists(path):
        fail("checkpoint not found: %s" % path)
    src = sqlite3.connect(path)
    src.backup(conn)
    src.close()
    conn.commit()
    emit({"ok": True, "result": {"restored": path}})


def cmd_reset(conn, args):
    path = ensure_initial(args.db) if args.name == "initial" else os.path.join(
        checkpoints_dir(args.db), args.name + ".db")
    if not os.path.exists(path):
        fail("checkpoint not found: %s" % path)
    src = sqlite3.connect(path)
    src.backup(conn)
    src.close()
    conn.execute("INSERT OR REPLACE INTO state(key,value) VALUES('round','0')")
    conn.commit()
    emit({"ok": True, "result": {"reset_to": path, "round": 0}})


def cmd_round_get(conn, args):
    init_db(conn)
    row = conn.execute("SELECT value FROM state WHERE key='round'").fetchone()
    emit({"ok": True, "result": {"round": int(row["value"]) if row else 0}})


def cmd_round_inc(conn, args):
    init_db(conn)
    row = conn.execute("SELECT value FROM state WHERE key='round'").fetchone()
    n = (int(row["value"]) if row else 0) + 1
    conn.execute("INSERT OR REPLACE INTO state(key,value) VALUES('round',?)", (str(n),))
    conn.commit()
    emit({"ok": True, "result": {"round": n}})


def cmd_list(conn, args):
    init_db(conn)
    counts = {}
    for row in conn.execute("SELECT type,status,COUNT(*) c FROM nodes GROUP BY type,status"):
        counts.setdefault(row["type"], {})[row["status"]] = row["c"]
    recent = conn.execute(
        "SELECT id,type,name,status FROM nodes ORDER BY created_at DESC LIMIT 20"
    ).fetchall()
    emit({"ok": True, "result": {"counts": counts,
                                 "recent": [dict(r) for r in recent]}})


def cmd_export(conn, args):
    init_db(conn)
    out = {}
    for table in ("nodes", "edges", "skill_versions", "validations",
                  "experiments", "execution_plans", "state"):
        out[table] = [dict(r) for r in conn.execute("SELECT * FROM %s" % table)]
    emit({"ok": True, "result": out})


def build_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--db", default=DEFAULT_DB)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("init"); s.set_defaults(fn=cmd_init)
    s = sub.add_parser("add_task"); s.add_argument("--name", required=True)
    s.add_argument("--description", default=""); s.set_defaults(fn=cmd_add_task)

    s = sub.add_parser("search_capabilities"); s.add_argument("--query", required=True)
    s.set_defaults(fn=cmd_search_capabilities)

    s = sub.add_parser("get_task_strategies"); s.add_argument("--task", required=True)
    s.set_defaults(fn=cmd_get_task_strategies)

    s = sub.add_parser("get_missing_capabilities"); s.add_argument("--strategy", required=True)
    s.set_defaults(fn=cmd_get_missing_capabilities)

    s = sub.add_parser("propose_capability")
    s.add_argument("--name", required=True); s.add_argument("--description", default="")
    s.add_argument("--spec", default="{}"); s.add_argument("--level", default="")
    s.set_defaults(fn=cmd_propose_capability)

    s = sub.add_parser("propose_strategy")
    s.add_argument("--task", required=True); s.add_argument("--name", required=True)
    s.add_argument("--description", default=""); s.add_argument("--capabilities", required=True)
    s.set_defaults(fn=cmd_propose_strategy)

    s = sub.add_parser("link_skill")
    s.add_argument("--capability", required=True); s.add_argument("--name", required=True)
    s.add_argument("--description", default=""); s.add_argument("--code-path", required=True)
    s.add_argument("--source-hash", required=True); s.add_argument("--kwargs", default="{}")
    s.add_argument("--contract", default="{}"); s.add_argument("--version", default="v1")
    s.set_defaults(fn=cmd_link_skill)

    s = sub.add_parser("request_discovery")
    s.add_argument("--capability", required=True); s.add_argument("--job-type", default="DISCOVER_LOW_LEVEL")
    s.add_argument("--budget", type=float, default=0.0); s.add_argument("--actor-id", default="")
    s.set_defaults(fn=cmd_request_discovery)

    s = sub.add_parser("submit_validation")
    s.add_argument("--skill", required=True); s.add_argument("--verdict", required=True)
    s.add_argument("--metrics", default="{}"); s.add_argument("--scope", default="")
    s.add_argument("--evidence", default=""); s.add_argument("--scenario-set", default="")
    s.add_argument("--seed-domain", default=""); s.add_argument("--version", default="v1")
    s.set_defaults(fn=cmd_submit_validation)

    s = sub.add_parser("retire_skill")
    s.add_argument("--skill", required=True); s.add_argument("--reason", required=True)
    s.set_defaults(fn=cmd_retire_skill)

    s = sub.add_parser("merge_capabilities")
    s.add_argument("--keep", required=True); s.add_argument("--merge", required=True)
    s.set_defaults(fn=cmd_merge_capabilities)

    s = sub.add_parser("checkpoint"); s.add_argument("--name", required=True)
    s.set_defaults(fn=cmd_checkpoint)

    s = sub.add_parser("restore"); s.add_argument("--name", required=True)
    s.set_defaults(fn=cmd_restore)

    s = sub.add_parser("reset"); s.add_argument("--name", default="initial")
    s.set_defaults(fn=cmd_reset)

    s = sub.add_parser("round_get"); s.set_defaults(fn=cmd_round_get)
    s = sub.add_parser("round_inc"); s.set_defaults(fn=cmd_round_inc)
    s = sub.add_parser("list"); s.set_defaults(fn=cmd_list)
    s = sub.add_parser("export"); s.set_defaults(fn=cmd_export)
    return p


def main():
    args = build_parser().parse_args()
    conn = connect(args.db)
    try:
        args.fn(conn, args)
    except SystemExit:
        raise
    except Exception as exc:
        fail("%s: %s" % (type(exc).__name__, exc))
    finally:
        conn.close()


if __name__ == "__main__":
    main()
