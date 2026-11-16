#!/usr/bin/env python3
"""Graph-side consistency checks for the ll_v3 Phase G seed.

Reads the graph through graph_cli.py export and writes
work/ll_v3/reports/graph_seed_checks.json.
"""
import json
import os
import subprocess
import sys
from collections import defaultdict, deque

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DB = os.path.join(ROOT, "experiments", "ll_v3", "skill_system.db")
CLI = os.path.join(ROOT, "graph_cli.py")
OUT = os.path.join(HERE, "reports", "graph_seed_checks.json")

sys.path.insert(0, HERE)
import seed_graph  # noqa: E402  (CAPS, STRATEGIES)

PREDICATE_READS = {
    "01_box_lift_floor": ["object_pose_tracking"],
    "02_push_button": ["scene_joint_state_tracking"],
    "03_box_lift": ["object_pose_tracking", "scene_geometry_tracking",
                    "contact_sensing"],
    "04_push_shelf_forward": ["object_pose_tracking",
                              "scene_geometry_tracking", "contact_sensing"],
    "05_drill_lift": ["object_pose_tracking", "scene_geometry_tracking",
                      "contact_sensing", "grasp_state_tracking"],
    "06_drill_pnp": ["object_pose_tracking", "scene_geometry_tracking",
                     "contact_sensing"],
    "07_box_table_to_shelf": ["object_pose_tracking",
                              "scene_geometry_tracking", "contact_sensing"],
    "08_pick_drill_from_holder": ["scene_geometry_tracking",
                                  "contact_sensing",
                                  "grasp_state_tracking"],
    "09_obstacle_aware_pick_drill": ["object_pose_tracking",
                                     "scene_geometry_tracking",
                                     "contact_sensing",
                                     "grasp_state_tracking"],
}


def export_graph():
    p = subprocess.run([sys.executable, CLI, "--db", DB, "export"],
                       capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    out = json.loads(p.stdout)
    if not out.get("ok"):
        raise RuntimeError(p.stdout)
    return out["result"]


def main():
    ex = export_graph()
    nodes = ex["nodes"]
    edges = ex["edges"]
    by_id = {n["id"]: n for n in nodes}
    by_name = {n["name"]: n for n in nodes}
    caps = [n for n in nodes if n["type"] == "CAPABILITY"]
    strats = [n for n in nodes if n["type"] == "STRATEGY"]
    tasks = [n for n in nodes if n["type"] == "TASK"]

    def spec(n):
        return json.loads(n.get("spec_json") or "{}")

    def name(nid):
        return by_id[nid]["name"]

    req_out = defaultdict(list)
    for e in edges:
        if e["relation"] == "REQUIRES":
            req_out[e["source"]].append(e["target"])

    has_strat = defaultdict(list)
    for e in edges:
        if e["relation"] == "HAS_STRATEGY":
            has_strat[e["source"]].append(e["target"])

    # ---- counts ----
    level_counts = defaultdict(int)
    kind_counts = defaultdict(int)
    for c in caps:
        s = spec(c)
        level_counts[s.get("level", "<missing>")] += 1
        kind_counts[s.get("kind", "<missing>")] += 1
    edge_counts = defaultdict(int)
    for e in edges:
        edge_counts[e["relation"]] += 1

    # ---- check 1: L1 has zero REQUIRES ----
    l1_with_edges = []
    for c in caps:
        if spec(c).get("level") != "L1":
            continue
        if req_out.get(c["id"]):
            l1_with_edges.append({"capability": c["name"],
                                  "requires": [name(t) for t in req_out[c["id"]]]})

    # ---- check 2: L2 >=2 REQUIRES, >=2 L1, spec matches edges ----
    l2_bad = []
    l2_detail = []
    for c in caps:
        s = spec(c)
        if s.get("level") != "L2":
            continue
        targets = [name(t) for t in req_out.get(c["id"], [])]
        l1_targets = [t for t in targets
                      if spec(by_name[t]).get("level") == "L1"]
        spec_req = list(s.get("requires") or [])
        l2_detail.append({"capability": c["name"], "requires": targets,
                          "n": len(targets), "n_l1": len(l1_targets)})
        if len(targets) < 2 or len(l1_targets) < 2 or set(targets) != set(spec_req):
            l2_bad.append({"capability": c["name"], "requires": targets,
                           "n": len(targets), "n_l1": len(l1_targets),
                           "spec_requires": spec_req})

    # ---- check 3: no capability lacks a level ----
    missing_level = [c["name"] for c in caps if not spec(c).get("level")]

    # ---- check 4: every task has two strategies ----
    task_strat_counts = {t["name"]: len(has_strat.get(t["id"], []))
                         for t in tasks}
    tasks_bad = {k: v for k, v in task_strat_counts.items() if v != 2}

    # ---- transitive closure per task ----
    def closure(strategy_id):
        seen = set()
        q = deque([strategy_id])
        while q:
            src = q.popleft()
            for t in req_out.get(src, []):
                if t in seen:
                    continue
                seen.add(t)
                q.append(t)
        return seen

    task_closure = {}
    for t in tasks:
        ids = set()
        for sid in has_strat.get(t["id"], []):
            ids |= closure(sid)
        task_closure[t["name"]] = sorted(name(i) for i in ids)

    # ---- predicate coverage resolves ----
    predicate_missing = {}
    for task, reads in PREDICATE_READS.items():
        have = set(task_closure.get(task, []))
        miss = [r for r in reads if r not in have]
        if miss:
            predicate_missing[task] = miss

    # ---- unclaimed: capability not in any task closure ----
    all_claimed = set()
    for names in task_closure.values():
        all_claimed |= set(names)
    unclaimed = sorted(c["name"] for c in caps if c["name"] not in all_claimed)

    # ---- strategy direct sets match the design ----
    expected_strats = {name: set(caps) for _, name, caps, _ in seed_graph.STRATEGIES}
    strat_mismatch = {}
    for s in strats:
        targets = {name(t) for t in req_out.get(s["id"], [])}
        exp = expected_strats.get(s["name"])
        if exp is None:
            strat_mismatch[s["name"]] = {"unexpected_strategy": True}
        elif targets != exp:
            strat_mismatch[s["name"]] = {
                "missing": sorted(exp - targets),
                "extra": sorted(targets - exp),
            }

    # ---- capability requires match the design ----
    expected_caps = {c["name"]: set(c.get("requires") or [])
                     for c in seed_graph.CAPS if c["level"] == "L2"}
    cap_mismatch = {}
    for c in caps:
        s = spec(c)
        if s.get("level") != "L2":
            continue
        targets = {name(t) for t in req_out.get(c["id"], [])}
        exp = expected_caps.get(c["name"], set())
        if targets != exp:
            cap_mismatch[c["name"]] = {"missing": sorted(exp - targets),
                                       "extra": sorted(targets - exp)}

    # ---- cycle check on capability REQUIRES ----
    color = {}

    def visit(nid):
        color[nid] = 1
        for t in req_out.get(nid, []):
            if by_id[t]["type"] != "CAPABILITY":
                continue
            if color.get(t) == 1:
                return True
            if color.get(t, 0) == 0 and visit(t):
                return True
        color[nid] = 2
        return False

    cycle = any(visit(c["id"]) for c in caps if color.get(c["id"], 0) == 0)

    # ---- node/edge inventory ----
    result = {
        "counts": {
            "nodes": {
                "TASK": len(tasks),
                "CAPABILITY": len(caps),
                "STRATEGY": len(strats),
                "by_level": dict(level_counts),
                "by_kind": dict(kind_counts),
                "by_status": {s: sum(1 for c in caps
                                     if c["status"] == s)
                              for s in sorted({c["status"] for c in caps})},
            },
            "edges": dict(edge_counts),
        },
        "checks": {
            "l1_zero_requires": {"pass": not l1_with_edges,
                                 "violations": l1_with_edges},
            "l2_at_least_two_requires": {"pass": not l2_bad,
                                         "violations": l2_bad},
            "no_missing_level": {"pass": not missing_level,
                                 "violations": missing_level},
            "two_strategies_per_task": {"pass": not tasks_bad,
                                        "violations": tasks_bad,
                                        "counts": task_strat_counts},
            "predicate_coverage_resolves": {
                "pass": not predicate_missing,
                "violations": predicate_missing,
                "required_reads": PREDICATE_READS,
            },
            "strategy_direct_sets_match_design": {
                "pass": not strat_mismatch,
                "violations": strat_mismatch,
            },
            "l2_requires_match_design": {
                "pass": not cap_mismatch,
                "violations": cap_mismatch,
            },
            "no_requires_cycle": {"pass": not cycle},
        },
        "l2_detail": sorted(l2_detail, key=lambda x: x["capability"]),
        "task_closure": task_closure,
        "unclaimed": unclaimed,
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    print(json.dumps({
        "counts": result["counts"],
        "checks": {k: v["pass"] for k, v in result["checks"].items()},
        "unclaimed": unclaimed,
    }, indent=2))


if __name__ == "__main__":
    main()
