#!/usr/bin/env python3
"""Parallel evaluation sweep: one server slot per (task, seed) episode.

The sequential sweep runs 10 seeds per task on one server.  This script gives
every (task, seed) pair its own server slot (queue/<task>__e<seed>), boots them
concurrently, registers the verified library on each, and runs the task program
for that single seed.  With 90 slots on a 128-core host the whole 9x10 sweep
finishes in tens of seconds instead of minutes.

Usage:
  python scripts/eval_sweep.py --tasks 01_box_lift_floor,02_push_button \
      --seeds 0-9 --jobs 90

Output: JSON with per-task/per-seed success, wall time and the total.
"""
import argparse
import json
import os
import sqlite3
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QTOOL = os.path.join(HERE, "scripts", "qtool.sh")
SERVER_CTL = os.path.join(HERE, "scripts", "server_ctl.sh")
LOG_DIR = os.path.join(HERE, ".scratch")


def parse_seeds(text):
    out = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = part.split("-", 1)
            out.extend(range(int(lo), int(hi) + 1))
        else:
            out.append(int(part))
    return out


def library_skills(db):
    """Every registered skill name, any status.

    Task programs call verified L2s whose dependency L1s may carry a rejected
    status (a library defect, but the evaluation must reproduce the library as
    registered).  Registering the whole library keeps the sweep comparable with
    the sequential evaluation, which registered each program's dependencies.
    """
    conn = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    rows = conn.execute(
        "SELECT DISTINCT name FROM nodes WHERE type='SKILL' ORDER BY name"
    ).fetchall()
    conn.close()
    return [r[0] for r in rows]


def skill_calls(path):
    """Skill names a source file calls through api.skill('name', ...)."""
    import ast
    try:
        tree = ast.parse(open(path, encoding="utf-8").read())
    except (OSError, SyntaxError):
        return set()
    out = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "skill"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[0].value, str)):
            out.add(node.args[0].value)
    return out


def program_dependencies(task, skills_dir):
    """Transitive skill closure of the task program, plus the program itself."""
    program = os.path.join(skills_dir, "solve_%s.py" % task)
    seen = set()
    queue = list(skill_calls(program))
    while queue:
        name = queue.pop()
        if name in seen:
            continue
        seen.add(name)
        path = os.path.join(skills_dir, name + ".py")
        if os.path.exists(path):
            queue.extend(skill_calls(path) - seen)
    return sorted(seen)


def qtool(queue, req, timeout):
    env = os.environ.copy()
    env["HMG_TASK"] = queue
    proc = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                          cwd=HERE, env=env, capture_output=True, text=True)
    if proc.returncode != 0:
        return {"ok": False, "error": proc.stdout.strip() or proc.stderr.strip()}
    try:
        return json.loads(proc.stdout)
    except ValueError:
        return {"ok": False, "error": proc.stdout.strip()}


def wait_ready(log_path, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with open(log_path, encoding="utf-8", errors="replace") as fh:
                if "server ready" in fh.read():
                    return True
        except OSError:
            pass
        time.sleep(0.1)
    return False


def run_one(task, seeds, skills, args):
    first = seeds[0]
    slot = "e%d" % first
    queue = "%s__%s" % (task, slot)
    t0 = time.time()
    env = os.environ.copy()
    env["HMG_VENV"] = args.venv
    env["HMG_ALLOW_SUCCESS"] = "1"
    # One thread per server: MuJoCo/robosuite pulls in BLAS whose default
    # thread pool oversubscribes the host when many episodes run at once.
    for var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                "NUMEXPR_NUM_THREADS"):
        env[var] = "1"
    log_path = ""
    if args.reuse:
        log_path = os.path.join(LOG_DIR, "srv_%s_*.log" % queue)
        import glob
        hits = sorted(glob.glob(log_path))
        log_path = hits[-1] if hits else ""
    else:
        start = subprocess.run(
            ["bash", SERVER_CTL, "start", task, str(first), slot],
            cwd=HERE, env=env, capture_output=True, text=True,
        )
        for line in start.stdout.splitlines():
            if "log=" in line:
                log_path = line.split("log=", 1)[1].strip()
        if not log_path:
            return {"task": task, "seeds": seeds, "ok": False,
                    "error": "server did not start: %s" % start.stdout.strip()}
    boot_t = time.time() - t0
    if log_path and not wait_ready(log_path, args.boot_timeout):
        return {"task": task, "seeds": seeds, "ok": False,
                "error": "server not ready in %ss" % args.boot_timeout,
                "log": log_path}
    reg_t0 = time.time()
    skills = program_dependencies(task, args.skills_dir)
    if not args.reuse or args.warm:
        for name in skills:
            path = os.path.join(args.skills_dir, name + ".py")
            if not os.path.exists(path):
                continue
            with open(path, encoding="utf-8") as fh:
                src = fh.read()
            r = qtool(queue, {"tool": "write", "name": name, "source": src},
                      args.write_timeout)
            if not r.get("ok"):
                return {"task": task, "seeds": seeds, "ok": False,
                        "error": "register %s: %s" % (name, r.get("error"))}
    register_s = time.time() - reg_t0
    if args.warm:
        return {"task": task, "seeds": seeds, "ok": True, "warm": True,
                "boot_s": round(boot_t, 1), "register_s": round(register_s, 1),
                "log": log_path}
    program = os.path.join(args.skills_dir, "solve_%s.py" % task)
    with open(program, encoding="utf-8") as fh:
        src = fh.read()
    req = {
        "tool": "run_seeds", "name": "solve_%s" % task,
        "seeds": seeds, "source": src, "kwargs": {},
        "include_success": True, "results": "summary",
    }
    resp = qtool(queue, req, args.episode_timeout)
    wall = time.time() - t0
    out = {"task": task, "seeds": seeds, "ok": bool(resp.get("ok")),
           "boot_s": round(boot_t, 1), "register_s": round(register_s, 1),
           "run_s": round(time.time() - reg_t0 - register_s, 1),
           "wall_s": round(wall, 1), "log": log_path, "episodes": []}
    res = resp.get("result") or {}
    eps = res.get("episodes") or []
    if eps:
        for ep in eps:
            rec = {"seed": ep.get("seed"), "success": ep.get("success"),
                   "ok": ep.get("ok"), "t": ep.get("t"),
                   "wall_s": ep.get("wall_s")}
            if ep.get("error"):
                rec["error"] = ep["error"]
            out["episodes"].append(rec)
    else:
        out["error"] = resp.get("error") or "no episode returned"
    return out


def stop_slot(task, slot):
    """Stop only this sweep's slot server; never touch the base or other slots."""
    pid_file = os.path.join(HERE, "run", "%s__%s.pid" % (task, slot))
    try:
        with open(pid_file) as fh:
            pid = int(fh.read().strip())
    except (OSError, ValueError):
        return
    try:
        os.kill(pid, 15)
    except OSError:
        pass
    try:
        os.unlink(pid_file)
    except OSError:
        pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--tasks", required=True)
    p.add_argument("--seeds", default="0-9")
    p.add_argument("--jobs", type=int, default=90)
    p.add_argument("--group", type=int, default=1,
                   help="seeds per server slot (higher = fewer servers, "
                        "less boot contention, more sequential episodes)")
    p.add_argument("--db", default="experiments/ll_v1/skill_system.db")
    p.add_argument("--skills-dir", default="work/ll_v1/skills")
    p.add_argument("--venv", default=os.environ.get(
        "HMG_VENV", "/home/pjy0422/hmg_migration/stack/hmg_venv"))
    p.add_argument("--boot-timeout", type=int, default=180)
    p.add_argument("--write-timeout", type=int, default=120)
    p.add_argument("--episode-timeout", type=int, default=300)
    p.add_argument("--keep-servers", action="store_true")
    p.add_argument("--warm", action="store_true",
                   help="only boot the slot servers and exit; run the sweep "
                        "later with --reuse")
    p.add_argument("--reuse", action="store_true",
                   help="use already-running slot servers (skip boot)")
    args = p.parse_args()

    tasks = [t.strip() for t in args.tasks.split(",") if t.strip()]
    seeds = parse_seeds(args.seeds)
    skills = library_skills(os.path.join(HERE, args.db))
    group = max(1, args.group)
    jobs = []
    for t in tasks:
        for i in range(0, len(seeds), group):
            jobs.append((t, seeds[i:i + group]))
    t0 = time.time()
    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futs = {pool.submit(run_one, t, chunk, skills, args): (t, chunk)
                for t, chunk in jobs}
        for fut in as_completed(futs):
            results.append(fut.result())
    total_s = time.time() - t0
    if not args.keep_servers and not args.warm:
        for t, chunk in jobs:
            stop_slot(t, "e%d" % chunk[0])
    results.sort(key=lambda r: (r["task"], r["seeds"][0]))
    per_task = {}
    for r in results:
        for ep in r.get("episodes", []):
            per_task.setdefault(r["task"], {})[str(ep["seed"])] = bool(ep.get("success"))
    summary = {t: {"success_count": sum(1 for v in d.values() if v),
                   "per_seed": d}
               for t, d in per_task.items()}
    total = sum(v["success_count"] for v in summary.values())
    print(json.dumps({
        "ok": True,
        "warm": bool(args.warm),
        "wall_s": round(total_s, 1),
        "episodes": sum(len(r.get("episodes", [])) for r in results),
        "success_total": total,
        "tasks": summary,
        "boot_s": sorted(r.get("boot_s") for r in results
                         if r.get("boot_s") is not None),
        "failures": [r for r in results
                     if not r.get("ok") or not r.get("episodes")],
    }, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
