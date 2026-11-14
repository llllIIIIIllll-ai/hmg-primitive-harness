#!/usr/bin/env python3
"""Run a short primitive over one or more seeds through the file queue.

This is the execution surface for L1/L2 contract scenarios. It does not
interpret the task success predicate; it returns the primitive's own result and
the episode summary written by the server.

Direct mode:
  python micro/run_micro.py --task 02_push_button --seed 0 \
      --source work/tracks/02_push_button/actor_extract/skills/walk_to_reach.py \
      --name walk_to_reach --kwargs '{}'

Scenario mode:
  python micro/run_micro.py --scenario micro/scenarios/example.yaml
"""
import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")


def load_scenario(path):
    try:
        import yaml
    except Exception as exc:
        raise RuntimeError("scenario mode needs PyYAML: %s" % exc)
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if "source" not in data or "task" not in data:
        raise ValueError("scenario needs 'task' and 'source'")
    data.setdefault("seeds", [0])
    data.setdefault("name", "micro_" + os.path.basename(path).split(".")[0])
    data.setdefault("kwargs", {})
    return data


def run(task, seeds, source_path, name, kwargs, timeout):
    if not os.path.isabs(source_path):
        source_path = os.path.join(ROOT, source_path)
    with open(source_path, encoding="utf-8") as fh:
        source = fh.read()
    req = {
        "tool": "run_seeds",
        "name": name,
        "seeds": list(seeds),
        "source": source,
        "kwargs": kwargs,
        "results": "full",
    }
    env = os.environ.copy()
    env["HMG_TASK"] = task
    proc = subprocess.run(
        [QTOOL, json.dumps(req), str(timeout)],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=timeout + 20,
    )
    if proc.returncode != 0:
        return {"ok": False, "error": proc.stdout.strip() or proc.stderr.strip()}
    try:
        return json.loads(proc.stdout)
    except Exception as exc:
        return {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenario")
    p.add_argument("--task")
    p.add_argument("--seed", type=int, action="append", dest="seeds")
    p.add_argument("--source")
    p.add_argument("--name", default="micro")
    p.add_argument("--kwargs", default="{}")
    p.add_argument("--timeout", type=int, default=180)
    args = p.parse_args()

    if args.scenario:
        data = load_scenario(args.scenario)
        task = data["task"]
        seeds = data["seeds"]
        source = data["source"]
        name = data["name"]
        kwargs = data["kwargs"]
    else:
        if not (args.task and args.source):
            p.error("direct mode needs --task and --source")
        task = args.task
        seeds = args.seeds or [0]
        source = args.source
        name = args.name
        kwargs = json.loads(args.kwargs)

    result = run(task, seeds, source, name, kwargs, args.timeout)
    out = {"ok": bool(result.get("ok")), "task": task, "seeds": seeds,
           "name": name, "response": result}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
