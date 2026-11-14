# Micro-scenarios

A micro-scenario is a short, task-independent contract check for an L1/L2
skill. It is not the benchmark success predicate.

```yaml
capability: servo_ee_world
task: 02_push_button
seeds: [0, 1, 2]
source: work/tracks/02_push_button/actor_extract/skills/walk_to_reach.py
name: walk_to_reach
kwargs: {}
metrics:
  max_tracking_error_m: 0.01
  branch_continuity: true
  joint_limit_margin_min: 0.0
```

Run it with:

```bash
python micro/run_micro.py --scenario micro/scenarios/example.yaml
```

The runner returns the primitive result and the per-seed episode summary. The
actor and validator interpret the metrics; the runner does not declare task
success.

Use diverse initial poses/objects and keep held-out cases separate from the
fitted ones. A skill that only passes the fitted scenarios is not verified.
