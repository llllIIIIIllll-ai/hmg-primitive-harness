# HMG harness protocol (read this before acting)

## The persistent engine (one per TRACK)
A server process holds ONE HMG env (G1, WBC balance) and the primitive
registry.  Each track (task) has its OWN server and its OWN queue directory
`queue/<task>/`, so parallel tracks never share requests.

Resource policy (measured 2026-11-09): one server uses about 1 CPU core and
~3.7 GB RAM.  `server_ctl.sh` reserves the WHOLE holder cpuset for each
server (`HMG_CPUS=8` by default) because Slurm pins a step to the first
cpus-per-task CPUs; asking for the whole cpuset lets the kernel spread the
~1-core servers over all cores.  This supports up to 6 parallel servers per
8-CPU holder with near-flat per-episode latency (8 servers is possible but
slower).  Memory is not a constraint (6 x ~3.7 GB).

    ./server_ctl.sh start <task> [seed] [job]   # ready task=... queue=...
    ./server_ctl.sh status [task]
    ./server_ctl.sh stop [task]                 # no task = all

Talk to YOUR track's server ONLY through its file queue (the GPU node has no
TCP route from the login node).  `HMG_TASK` selects `queue/<task>/`:

    HMG_TASK=<task> ./qtool.sh '{"tool":"observe"}'
    HMG_TASK=<task> ./qtool.sh '{"tool":"write","name":"squat","source":"..."}'
    HMG_TASK=<task> ./qtool.sh '{"tool":"call","name":"squat","kwargs":{"depth":0.2}}'
    HMG_TASK=<task> ./qtool.sh '{"tool":"list"}'
    HMG_TASK=<task> ./qtool.sh '{"tool":"finish","reason":"..."}'

or write `queue/<task>/req_<id>.json` and read `queue/<task>/resp_<id>.json`
yourself.  A request is answered once by the env that owns the track; never
point two servers at the same queue directory.

## Tools the actor may use
| tool | meaning |
|---|---|
| observe | base pose/tilt, object (pos, hands, dxy_hand), hand-object contacts, t |
| list | registered primitive names |
| write(name, source) | register Python `def run(api, **kwargs)` |
| call(name, kwargs) | run a registered primitive |
| finish(reason) | end the episode (writes the coordinator-only summary) |
| reset(seed) | start a new episode IN this process now (no summary for the old one) |
| finish(reason, keep=true, seed?) | write the summary, then reset in-process instead of exiting |
| run / run_seeds / sweep | run one registered primitive over several seeds in one request |

`api` inside a primitive exposes: `observe()`, `joints()`,
`step_goal(upper, nav, height, dur)`, `move_base(x,y,yaw,dur)`,
`set_height(h,dur)`, `reach_box(gap,z_off,dur)`, `close_grip(angle,dur)`,
`lift(dz,dur)`, plus `api.t`.  No imports, no simulator, no reward.

## Multi-episode requests (opt-in; added 2026-11-09 by the fixer)
Server startup costs ~10-18 s and an in-process reset ~0.5 s, so testing one
primitive over several seeds in ONE request is much faster than restarting
the server per seed.  In-process reset is bit-identical to a fresh-process
episode of the same seed (verified: identical final qpos and engine summary).

    {"tool": "run_seeds", "name": "<registered primitive>",
     "seeds": [0, 1, 2], "kwargs": {...},
     "source": "<optional: register/replace the source first>",
     "reload": false, "results": "full"|"none", "reason": "<optional>"}

Each seed gets a fresh episode (reset), the primitive runs once, and ONE
coordinator-only episode summary is written per seed.  The response is
`{"primitive", "n", "episodes": [{"seed", "ok", "error", "t", "wall_s",
"summary", "result"}]}` and NEVER contains the engine success predicate.
`reset` returns `{"seed", "t"}`; `finish` with `keep=true` returns
`{"done", "reason", "next_seed"}`.

## Primitive sandbox notes (measured 2026-11-09)
- Primitive scope has NO Python builtins (`int`, `float`, `round`, `len`,
  `abs`, `zip`, `min`, `max`, `range`, `Exception` are undefined). Use
  `.item()`, `np.round`, `np.sum`, `np.mean`, or explicit loops.
- `ndarray.sum()` / `.mean()` methods raise `KeyError: '__import__'`; the
  `np.sum()` / `np.mean()` function forms work.
- The source filter rejects a source containing forbidden substrings such as
  `os.` anywhere (even an identifier like `cpos.append` or text in a
  docstring). Avoid them.
- `observe()['hand_object_contacts']` and `hand_contacts_side()` match only
  the SceneObject ROOT body; objects whose collision geoms sit on child
  bodies (drill, some boxes) read 0 there. `api.contact_pairs()` is the
  truthful contact source.
- Do not read the reward / success-predicate implementation. Treat the task
  outcome as unknown and judge from observations.

## Skill layer (added 2026-11-10)
- Task solutions are compositions of registered skills; each skill has a
  mechanism name, a fixed signature with named params, documented
  preconditions/postconditions, a pinned source hash and a verified scope.
  `PRIMITIVES.md` is the skill-layer system prompt plus a compact index.
  LOOKUP ORDER: index entry -> `library/skills/<name>.md` (full signature,
  preconditions, contract, verified scope, call-affecting caveats) -> then call
  `api.skill("<name>", **kwargs)`. An index name alone is never enough for a
  `[CAUTION]` skill: open its description first. Proof history per skill is in
  `library/evidence/<name>.md`.
- `work/tracks/<task>/task_memory.md`: current failure state (measured
  signature), disproven hypotheses, do-not-retry list, next experiment and its
  deciding measurement. Keep it current; it survives context compaction.
- Composition is wired (`api.skill`, 2026-11-10): inside `def run(api, **kwargs)`
  call another registered skill with `r = api.skill("name", **kwargs)`; it runs
  on the same api/env/episode and returns the inner dict unchanged. An unknown
  name raises KeyError listing the registry; a self-call or cycle stops at 8
  primitive frames (7 nested hops) with RecursionError, not a process crash. Never change physics, reward or predicates.

## Debugging tools (read-only; wired 2026-11-09 by the observability fixer)
Per-episode recording is ON by default (`HMG_RECORD=0` disables it; about
+10% episode wall time, <1 MB per ~1000 steps). Use these when you cannot see
whys an episode failed; they never contain the success predicate or reward.
- `{"tool":"events"}` — the current episode's earliest 1-3 events (kind, t,
  residual, contact dump), maxima (tilt, base drop, slip, act jump, joint-limit
  margin, torque fraction, penetration, contact counts), counts by kind, and
  the trace path.
- `{"tool":"window","kind":...,"index":0,"pad":0.2,"fields":[...]}`
  — the event plus sampled arrays over [t-pad, t+pad] (t, q, qd, act, tau,
  obj, obj_quat, obj_lo/hi, base_z, tilt, hand_n, contact_n, nav, height).
- `{"tool":"probe","what":...}` — `contacts` (all contacts with geom/body
  names, normal, distance, penetration, forces), `object` (pose + body-tree
  AABB, optional history), `joints`, `actuators`, `hands`, `geom`, `scene`,
  `bodies`, `state`.
- `{"tool":"trace","fields":[...],"stride":N,"t0":..,"t1":..,"format":"npz"|"csv"}`
  — writes a replayable time-series file under the episode dir and returns
  its path.
- `{"tool":"stills","kind":...,"index":0,"pad":0.2,"n":3,"lookat":"object","mp4":true}`
  — PNGs / a short mp4 around a failing frame on a scratch MjData.
- `run_seeds` accepts `"obs": true` to attach each seed's event summary.
`DEBUG_TOOLS.md` lists every tool above with the question it answers, its
fields, its cost and the tracks that use it, plus the known blind spots.
If these still lack the signal you need, identify the one quantity that would
decide your current question (name, unit, frame, the comparison), build only
that probe in `hmg_tools.py` (read-only), and continue on the same task. Do not
build broad dumps or "all possibly useful" fields; report what you added and
what decision it changed.

## Hard rules
- 35-D goal: 31 upper-body joints + 3 nav + 1 base height. The LOWER BODY is
  owned by the WBC; never command hip/knee/ankle.
- Only the 31 exact joint names in `hmg_harness/contract.py::ALLOWED_UPPER`.
- **You may fix the harness itself when it is buggy**: edit
  `hmg_tools.py` (tool implementations), `hmg_harness/contract.py` (joint
  names/order), or your own primitive sources, and restart the tool server
  (`pkill -f hmg_server.py`, then relaunch on the GPU holder) to get a fresh
  env. Report every edit you make and why.
- Never edit the benchmark/simulator package (`humanoidmimicgen_repo/`), the
  reward, the success predicate, or the model limits.
- The coordinator owns the experiment policy: how many changes to combine in
  one iteration is its call. Read the earliest event/residual first.
