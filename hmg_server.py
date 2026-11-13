"""Persistent tool server for a Claude Code actor.

Runs ONE env (and its WBC policy) for the whole episode.  The actor talks to
it over a file queue on the shared filesystem (the GPU node is not reachable
by TCP from the login node):

    queue/req_<id>.json  {"tool": "observe"|"write"|"call"|"list"|"finish",
                          "name": str, "source": str, "kwargs": {...}}
    queue/resp_<id>.json {"ok": bool, "result": ..., "error": str?}

    Performance additions (all opt-in; the five tools above are unchanged):
      {"tool": "reset", "seed": int}
          start a new episode in this process now (no summary for the old
          one; the caller normally finishes it first)
      {"tool": "finish", "reason": str, "keep": true, "seed": int?}
          write the episode summary exactly as before, then reset in-process
          (to "seed" when given) instead of exiting; the response is the same
          {'done', 'reason'} object plus 'next_seed'
      {"tool": "run"|"run_seeds"|"sweep", "name": str, "seeds": [int],
       "kwargs": {...}, "source": str?, "reason": str?}
          one request: reset to each seed, run the registered primitive once,
          write one .coord summary per seed, return per-seed primitive
          results/timing (no success predicate field)
    The engine episode summary in .coord is coordinator-only; it is never
    copied into a response.

Usage (on the GPU holder):
    srun --jobid=JOB ... python hmg_server.py 01_box_lift_floor
"""
import glob
import json
import os
import sys
import time

# Start of the expensive import/build block, so the 'server ready' line can
# report how long this start cost (cold NFS metadata dominates it).  Purely
# observational: no state, no episode behavior.
_T0 = time.time()

from hmg_harness.adapter import TaskEnv
from hmg_harness.primitives import PrimitiveRuntime
from hmg_harness.contract import ALLOWED_UPPER
from hmg_harness.fullbody import WBCGoal
from hmg_harness import obs as hmg_obs
import numpy as np

TASK = sys.argv[1] if len(sys.argv) > 1 else '01_box_lift_floor'
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 0
HERE = os.path.dirname(os.path.abspath(__file__))
# One queue directory PER TRACK: several servers can run in parallel without
# racing on the same req_*.json (each request must be answered exactly once,
# by the env that owns that track).  server_ctl.sh sets HMG_QUEUE.
QDIR = os.environ.get('HMG_QUEUE') or os.path.join(HERE, 'queue')
if not os.path.isabs(QDIR):
    QDIR = os.path.join(HERE, QDIR)
os.makedirs(QDIR, exist_ok=True)

env = TaskEnv(TASK, seed=SEED)
obs = env.reset(SEED)
T = {'t': 0.0, 'nav': np.zeros(3), 'height': 0.74}
import hmg_tools

# Source of every registered primitive, kept only so a batch request can
# optionally recompile it between episodes ('run' with reload=true).
SRC = {}
# Episode bookkeeping for in-process restarts: seed of the episode the env is
# sitting in, and whether any primitive has run since its reset.
EP = {'seed': SEED, 'started': False}


def make_tools():
    """Per-episode Tools bookkeeping (t, base-height/nav commands, the reset
    pose used by reach_box).  Same object shape a fresh server builds."""
    t = hmg_tools.Tools(env, lambda: state())
    t.state = lambda: state()
    return t


tools = make_tools()
RT = PrimitiveRuntime(tools)

# ---------------------------------------------------------------------------
# Per-episode observability (added 2026-11-09, obs fixer).
#
# Every episode gets a Recorder (hmg_harness/recorder.py) that is fed one
# record per control step, AFTER the physics step, by wrapping
# env.step_goal.  The wrapper only reads sim.data (qpos/qvel/qfrc_actuator/
# contact list); it never writes state, steps the sim or touches reward /
# success, so recording is behaviour-neutral.  HMG_RECORD=0 starts the server
# with recording disabled (used for the equivalence measurement).
# ---------------------------------------------------------------------------
OBS = {
    'on': os.environ.get('HMG_RECORD', '1').strip().lower()
          not in ('0', 'false', 'off', 'no'),
    'root': os.environ.get('HMG_OBS_DIR')
            or os.path.join(HERE, 'out', 'obs'),
    'contact_every': max(1, int(os.environ.get('HMG_RECORD_CONTACT_EVERY',
                                               '10'))),
    'cur': None,
    'episodes': 0,
    'errors': 0,
}


def start_obs(seed):
    """Create the recorder for the episode the env has just been reset to."""
    if not OBS['on']:
        OBS['cur'] = hmg_obs.ObsEpisode(env, TASK, seed, OBS['root'],
                                       enabled=False)
        return OBS['cur']
    ep = hmg_obs.ObsEpisode(
        env, TASK, seed, OBS['root'], enabled=True,
        contact_every=OBS['contact_every'],
        thresholds=None,
        meta={'server_pid': os.getpid(),
              'record_version': '2026-11-09-obs1'})
    OBS['cur'] = ep
    OBS['episodes'] += 1
    print('recorder episode -> %s' % ep.dir, flush=True)
    return ep


def finalize_obs(success=None, safety_ok=None, reason='', ended_by='finish',
                 extra=None):
    """Finalize the current recorder.  Never raises: an observability
    failure must not change an episode's outcome."""
    ep = OBS['cur']
    if ep is None:
        return None
    try:
        return ep.finalize(success=success, safety_ok=safety_ok,
                           reason=reason, ended_by=ended_by, extra=extra)
    except Exception as exc:
        OBS['errors'] += 1
        print('recorder finalize failed: %s: %s' % (type(exc).__name__, exc),
              flush=True)
        return None


_ORIG_STEP_GOAL = env.step_goal


def _step_goal_recorded(goal):
    """Physics step first, then record its outcome (read-only)."""
    out = _ORIG_STEP_GOAL(goal, return_obs=False)
    ep = OBS['cur']
    if ep is not None:
        try:
            ep.record(goal)
        except Exception as exc:
            OBS['errors'] += 1
            if OBS['errors'] <= 3:
                print('recorder step failed: %s: %s'
                      % (type(exc).__name__, exc), flush=True)
    return out


env.step_goal = _step_goal_recorded
start_obs(SEED)


def state():
    sim = env.sim
    d = sim.data
    # Multi-object scenes expose mujoco_objects in declaration order (e.g.
    # 03_box_lift lists the static table first).  Select the object the task
    # declares through get_object(); see hmg_tools.task_object.  Contacts are
    # then counted against that same object, not objs[0].
    objs, sel = hmg_tools.task_object(env)
    om, hands = None, {}
    for s_ in ('left', 'right'):
        for cand in ('gripper0_%s_grip_site' % s_, '%s_eef_target' % s_):
            i = int(sim.model.site_name2id(cand))
            if i >= 0:
                hands[s_] = np.round(d.site_xpos[i], 3).tolist()
                break
    if sel is not None:
        m = sim.model
        bid = m.body_name2id(sel.root_body)
        op = d.body_xpos[bid].copy()
        om = {'name': sel.name, 'pos': np.round(op, 3).tolist(),
              'hand': hands}
    return {'t': round(getattr(tools, 't', 0.0), 2),
            'base': np.round(d.qpos[:3], 3).tolist(),
            'base_tilt_deg': round(float(np.degrees(env.tilt_rad())), 1),
            'object': om,
            'hand_object_contacts': env.hand_contacts(
                None if sel is None else sel.name)}


def new_episode(seed=None):
    """Start a fresh episode IN THIS PROCESS (performance only).

    robosuite's reset is a hard reset: it rebuilds the model/sim and the scene
    RNG is re-seeded from ``seed``, so the simulator starts exactly as it does
    in a new process.  The WBC policy is put back to its post-construction
    bookkeeping (adapter.reset_policy) and Tools is rebuilt, so the first
    control step matches a freshly constructed env/server for the same
    task+seed.  Registered primitives survive; a 'finish' summary is not
    written for the episode being left behind (that is what 'finish' is for).
    """
    global SEED, tools
    if seed is None:
        seed = SEED
    # finalize the episode being left behind BEFORE the hard reset rebuilds
    # the model (trace + actor view; the .coord summary is still only written
    # by finish/run_seeds, exactly as before)
    finalize_obs(success=None, reason='reset', ended_by='reset')
    SEED = int(seed)
    env.reset(SEED)
    if not env.reset_policy() and getattr(env, 'wrapped', None) == 'wbc':
        print('WARNING: WBC policy state not restored on reset', flush=True)
    tools = make_tools()
    RT.api = tools
    EP['seed'] = SEED
    EP['started'] = False
    start_obs(SEED)
    return SEED


def episode_summary(reason):
    """The engine episode summary (same fields/format as before)."""
    summ = {'task': TASK, 'seed': SEED, 'reason': reason,
            't': round(getattr(tools, 't', 0.0), 2)}
    try:
        d = env.sim.data
        summ['qpos'] = [round(float(x), 8) for x in d.qpos]
        summ['nq'] = int(len(d.qpos))
    except Exception as exc:
        summ['qpos_error'] = '%s: %s' % (type(exc).__name__, exc)
    try:
        res = env.env.is_success()
        # is_success() returns a dict {'task': bool, ...}; bool(dict) is
        # always True, which made the first version of this summary
        # meaningless (caught by the 03_box_lift verifier, 2026-11-09).
        if isinstance(res, dict):
            summ['success'] = bool(res.get('task', False))
            summ['success_detail'] = {k: bool(v) for k, v in res.items()}
        else:
            summ['success'] = bool(res)
    except Exception as exc:
        summ['success'] = None
        summ['success_error'] = '%s: %s' % (type(exc).__name__, exc)
    return summ


def write_summary(summ):
    """Write one summary into .coord (same name/content as before).

    A batch can finish several episodes within one second, and separate
    servers on the same task can finish in the same second, so the name is
    reserved atomically: write a unique temp file, then hard-link it to the
    first free timestamped name (link fails with EEXIST if taken).  The
    common case (no collision) keeps the historical
    episode_<task>_<HHMMSS>.json name; collisions get _1, _2, ...
    """
    cdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '.coord')
    os.makedirs(cdir, exist_ok=True)
    base = os.path.join(cdir, 'episode_%s_%s'
                        % (TASK, time.strftime('%H%M%S')))
    tmp = '%s.tmp.%d' % (base, os.getpid())
    with open(tmp, 'w') as f:
        json.dump(summ, f, indent=1)
    p, n = base + '.json', 1
    while True:
        try:
            os.link(tmp, p)          # atomic claim; no empty-file window
            os.unlink(tmp)
            break
        except FileExistsError:
            if n > 999:
                os.unlink(tmp)
                raise
            p = '%s_%d.json' % (base, n)
            n += 1
        except OSError:
            # filesystem without hard links: historical exists-check
            while os.path.exists(p):
                n += 1
                p = '%s_%d.json' % (base, n)
            os.replace(tmp, p)
            break
    # The summary is coordinator-only. NEVER print its success flag here: the
    # server log is readable by actors (for the "server ready" line), so a
    # success=True/False here leaks the engine predicate into the actor's
    # feedback. Path only.
    print('episode summary -> %s' % p, flush=True)
    return p


def _jsonable(o):
    """numpy -> JSON-safe (only used by the new batch tool)."""
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, (str, int, float, bool)) or o is None:
        return o
    return str(o)


def _seed_list(req):
    seeds = req.get('seeds')
    if seeds is not None:
        if isinstance(seeds, int):
            seeds = [seeds]
        return [int(s) for s in seeds]
    n = int(req.get('repeat') or 0)
    if n > 0:
        s0 = int(req.get('seed', SEED))
        return [s0 + i for i in range(n)]
    return [int(req.get('seed', SEED))]


def run_episodes(req):
    """Run one registered primitive over several seeds in one process.

    Each episode is an in-process restart (new_episode): the simulator starts
    from the same seeded reset a fresh server would use.  One engine summary
    is written per episode in the existing .coord format; the response has
    one record per seed with the primitive's own return value and timing.
    The engine success predicate is NEVER part of a response unless the
    SERVER process was started with HMG_ALLOW_SUCCESS=1 (coordinator-only
    equivalence tests); a request field alone cannot enable it.
    """
    name = req.get('name')
    if not name:
        raise ValueError('run needs "name" (a registered primitive)')
    src = req.get('source')
    if src is not None:
        RT.register(name, src)
        SRC[name] = src
    reload_each = bool(req.get('reload'))
    if reload_each and name not in SRC:
        raise ValueError('reload=true needs the primitive source in this '
                         'request (or an earlier write/run with source)')
    kwargs = req.get('kwargs') or {}
    seeds = _seed_list(req)
    include_success = (bool(req.get('include_success'))
                       and os.environ.get('HMG_ALLOW_SUCCESS') == '1')
    stop_on_success = bool(req.get('stop_on_success')) and include_success
    mode = req.get('results', 'full')
    # Per-seed observability is OPT-IN (obs=true) so the documented response
    # schema of an existing run_seeds request is byte-identical without it.
    include_obs = bool(req.get('obs'))
    out = []
    for s in seeds:
        if reload_each:
            # fresh exec globals: a primitive with module-level state starts
            # its next episode exactly as it would after a re-registration
            RT.register(name, SRC[name])
        # reset_s times the in-process episode restart (finalize + env.reset +
        # make_tools + start_obs).  It is reported separately from wall_s so a
        # slow episode can be attributed to the reset (NFS/asset bound) rather
        # than the primitive.  Timing only: no state, no trajectory, no physics.
        reset_s = None
        if EP['started'] or int(s) != EP['seed']:
            _tr = time.perf_counter()
            new_episode(s)
            reset_s = round(time.perf_counter() - _tr, 3)
        EP['started'] = True
        t0 = time.perf_counter()
        err = None
        try:
            res = RT.call(name, **kwargs)
        except Exception as exc:
            res = None
            err = '%s: %s' % (type(exc).__name__, exc)
        wall = time.perf_counter() - t0
        reason = req.get('reason')
        if reason is None:
            reason = 'run %s seed %d' % (name, int(s))
        try:
            summ = episode_summary(reason)
            # The call configuration is part of the experiment: the same
            # source hash with default kwargs selects different branches
            # (05 gate A, 2026-11-10: {} vs off{seat_mode:'branch'} flipped
            # seed 12 from engine FALSE to TRUE).  Coordinator-only summary
            # field -- the actor response schema is unchanged, and no state,
            # trajectory or predicate value is involved.
            summ['primitive'] = name
            summ['cfg'] = kwargs
            path = write_summary(summ)
        except Exception as exc:
            summ = {'t': round(getattr(tools, 't', 0.0), 2)}
            path = None
            err = (err + '; ' if err else '') + 'summary write failed: ' \
                '%s: %s' % (type(exc).__name__, exc)
        pub = finalize_obs(success=summ.get('success'), reason=reason,
                           ended_by='run_seeds', extra={'seed': int(s)})
        # wall_s is the primitive's own wall time (measured below); PROTOCOL.md
        # documented it from the start but the record never carried it, so
        # timing had to be inferred from queue round-trips.  Additive: no
        # episode state, no trajectory, no physics is involved.
        rec = {'seed': int(s), 'ok': err is None, 'error': err,
               't': summ.get('t'), 'wall_s': round(wall, 3),
               'reset_s': reset_s}
        if include_obs and pub is not None:
            # Actor-safe episode evidence (no success predicate / reward).
            rec['obs'] = {
                'recording': pub.get('recording', False),
                'n_events': pub.get('counts', {}).get('events', 0),
                'earliest_events': pub.get('earliest_events', []),
                'maxima': pub.get('maxima', {}),
                'trace': pub.get('trace'),
            }
        if include_success:
            rec['success'] = summ.get('success')
        if mode != 'none' and res is not None:
            rec['result'] = _jsonable(res)
        out.append(rec)
        if stop_on_success and summ.get('success'):
            break
    return {'primitive': name, 'n': len(out), 'episodes': out}


def _obs_sets():
    """(object body set, hand body set) for contact classification."""
    ep = OBS['cur']
    if ep is not None and ep.rec is not None:
        return ep.obj_bids, ep.hand_bids
    m, _d = hmg_obs.raw_model_data(env)
    _bid, obids, _gids = hmg_obs.object_sets(env, m)
    return obids, hmg_obs.hand_body_ids(m)


def _obs_gids():
    m, _d = hmg_obs.raw_model_data(env)
    _bid, _obids, gids = hmg_obs.object_sets(env, m)
    return gids


def _sim_time():
    try:
        return round(float(env.sim.data.time), 4)
    except Exception:
        return None


def probe(req):
    """Read-only probes: contacts / object / joints / actuators / hands /
    geom / scene / bodies / state.  Nothing here advances the sim or touches
    the reward or the success predicate."""
    what = str(req.get('what') or 'contacts').strip().lower()
    m, d = hmg_obs.raw_model_data(env)
    if what == 'contacts':
        obids, hbids = _obs_sets()
        items = hmg_obs.contact_dump(m, d, limit=int(req.get('limit', 200)),
                                     obj_bids=obids, hand_bids=hbids)
        return {'what': 'contacts', 't': _sim_time(),
                'ncon': int(d.ncon),
                'hand_object_contacts': hmg_obs.hand_object_count(items),
                'contacts': items}
    if what == 'joints':
        return {'what': 'joints', 't': _sim_time(),
                'joints': hmg_obs.joint_inventory(
                    m, d, match=req.get('match', ''),
                    include_torque=not bool(req.get('no_torque'))),
                'contract': tools.joint_ranges()}
    if what == 'actuators':
        return {'what': 'actuators', 't': _sim_time(),
                'actuators': hmg_obs.actuator_inventory(
                    m, d, match=req.get('match', ''))}
    if what == 'object':
        # Task object pose + world AABB over the object's BODY TREE (the
        # drill/box collision geoms sit on child bodies, so geom_detail()
        # on the root body alone reports ngeom=0).
        ep = OBS['cur']
        obids, _hbids = _obs_sets()
        tree_aabb = hmg_obs.object_aabb(m, d, ep.obj_gids if ep is not None
                                        and ep.rec is not None else
                                        _obs_gids())
        try:
            import hmg_tools as _ht
            _objs, _sel = _ht.task_object(env)
        except Exception:
            _sel = None
        out = {'what': 'object', 't': _sim_time(),
               'object': (None if _sel is None else {
                   'name': _sel.name, 'root_body': _sel.root_body,
                   'body_pos': np.round(
                       np.asarray(d.xpos[ep.obj_bid], float),
                       5).tolist() if ep is not None and ep.obj_bid
                       is not None else None,
                   'aabb_lo': (None if tree_aabb is None else
                               np.round(tree_aabb[0], 5).tolist()),
                   'aabb_hi': (None if tree_aabb is None else
                               np.round(tree_aabb[1], 5).tolist())}),
               'geom_detail': tools.geom_detail()}
        if ep is not None and ep.rec is not None and (
                req.get('history') or req.get('t0') is not None
                or req.get('t1') is not None):
            out['history'] = ep.rec.snapshot(
                fields=['t', 'obj', 'obj_quat', 'obj_lo', 'obj_hi'],
                stride=int(req.get('stride', 1)), t0=req.get('t0'),
                t1=req.get('t1'))
        return out
    if what == 'hands':
        g = tools.geom()
        return {'what': 'hands', 't': _sim_time(),
                'hand_bodies': g.get('hand_bodies'),
                'sites': g.get('sites'),
                'hand_geoms': tools.hand_geoms()}
    if what == 'geom':
        if req.get('match'):
            return {'what': 'geom', 't': _sim_time(),
                    'bodies': tools.bodies(match=req['match'])}
        return {'what': 'geom', 't': _sim_time(),
                'object_aabb': tools.geom_detail()}
    if what == 'scene':
        return {'what': 'scene', 't': _sim_time(),
                'objects': tools.scene_objects(
                    detail=bool(req.get('detail')))}
    if what == 'bodies':
        return {'what': 'bodies', 't': _sim_time(),
                'bodies': tools.bodies(match=req.get('match', ''))}
    if what == 'state':
        return state()
    raise ValueError('unknown probe what=%r (use contacts|object|joints|'
                     'actuators|hands|geom|scene|bodies|state)' % what)


def _obs_episode():
    return OBS['cur']


def handle(req):
    tool = req.get('tool')
    if tool == 'observe':
        return state()
    if tool == 'list':
        return sorted(RT.reg)
    if tool == 'write':
        prims = RT.register(req['name'], req['source'])
        SRC[req['name']] = req['source']
        return {'primitives': prims}
    if tool == 'call':
        # a call can move the sim, so a later batch must reset before it
        # assumes the env still sits at its episode start
        EP['started'] = True
        return RT.call(req['name'], **(req.get('kwargs') or {}))
    if tool in ('run', 'run_seeds', 'sweep'):
        # one request, one primitive, several seeds (see run_episodes)
        return run_episodes(req)
    if tool == 'reset':
        # explicit new episode in this process; no summary for the old one
        return {'seed': new_episode(req.get('seed')), 't': 0.0}
    if tool == 'events':
        # Earliest events, maxima, counts and the trace pointer.  The
        # recorder's success/safety fields are stripped inside public().
        ep = _obs_episode()
        if ep is None:
            return {'recording': False}
        return ep.public()
    if tool == 'window':
        # Sampled per-step arrays around one event (default +-0.2 s).
        ep = _obs_episode()
        if ep is None:
            return {'recording': False}
        return ep.window(kind=req.get('kind'), index=req.get('index', 0),
                         pad=req.get('pad', 0.2),
                         max_samples=int(req.get('max_samples', 200)),
                         fields=req.get('fields'))
    if tool == 'probe':
        return probe(req)
    if tool == 'trace':
        # Write the current high-rate series to npz/csv and return the path.
        ep = _obs_episode()
        if ep is None:
            return {'recording': False}
        return ep.trace_snapshot(fields=req.get('fields'),
                                 stride=int(req.get('stride', 1)),
                                 t0=req.get('t0'), t1=req.get('t1'),
                                 fmt=str(req.get('format', 'npz')))
    if tool == 'stills':
        # Render stills / a short mp4 around an event on scratch MjData.
        ep = _obs_episode()
        if ep is None:
            return {'recording': False}
        return ep.stills(kind=req.get('kind'), index=req.get('index', 0),
                         pad=req.get('pad', 0.2), n=int(req.get('n', 3)),
                         width=int(req.get('width', 640)),
                         height=int(req.get('height', 480)),
                         lookat=req.get('lookat', (1.7, 0.0, 0.9)),
                         dist=float(req.get('dist', 1.2)),
                         azim=float(req.get('azim', -90.0)),
                         elev=float(req.get('elev', 15.0)),
                         mp4=bool(req.get('mp4')),
                         fps=int(req.get('fps', 10)))
    if tool == 'finish':
        # Coordinator-only episode summary: the engine's own success flag and
        # the final qpos are written OUTSIDE the queue (the actor's response
        # below is unchanged, so nothing about the predicate reaches it).
        summ = episode_summary(req.get('reason', ''))
        try:
            write_summary(summ)
        except Exception as exc:
            print('summary write failed: %r' % (exc,), flush=True)
        # Observability record: the same success value goes to the
        # coordinator-only recorder copy; the actor response below is
        # unchanged and never contains it.
        finalize_obs(success=summ.get('success'), reason=summ['reason'],
                     ended_by='finish')
        return {'done': True, 'reason': summ['reason']}
    raise ValueError('unknown tool %r' % tool)


print('server ready %.1fs' % (time.time() - _T0), flush=True)
# Queue polling (performance-only change, 2026-11-09).  The old loop slept a
# flat 0.2 s between sweeps, so EVERY tool request paid up to 0.2 s of pure
# latency (median 0.1 s) on top of the primitive's own runtime.  The sweep
# itself is cheap (one listdir of a per-track queue), so poll fast for a
# couple of seconds after answering a request (the actor's next request
# usually follows a tool result immediately) and fall back to a light idle
# period when the actor is thinking.  Requests are still answered exactly
# once, in the same sorted order, with identical response files.
_FAST_SLEEP = 0.002      # poll period while a follow-up request is likely
_IDLE_SLEEP = 0.05       # poll period while idle (was 0.2 for both)
_FAST_WINDOW = 2.0       # seconds of fast polling after the last request
answered = set()         # request ids already answered / skipped
fast_until = 0.0
while True:
    try:
        names = os.listdir(QDIR)
    except OSError:
        names = []
    handled = False
    for name in sorted(names):
        if not (name.startswith('req_') and name.endswith('.json')):
            continue
        rid = name[4:-5]
        if rid in answered:
            continue
        rp = os.path.join(QDIR, 'resp_%s.json' % rid)
        if os.path.exists(rp):
            answered.add(rid)
            continue
        # A client writes the request file in one go; a sweep at 2 ms can
        # observe it mid-write, so retry briefly before treating it as bad.
        req = None
        for _ in range(10):
            try:
                txt = open(os.path.join(QDIR, name)).read()
            except OSError:
                txt = ''
            if txt.strip():
                try:
                    req = json.loads(txt)
                    break
                except ValueError:
                    pass
            time.sleep(0.005)
        exit_after = False
        fatal = False
        try:
            if req is None:
                raise ValueError('unreadable request %r' % name)
            res = handle(req)
            out = {'ok': True, 'result': res}
            if req.get('tool') == 'finish':
                # Old behaviour (default): summary written, response
                # answered, server exits.  New option keep=true: reset the
                # env for the next episode and stay alive (the reset happens
                # before the answer, so a follow-up request already sees the
                # new episode).
                if req.get('keep') or req.get('keep_alive'):
                    try:
                        out['result']['next_seed'] = new_episode(
                            req.get('seed'))
                    except Exception as e:
                        out = {'ok': False,
                               'error': 'in-process reset failed: %s: %s'
                                        % (type(e).__name__, e)}
                        fatal = True
                else:
                    exit_after = True
        except Exception as e:
            out = {'ok': False, 'error': '%s: %s' % (type(e).__name__, e)}
        # Bug fix (2026-11-09, actor_2): this dump used to run OUTSIDE the
        # try/except.  A primitive that returned a numpy bool_ (not JSON
        # serializable) made json.dump raise HERE, outside any handler, and
        # the whole persistent server died (log: "TypeError: Object of type
        # bool_ is not JSON serializable" followed by srun exit 1), losing
        # the episode and never writing the response.  Serialize first so a
        # bad result only fails its own request.
        try:
            txt = json.dumps(out)
        except Exception as e:
            txt = json.dumps({'ok': False,
                              'error': 'unserializable result: %s: %s'
                                       % (type(e).__name__, e)})
        open(rp + '.tmp', 'w').write(txt)
        os.replace(rp + '.tmp', rp)
        answered.add(rid)
        handled = True
        if exit_after:
            sys.exit(0)
        if fatal:
            sys.exit(1)
    now = time.monotonic()
    if handled:
        fast_until = now + _FAST_WINDOW
    time.sleep(_FAST_SLEEP if now < fast_until else _IDLE_SLEEP)
