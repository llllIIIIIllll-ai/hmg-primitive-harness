"""Trusted HMG adapter: the only code that touches the simulator.

Two backends, auto-detected from the env's action space:
  * 'groot'   : the HMG LocomanipEnv wrapper, action = {"q": <43 or 35>}
  * 'native'  : the plain robosuite env (flat action vector)
Candidate programs never see either; they get ``step(q43)`` only.
"""
import copy
import os
import sys

import numpy as np

_DEFAULT_REPO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'third_party', 'humanoidmimicgen')
HMG_REPO = os.environ.get('HMG_REPO', _DEFAULT_REPO)
TASKS = {
    '01_box_lift_floor': 'LMBoxLiftFloor',
    '02_push_button': 'LMPushButton',
    '03_box_lift': 'LMBoxLift',
    '04_push_shelf_forward': 'LMPushShelfForward',
    '05_drill_lift': 'LMDrillLift',
    '06_drill_pnp': 'LMDrillPnP90',
    '07_box_table_to_shelf': 'LMBoxTableToShelf',
    '08_pick_drill_from_holder': 'LMPickDrillFromHolder',
    '09_obstacle_aware_pick_drill': 'LMDrillLiftObstacle',
}


def _import_hmg():
    os.environ.setdefault('MUJOCO_GL', 'egl')
    if HMG_REPO not in sys.path:
        sys.path.insert(0, HMG_REPO)
    import robosuite as suite  # noqa: F401
    import humanoidmimicgen.locomanipulation.models.robots.manipulators.g1_robot  # noqa: F401,E402
    import humanoidmimicgen.locomanipulation.envs  # noqa: F401,E402
    return suite


# ---- in-process episode restarts ----------------------------------------
# Performance only (2026-11-09, fixer perf_2): a server can start a second
# episode without a new process.  robosuite's hard reset rebuilds the
# model/sim and the scene RNG is re-seeded from the requested seed, so the
# simulator side is already fresh; the WBC balance policy, however, is a
# long-lived object that carries per-episode bookkeeping (observation
# history, last action, navigation/base-height commands).  Snapshot the
# values the constructor leaves behind and put them back before the next
# episode, so its first control step is identical to a fresh process.
# This touches no physics, reward, predicate or model limit.
_LOWER_STATE_ATTRS = (
    'cmd', 'height_cmd', 'freq_cmd', 'roll_cmd', 'pitch_cmd', 'yaw_cmd',
    'action', 'obs_buffer', 'use_policy_action', 'use_teleop_policy_cmd',
)


def _snapshot_obj(obj, attrs):
    if obj is None:
        return None
    snap = {'__obj__': obj}
    for a in attrs:
        if hasattr(obj, a):
            snap[a] = copy.deepcopy(getattr(obj, a))
    return snap


def _snapshot_policy(pol):
    """Capture the mutable state a freshly constructed policy starts with."""
    lower = getattr(pol, 'lower_body_policy', None)
    upper = getattr(pol, 'upper_body_policy', None)
    decoupled = lower is not None or upper is not None
    return {
        'decoupled': decoupled,
        'lower': _snapshot_obj(lower if decoupled else pol,
                               _LOWER_STATE_ATTRS),
        'upper': (None if upper is None
                  else {'goal': copy.deepcopy(getattr(upper, 'goal', {}))}),
    }


def _restore_obj(obj, snap):
    if obj is None or snap is None:
        return False
    for a, val in snap.items():
        if a == '__obj__':
            continue
        cur = getattr(obj, a, None)
        if isinstance(cur, np.ndarray) and isinstance(val, np.ndarray) \
                and cur.shape == val.shape and cur.dtype == val.dtype:
            cur[...] = val            # keep array identity
        else:
            setattr(obj, a, copy.deepcopy(val))
    return True


def _restore_policy(pol, snap):
    """Put a policy back into its post-construction state.  Returns True."""
    if pol is None or snap is None:
        return False
    if snap['decoupled']:
        lower = getattr(pol, 'lower_body_policy', None)
        upper = getattr(pol, 'upper_body_policy', None)
    else:
        lower, upper = pol, None
    ok = _restore_obj(lower, snap['lower'])
    if upper is not None and snap.get('upper') is not None:
        goal = getattr(upper, 'goal', None)
        if isinstance(goal, dict):
            goal.clear()
            goal.update(copy.deepcopy(snap['upper']['goal']))
    # these are always None on a fresh policy
    for a in ('obs_history', 'obs_tensor', 'observation'):
        if hasattr(lower, a):
            if a == 'obs_history':
                lower.obs_history.clear()
            else:
                setattr(lower, a, None)
    return ok


class TaskEnv:
    """reset/step only.  Nothing here can disable a safety limit."""

    def __init__(self, task, seed=0, render=False, camera=None):
        self.task = task if task in TASKS else None
        self.name = TASKS.get(task, task)
        self.suite = _import_hmg()
        self.render = render
        self.camera = camera
        # reset() reads self.seed even when it is called without an argument
        # (hmg_render.py does exactly that); the server always passes the seed,
        # which is why this never showed up there.
        self.seed = int(seed)
        self.env = self._make_hmg(render)
        sp = getattr(self.env, 'action_space', None)
        self.backend = ('wbc' if getattr(self, 'wrapped', None) == 'wbc'
                        else 'groot' if (getattr(self, 'wrapped', False)
                                         or hasattr(sp, 'spaces')
                                         or isinstance(sp, dict))
                        else 'native')

    def _make_hmg(self, render):
        """Official HMG wrapper (LocomanipEnv): action = {"q": <groot order>}.
        Falls back to the raw robosuite env only if the WBC runtime is
        unavailable, and says so loudly."""
        try:
            from humanoidmimicgen.dataset_playback import PlaybackConfig
            from humanoidmimicgen.wbc_runtime import get_env as get_sync_env
            cfg = PlaybackConfig(task_name=self.name, robot='G1',
                                 control_frequency=50, sim_frequency=200,
                                 enable_offscreen=bool(render),
                                 enable_onscreen=False, renderer='mujoco')
            env = get_sync_env(cfg, onscreen=False, offscreen=bool(render))
            self.cfg = cfg
            self.action_env = None
            try:
                import time as _t
                from humanoidmimicgen.wbc_runtime import (
                    get_robot_type_and_model, get_policies)
                from humanoidmimicgen.dataset_playback import ActionEnv
                rt, rm = get_robot_type_and_model(cfg.robot, cfg.enable_waist)
                pol, _, _ = get_policies(cfg, rt, rm,
                                         activate_keyboard_listener=False)
                self.action_env = ActionEnv(env, pol)
                self.wrapped = 'wbc'
                # post-construction policy state, restored on in-process
                # episode restarts (see reset_policy)
                self._policy_init = _snapshot_policy(pol)
                print('[adapter] WBC balance backend active')
            except Exception as exc:
                print('[adapter] WBC policy unavailable (%s); raw {"q"} backend'
                      % exc)
                self.wrapped = 'groot'
            return env
        except Exception as exc:
            print('[adapter] HMG wrapper unavailable (%s); raw robosuite env' % exc)
            self.wrapped = False
            return self.suite.make(
                self.name, robots='G1', has_renderer=False,
                has_offscreen_renderer=bool(render), use_camera_obs=False,
                **( {'camera_names': [self.camera]} if self.camera else {}))
        sp = getattr(self.env, 'action_space', None)
        self.backend = 'groot' if (hasattr(sp, 'spaces') or isinstance(sp, dict)) else 'native'
        self.seed = int(seed)

    @property
    def sim(self):
        inner = self.env
        for _ in range(4):
            if hasattr(inner, 'sim'):
                return inner.sim
            nxt = getattr(inner, 'env', None)
            if nxt is None:
                break
            inner = nxt
        raise AttributeError('no .sim found in the env chain')

    @property
    def control_timestep(self):
        for obj in (self.env, getattr(self.env, 'env', None)):
            if obj is None:
                continue
            if hasattr(obj, 'control_timestep'):
                return float(obj.control_timestep)
            if hasattr(obj, 'control_freq'):
                return 1.0 / float(obj.control_freq)
        return 0.02

    # ---- lifecycle -----------------------------------------------------
    def reset(self, seed=None):
        """Honour the seed: the adapter used to ignore it and every fresh
        server drew a new table/box placement, which broke verifier
        reproduction.  Tries the wrapper's seed argument first."""
        if seed is not None:
            self.seed = int(seed)
        for call in (lambda: self.env.reset(seed=self.seed),
                     lambda: self.env.reset()):
            try:
                return call()
            except TypeError:
                continue
        self.env.rng = np.random.default_rng(self.seed)
        return self.env.reset()

    def reset_policy(self):
        """Restore the WBC policy to its post-construction bookkeeping.

        Called after ``reset(seed)`` when a server starts the next episode in
        the same process.  Returns True when a snapshot was restored.
        """
        pol = getattr(getattr(self, 'action_env', None), 'wbc_policy', None)
        return _restore_policy(pol, getattr(self, '_policy_init', None))

    def step_goal(self, goal, return_obs=True):
        """35-D WBC goal: 31 upper-body targets + nav(3) + base height(1).

        return_obs=False skips *only* the post-step observe() that
        SyncEnv.get_step_info() calls.  The pre-step observe() that
        ActionEnv.step() needs in order to run the WBC is untouched, and
        reward/terminated/truncated/info come from the step cache, so they
        are unaffected.  Callers that read the obs tuple keep the default.
        """
        import time as _t
        assert self.action_env is not None, 'no WBC backend'
        now = _t.monotonic()
        d = {'target_time': now,
             'interpolation_garbage_collection_time': now - 2.0 / 50.0,
             'target_upper_body_pose': goal.upper,
             'navigate_cmd': goal.nav,
             'base_height_command': goal.height}
        if return_obs:
            return self.action_env.step(d)

        sy = self.action_env.sync_env
        cache = sy.cache
        sentinel = object()
        old = sy.__dict__.get('get_step_info', sentinel)

        def _no_post_obs():
            return (None, cache['reward'], cache['terminated'],
                    cache['truncated'], cache['info'])

        sy.get_step_info = _no_post_obs
        try:
            return self.action_env.step(d)
        finally:
            if old is sentinel:
                del sy.get_step_info
            else:
                sy.get_step_info = old

    def step(self, q43):
        act = np.asarray(q43, float)
        if self.backend == 'groot':
            obs, r, done, trunc, info = self.env.step({'q': act})
            return obs, float(r), bool(done), bool(trunc), info
        obs, r, done, info = self.env.step(act)
        return obs, float(r), bool(done), False, info

    # ---- diagnostics (engine-side, program never sees these) ------------
    def base_height(self):
        return float(self.sim.data.qpos[2])

    def tilt_rad(self):
        q = self.sim.data.qpos[3:7]
        w, x, y, z = q[0], q[1], q[2], q[3]
        # angle between the body z axis and world z
        up_z = 1.0 - 2.0 * (x * x + y * y)
        return float(np.arccos(np.clip(up_z, -1.0, 1.0)))

    def _inner_objects(self):
        """Walk TaskEnv -> G1SyncEnv -> Groot2LocoManipEnv -> task env: only
        the innermost env owns ``mujoco_objects`` (the old one-level lookup
        made hand_contacts() return 0 forever)."""
        inner = self.env
        for _ in range(5):
            objs = getattr(inner, 'mujoco_objects', None) or []
            if objs:
                return objs
            inner = getattr(inner, 'env', None)
            if inner is None:
                break
        return []

    def hand_contacts(self, obj_name=None):
        """Contacts between any hand body and the task object."""
        m, d = self.sim.model, self.sim.data
        objs = self._inner_objects()
        if not objs:
            return 0
        o = next((x for x in objs if x.name == obj_name), objs[0])
        bid = m.body_name2id(o.root_body)
        n = 0
        for ci in range(d.ncon):
            c = d.contact[ci]
            b1 = m.geom_bodyid[c.geom1]
            b2 = m.geom_bodyid[c.geom2]
            if bid not in (b1, b2):
                continue
            other = b2 if b1 == bid else b1
            if 'hand' in m.body(other).name or 'gripper' in m.body(other).name:
                n += 1
        return n

    def contacts(self):
        return int(self.sim.data.ncon)

    def object_pose(self, name=None):
        """Root pose of the first movable object (or a named one)."""
        objs = self._inner_objects()
        if not objs:
            return None
        o = next((x for x in objs if x.name == name), objs[0])
        m, d = self.sim.model, self.sim.data
        bid = m.body_name2id(o.root_body)
        return d.body_xpos[bid].copy(), d.body_xmat[bid].reshape(3, 3).copy()
