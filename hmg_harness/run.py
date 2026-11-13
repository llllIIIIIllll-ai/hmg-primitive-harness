"""run(task, seed, program, out) -- the ONE trusted entrypoint.

A program is ``fn(state, info) -> q43`` (np.ndarray of 43 joint targets in
the canonical order).  It never sees the env, reward, or success predicate.
"""
import hashlib
import os

import numpy as np

from .adapter import TaskEnv
from .fullbody import JointMap, WBCGoal, build_state, to_env_action
from .recorder import Recorder

MAX_STEPS = int(os.environ.get('HMG_MAX_STEPS', '300'))
CTRL_DECIM = int(os.environ.get('HMG_CTRL_DECIM', '10'))


def run(task, seed, program, out, render=False, max_steps=None):
    env = TaskEnv(task, seed=seed, render=render)
    obs = env.reset(seed=seed)
    jm = JointMap(env)
    src = getattr(program, '__code__', None)
    ph = hashlib.sha1(str(getattr(src, 'co_code', b'')).encode()).hexdigest()[:12]
    rec = Recorder(out, task, seed, ph, dt=float(env.control_timestep),
                   limits=(jm.lo, jm.hi))
    steps = max_steps or MAX_STEPS
    info = {'task': task, 'seed': seed}
    done = False
    t = 0.0
    dt = float(env.control_timestep)
    for k in range(steps):
        st = build_state(env, jm, task_obs=obs, t=t)
        cmd = np.asarray(program(st, info), float)
        if env.backend == 'wbc' and cmd.shape == (35,):
            obs, r, done, trunc, einfo = env.step_goal(WBCGoal.from_vec(cmd))
            q_cmd = st.q
        else:
            q_cmd = jm.clip(cmd)
            obs, r, done, trunc, einfo = env.step(
                to_env_action(jm, q_cmd) if env.backend == 'native' else q_cmd)
        op = env.object_pose()
        rec.add(t, st.q, st.qd, q_cmd, base_z=env.base_height(),
                tilt=env.tilt_rad(),
                obj_pos=None if op is None else op[0],
                hand_contacts=env.hand_contacts(),
                progress=None if op is None else float(op[0][2]))
        t += dt
        if done or trunc:
            break
    # official predicate is private to the env; the adapter only reads its
    # public success flag through the last info dict
    success = bool(einfo.get('success', False)) if isinstance(einfo, dict) else False
    summary = rec.save(success=success, safety_ok=True,
                       extra={'reward_last': float(r), 'steps': k + 1})
    return summary
