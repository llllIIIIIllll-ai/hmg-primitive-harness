"""Video harness: replay a qpos trace in the HMG env and write an mp4.

    python hmg_render.py --task 01_box_lift_floor --trace out/x/trace.npz \
        --out out/x.mp4 [--fps 50] [--lookat X,Y,Z] [--dist 1.9]
        [--azim -90] [--elev 12] [--width 640] [--height 480]

* The robot is recoloured to a neutral grey (the stock scene comes out green).
* Physics is untouched: the trace is applied by writing qpos each frame.
"""
import argparse
import json
import os
import sys

import numpy as np

os.environ.setdefault('MUJOCO_GL', 'egl')
_DEFAULT_REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'third_party', 'humanoidmimicgen')
REPO = os.environ.get('HMG_REPO', _DEFAULT_REPO)
if REPO not in sys.path:
    sys.path.insert(0, REPO)
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco  # noqa: E402


def make_env(task, w, h, seed=0):
    """Same construction the working actor render used: TaskEnv(render=True)."""
    from hmg_harness.adapter import TaskEnv
    return TaskEnv(task, seed=seed, render=True)


def inner_env(env, attr):
    e = env
    for _ in range(5):
        v = getattr(e, attr, None)
        if v is not None:
            return v
        e = getattr(e, 'env', None)
        if e is None:
            break
    return None


def raw(sim):
    """robosuite wraps the mjModel/mjData; mujoco wants the raw ones."""
    return (getattr(sim.model, '_model', sim.model),
            getattr(sim.data, '_data', sim.data))


def recolour_robot(sim, rgba=(0.72, 0.74, 0.78, 1.0)):
    m, _ = raw(sim)
    n = 0
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        name = m.body(b).name if hasattr(m.body(b), 'name') else ''
        if name.startswith('robot0') or name.startswith('gripper0'):
            m.geom_matid[g] = -1
            m.geom_rgba[g] = rgba
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', default='01_box_lift_floor')
    ap.add_argument('--trace', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--fps', type=float, default=50.0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--lookat', default='')
    ap.add_argument('--dist', type=float, default=1.9)
    ap.add_argument('--azim', type=float, default=-90.)
    ap.add_argument('--elev', type=float, default=12.)
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--follow', action='store_true',
                    help='per-frame camera: look at the midpoint of the base '
                         'and the object xy, at --follow-z height')
    ap.add_argument('--follow-z', type=float, default=0.75)
    a = ap.parse_args()

    z = np.load(a.trace, allow_pickle=True)
    if 'qpos' in z:
        q = z['qpos']
    elif 'q' in z:
        q = z['q']
    else:
        q = z[z.files[0]]
    q = np.asarray(q)
    if a.limit:
        q = q[:a.limit]
    q = q[::a.stride]
    env = make_env(a.task, a.width, a.height, a.seed)
    env.reset(a.seed)
    sim = inner_env(env, 'sim')
    m, d = raw(sim)
    n_col = recolour_robot(sim)
    print('recoloured %d robot geoms' % n_col)

    import cv2
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or '.', exist_ok=True)
    wr = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps,
                         (a.width, a.height))
    ctx = sim._render_context_offscreen
    ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    if a.lookat:
        ctx.cam.lookat[:] = [float(x) for x in a.lookat.split(',')]
    else:
        # frame the robot and the box
        look = np.array([d.qpos[0] + 0.35, d.qpos[1], 0.55])
        ctx.cam.lookat[:] = look
    ctx.cam.distance = a.dist
    ctx.cam.azimuth = a.azim
    ctx.cam.elevation = a.elev
    ob = np.asarray(z['obj']) if 'obj' in z.files else None
    if a.limit:
        ob = ob[:a.limit] if ob is not None else None
    if ob is not None:
        ob = ob[::a.stride]
    def follow_look(k):
        p = np.array(q[k][0:2], dtype=float)
        if ob is not None and len(ob) > k and np.all(np.isfinite(ob[k][0:2])):
            p = 0.5 * (p + ob[k][0:2])
        return [float(p[0]), float(p[1]), a.follow_z]
    for k in range(len(q)):
        d.qpos[:] = q[k]
        if a.follow:
            ctx.cam.lookat[:] = follow_look(k)
        mujoco.mj_forward(m, d)
        img = sim.render(width=a.width, height=a.height, camera_name=None)
        if img is None:
            continue
        wr.write(np.ascontiguousarray(img[::-1, :, :3]))
    wr.release()
    print(json.dumps({'out': a.out, 'frames': int(len(q)),
                      'fps': a.fps, 'seconds': len(q) / a.fps}))


if __name__ == '__main__':
    main()
