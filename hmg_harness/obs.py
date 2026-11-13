"""Per-episode observability: wiring, probes, trace export and stills.

This module is the bridge between the live simulator and ``Recorder``.  It is
CROSS-TRACK infrastructure: everything here is read-only with respect to
physics (it never writes qpos/qvel/actuators, never steps the sim and never
touches reward / success).  The only mutation it performs is rendering on a
SCRATCH ``mujoco.MjData`` so the live state is untouched.

Public surface used by hmg_server.py:
  * ``ObsEpisode`` -- one per episode; ``record(goal)`` is called after every
    control step, ``finalize(...)`` writes the episode files, and the
    ``events/window/probe/trace/stills`` helpers answer actor requests.
  * ``contact_dump`` / ``joint_inventory`` / ``actuator_inventory`` --
    read-only probes (geom/body names, normals, penetration, forces,
    joint limits, actuator ranges and measured torques).
"""
import json
import os
import time

import numpy as np

# Contract order of the 31 upper-body commands, used to reconstruct the
# contract-order command vector from the WBC-order goal the policy receives.
from .contract import PERM_TO_WBC

SAFETY_EVENT_KINDS = ('JOINT_LIMIT', 'TILT', 'BASE_DROP', 'OBJECT_SLIP',
                      'CONTACT_PENETRATION')


# ---------------------------------------------------------------------------
# model caches (read-only)
# ---------------------------------------------------------------------------
def raw_model_data(env):
    """The unwrapped mujoco.MjModel/MjData (robosuite wraps them; the
    existing Tools.raw does the same unwrap)."""
    mm, dd = env.sim.model, env.sim.data
    return getattr(mm, '_model', mm), getattr(dd, '_data', dd)


def build_limit_arrays(m):
    """(lo, hi) over qpos, +-inf where the joint is not limited.

    Only the qpos entries of limited hinge/slide joints get a real bound; the
    floating base and free object joints stay unbounded so the margin is not
    polluted by meaningless free-joint ranges.
    """
    nq = int(m.nq)
    lo = np.full(nq, -np.inf)
    hi = np.full(nq, np.inf)
    for j in range(int(m.njnt)):
        if not int(m.jnt_limited[j]):
            continue
        jt = int(m.jnt_type[j])
        if jt not in (2, 3):              # slide / hinge (G1: all hinges)
            continue
        adr = int(m.jnt_qposadr[j])
        lo[adr], hi[adr] = float(m.jnt_range[j][0]), float(m.jnt_range[j][1])
    return lo, hi


def build_torque_limits(m):
    """Per-DoF actuator range (absolute) from actuator_ctrlrange.

    For a general actuator with gear 1 and no bias, ``ctrlrange`` is the
    force/torque command range; the measured ``qfrc_actuator`` is compared
    against it to fire TORQUE_SATURATION.  Zero means 'unknown/disabled'.
    """
    lim = np.zeros(int(m.nv))
    for a in range(int(m.nu)):
        jid = int(m.actuator_trnid[a, 0])
        if jid < 0 or jid >= int(m.njnt):
            continue
        dof = int(m.jnt_dofadr[jid])
        if dof < 0 or dof >= int(m.nv):
            continue
        lo, hi = float(m.actuator_ctrlrange[a][0]), float(
            m.actuator_ctrlrange[a][1])
        if hi <= lo:
            continue
        lim[dof] = max(abs(lo), abs(hi))
    return lim



def object_aabb(m, d, obj_gids):
    """World AABB of the task object's geom tree at the CURRENT state.

    Boxes use their oriented corners; every other geom type (sphere, capsule,
    cylinder, mesh) uses MuJoCo's bounding-sphere radius, which is exact for
    spheres and conservative for the rest.  Read-only.
    """
    if not obj_gids:
        return None
    import mujoco
    lo = hi = None
    for gi in obj_gids:
        cen = np.asarray(d.geom_xpos[gi], float)
        Rg = np.asarray(d.geom_xmat[gi], float).reshape(3, 3)
        sz = np.asarray(m.geom_size[gi], float)
        gtype = int(m.geom_type[gi])
        if gtype == int(mujoco.mjtGeom.mjGEOM_BOX):
            ext = np.abs(Rg) @ sz
            pts = cen + np.array([[sx * ext[0], sy * ext[1], sz2 * ext[2]]
                                  for sx in (-1, 1) for sy in (-1, 1)
                                  for sz2 in (-1, 1)])
            blo, bhi = pts.min(axis=0), pts.max(axis=0)
        else:
            r = float(m.geom_rbound[gi])
            blo, bhi = cen - r, cen + r
        lo = blo if lo is None else np.minimum(lo, blo)
        hi = bhi if hi is None else np.maximum(hi, bhi)
    return lo, hi

def body_descendants(m, root):
    """Set of body ids equal to ``root`` or descending from it."""
    out = {int(root)}
    for b in range(1, int(m.nbody)):
        p = int(m.body_parentid[b])
        while p > 0:
            if p == int(root):
                out.add(int(b))
                break
            p = int(m.body_parentid[p])
    return out


def object_sets(env, m):
    """(root_body_id, body_id_set, geom_id_set) for the task's primary
    object, or (None, set(), set()).  Uses hmg_tools.task_object(), the same
    selection every existing probe uses."""
    try:
        import hmg_tools
        _objs, sel = hmg_tools.task_object(env)
    except Exception:
        sel = None
    if sel is None:
        return None, set(), set()
    import mujoco
    bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                sel.root_body))
    if bid < 0:
        return None, set(), set()
    bids = body_descendants(m, bid)
    gids = {gi for gi in range(int(m.ngeom))
            if int(m.geom_bodyid[gi]) in bids}
    return bid, bids, gids


def hand_body_ids(m):
    """Body ids that count as a hand: same criterion as TaskEnv.hand_contacts
    ('hand' or 'gripper' in the body name)."""
    return {i for i in range(int(m.nbody))
            if ('hand' in m.body(i).name or 'gripper' in m.body(i).name)}


# ---------------------------------------------------------------------------
# contact dump (per link; names, normal, distance, penetration, force)
# ---------------------------------------------------------------------------
def contact_dump(m, d, limit=200, obj_bids=None, hand_bids=None):
    """All current sim contacts as JSON-safe dicts.

    ``normal`` is the world contact normal (MuJoCo ``contact.frame[:3]``),
    ``dist`` the signed distance (negative = penetration), ``penetration``
    ``max(0, -dist)``.  Contact force is read from ``efc_force`` when the
    contact exposes an ``efc_address`` (normal component + total magnitude);
    it is ``None`` when the solver rows are unavailable.  ``hand_obj`` marks
    contacts between a hand/gripper body and the task object's body tree.
    Names are cached by the caller via the model itself (cheap enough).
    """
    out = []
    ncon = int(d.ncon)
    efc = np.asarray(d.efc_force, float)
    for ci in range(ncon):
        if len(out) >= int(limit):
            break
        c = d.contact[ci]
        g1, g2 = int(c.geom1), int(c.geom2)
        b1 = int(m.geom_bodyid[g1])
        b2 = int(m.geom_bodyid[g2])
        dist = float(c.dist)
        frame = np.asarray(c.frame, float)
        fn = ft = None
        try:
            ea = int(c.efc_address)
        except Exception:
            ea = -1
        if ea >= 0 and ea < len(efc):
            fn = float(efc[ea])                     # normal component
            ft = float(np.linalg.norm(efc[ea:ea + 3]))
        o1 = bool(obj_bids is not None and b1 in obj_bids)
        o2 = bool(obj_bids is not None and b2 in obj_bids)
        ho = bool(o1 or o2) and bool(hand_bids is not None and (
            (o1 and b2 in hand_bids) or (o2 and b1 in hand_bids)))
        out.append({
            'g1': m.geom(g1).name, 'g2': m.geom(g2).name,
            'b1': m.body(b1).name, 'b2': m.body(b2).name,
            'pos': np.round(np.asarray(c.pos, float), 5).tolist(),
            'normal': np.round(frame[:3], 4).tolist(),
            'dist': round(dist, 6),
            'penetration': round(max(0.0, -dist), 6),
            'force_normal': None if fn is None else round(fn, 3),
            'force': None if ft is None else round(ft, 3),
            'hand_obj': ho,
            'obj_contact': bool(o1 or o2),
        })
    if ncon > len(out):
        out.append({'truncated': True, 'ncon_total': ncon,
                    'shown': len(out)})
    return out


def hand_object_count(items):
    return sum(1 for c in items if c.get('hand_obj'))


# ---------------------------------------------------------------------------
# joint / actuator inventory
# ---------------------------------------------------------------------------
_JTYPE = {0: 'free', 1: 'ball', 2: 'slide', 3: 'hinge'}
_ATYPE = {0: 'general', 1: 'motor', 2: 'position', 3: 'velocity',
          4: 'intvelocity', 5: 'damper', 6: 'cylinder', 7: 'muscle',
          8: 'adhesion'}


def joint_inventory(m, d, match='', include_torque=True):
    """Read-only joint inventory: type, qpos/qvel, range, limits, body and
    the measured actuator torque at its DoF."""
    tau_lim = build_torque_limits(m)
    tau = np.asarray(d.qfrc_actuator, float)
    q, qd = np.asarray(d.qpos, float), np.asarray(d.qvel, float)
    out = []
    for j in range(int(m.njnt)):
        jt = int(m.jnt_type[j])
        nm = m.joint(j).name
        if match and match not in nm:
            continue
        qa = int(m.jnt_qposadr[j])
        da = int(m.jnt_dofadr[j])
        rec = {'name': nm, 'type': _JTYPE.get(jt, str(jt)),
               'body': m.body(int(m.jnt_bodyid[j])).name,
               'qposadr': qa, 'dofadr': da,
               'limited': bool(int(m.jnt_limited[j])),
               'range': np.round(np.asarray(m.jnt_range[j], float),
                                 5).tolist()}
        if jt in (2, 3):
            rec['q'] = round(float(q[qa]), 5)
            rec['qd'] = round(float(qd[da]), 5)
            if include_torque:
                rec['tau'] = round(float(tau[da]), 4)
                rec['tau_limit'] = round(float(tau_lim[da]), 4)
        elif jt == 0:
            rec['pos'] = np.round(q[qa:qa + 3], 5).tolist()
            rec['quat'] = np.round(q[qa + 3:qa + 7], 5).tolist()
            rec['qd'] = np.round(qd[da:da + 6], 5).tolist()
        out.append(rec)
    return out


def actuator_inventory(m, d, match=''):
    """Read-only actuator inventory: joint, ctrlrange (= force range for the
    G1 general actuators), gear, measured actuator force and generalized
    torque at its DoF."""
    af = np.asarray(d.actuator_force, float)
    tau = np.asarray(d.qfrc_actuator, float)
    out = []
    for a in range(int(m.nu)):
        nm = m.actuator(a).name
        jid = int(m.actuator_trnid[a, 0])
        jname = m.joint(jid).name if 0 <= jid < int(m.njnt) else None
        if match and match not in nm and (jname is None or match not in jname):
            continue
        dof = int(m.jnt_dofadr[jid]) if jname else -1
        out.append({
            'name': nm, 'joint': jname, 'type': _ATYPE.get(
                int(m.actuator_gaintype[a]), str(int(m.actuator_gaintype[a]))),
            'dofadr': dof,
            'ctrlrange': np.round(np.asarray(m.actuator_ctrlrange[a],
                                             float), 4).tolist(),
            'gear': np.round(np.asarray(m.actuator_gear[a], float), 4).tolist(),
            'force': round(float(af[a]), 4),
            'tau': (None if dof < 0 else round(float(tau[dof]), 4)),
            'force_frac': (None if dof < 0 or
                           max(abs(x) for x in m.actuator_ctrlrange[a]) <= 0
                           else round(abs(float(af[a])) / max(
                               abs(x) for x in m.actuator_ctrlrange[a]), 4)),
        })
    return out


# ---------------------------------------------------------------------------
# per-episode recorder session
# ---------------------------------------------------------------------------
class ObsEpisode:
    """One recorder session bound to one env episode.

    Created right after ``env.reset(seed)`` (the hard reset rebuilds the
    model, so the model caches must be rebuilt too).  ``record(goal)`` must be
    called AFTER the physics step; it only reads ``sim.data``.
    """

    def __init__(self, env, task, seed, out_root, enabled=True,
                 program_hash='', contact_every=10, thresholds=None,
                 meta=None):
        from .recorder import Recorder
        self.env = env
        self.task = task
        self.seed = int(seed)
        self.enabled = bool(enabled)
        self.program_hash = program_hash
        self.m, self.d = raw_model_data(env)
        self.dt = float(env.control_timestep)
        self.t = 0.0
        self.steps = 0
        self.finalized = False
        self.errors = 0
        self.contact_every = max(1, int(contact_every))
        self.lo, self.hi = build_limit_arrays(self.m)
        self.tau_lim = build_torque_limits(self.m)
        self.hand_bids = hand_body_ids(self.m)
        self.obj_bid, self.obj_bids, self.obj_gids = object_sets(env, self.m)
        self.dir = None
        self.rec = None
        if self.enabled:
            self.dir = self._make_dir(out_root)
            self.rec = Recorder(
                self.dir, task, self.seed, program_hash, dt=self.dt,
                limits=(self.lo, self.hi), thresholds=thresholds,
                meta=meta, contact_every=self.contact_every)

    def _make_dir(self, out_root):
        base = os.path.join(out_root, str(self.task),
                            'ep_%s_p%d_s%d' % (time.strftime('%Y%m%d_%H%M%S'),
                                               os.getpid(), self.seed))
        p, n = base, 1
        while True:
            try:
                os.makedirs(p, exist_ok=False)
                return p
            except FileExistsError:
                p = '%s_%d' % (base, n)
                n += 1

    # ---- recording (read-only after the step) --------------------------
    def record(self, goal):
        # A finalized episode is frozen: the trace on disk is the evidence
        # for it.  Commands issued afterwards (e.g. a stray 'call' after a
        # run_seeds batch) are NOT silently appended to a trace that was
        # already written, which would make the live view disagree with it.
        if not self.enabled or self.rec is None or self.finalized:
            return
        m, d = self.m, self.d
        try:
            contacts = contact_dump(m, d, obj_bids=self.obj_bids,
                                    hand_bids=self.hand_bids)
            hand_n = hand_object_count(contacts)
            max_pen = max([c.get('penetration', 0.) for c in contacts
                           if c.get('obj_contact')], default=0.)
            max_pen_all = max([c.get('penetration', 0.) for c in contacts
                               if 'penetration' in c], default=0.)
            q = np.asarray(d.qpos, float)
            qd = np.asarray(d.qvel, float)
            tau = np.asarray(d.qfrc_actuator, float)
            # WBC-order goal -> contract-order command (scatter)
            cmd = np.empty(31, float)
            cmd[PERM_TO_WBC] = np.asarray(goal.upper, float)
            obj_pos = obj_quat = obj_aabb = None
            if self.obj_bid is not None:
                obj_pos = np.asarray(d.xpos[self.obj_bid], float)
                obj_quat = np.asarray(d.xquat[self.obj_bid], float)
                obj_aabb = object_aabb(m, d, self.obj_gids)
            self.rec.add(
                self.t, q, qd, cmd,
                base_z=float(q[2]), tilt=float(self.env.tilt_rad()),
                obj_pos=obj_pos, obj_quat=obj_quat, obj_aabb=obj_aabb,
                hand_contacts=int(hand_n), torque=tau,
                torque_limit=self.tau_lim, contacts=contacts,
                max_penetration=float(max_pen),
                max_penetration_all=float(max_pen_all),
                nav=np.asarray(goal.nav, float), height=float(goal.height))
            self.t += self.dt
            self.steps += 1
        except Exception as exc:
            # Recording must never break an episode.  Count and continue,
            # but make the first failures visible in the server log.
            self.errors += 1
            if self.errors <= 3:
                import traceback
                traceback.print_exc()

    # ---- lifecycle -----------------------------------------------------
    def finalize(self, success=None, safety_ok=None, reason='', ended_by='finish',
                 extra=None):
        """Write the episode files.  ``success``/``safety_ok`` are stored in
        the coordinator-only summary; the actor-safe view (events.json and
        every tool response) never contains them."""
        if self.rec is None or self.finalized:
            return None
        if safety_ok is None and success is not None:
            safety_ok = not any(e['kind'] in SAFETY_EVENT_KINDS
                                for e in self.rec.events)
        ex = {'reason': reason, 'ended_by': ended_by}
        if extra:
            ex.update(extra)
        if success is None:
            # episode abandoned without a .coord summary (reset tool): keep
            # the evidence but do not invent an outcome.
            ex['outcome'] = 'not_recorded (episode left without finish)'
            try:
                self._save_no_outcome(ex)
            finally:
                self.finalized = True
            return None
        self.rec.save(success, bool(safety_ok), ex)
        self.finalized = True
        return self.public(ended_by=ended_by)

    def _save_no_outcome(self, ex):
        """Trace + actor view only; the private summary would need a success
        value the caller does not have."""
        np.savez_compressed(os.path.join(self.dir, 'trace.npz'),
                            **self.rec._arrays())
        with open(os.path.join(self.dir, 'contacts.jsonl'), 'w') as f:
            for snap in self.rec.contact_snapshots:
                f.write(json.dumps(snap) + '\n')
        self.rec._final = dict(ex)
        pub = self.rec.public(ended_by=ex.get('ended_by', 'reset'),
                              trace=os.path.join(self.dir, 'trace.npz'))
        with open(os.path.join(self.dir, 'events.json'), 'w') as f:
            json.dump(pub, f, indent=1)

    # ---- actor-facing views --------------------------------------------
    def public(self, ended_by=None):
        if self.rec is None:
            return {'recording': False, 'note':
                    'recording disabled (server started with HMG_RECORD=0)'}
        if ended_by is None:
            ended_by = ('finalized' if self.finalized else 'live')
        trace = (os.path.join(self.dir, 'trace.npz')
                 if self.finalized else None)
        out = self.rec.public(ended_by=ended_by, trace=trace)
        out['recording'] = True
        out['finalized'] = bool(self.finalized)
        out['live'] = not self.finalized
        out['recording_errors'] = int(self.errors)
        if not self.finalized:
            out['note'] = ('live episode: the trace is written at finish; '
                           'use {"tool":"trace"} for a snapshot now')
        return out

    def window(self, kind=None, index=0, pad=0.2, max_samples=200,
               fields=None):
        if self.rec is None:
            return {'recording': False}
        out = self.rec.window(kind=kind, index=index, pad=pad,
                              max_samples=max_samples, fields=fields)
        out['recording'] = True
        return out

    def trace_snapshot(self, fields=None, stride=1, t0=None, t1=None,
                       fmt='npz'):
        """Write the current high-rate series to disk (npz or csv) and return
        the path; usable while the episode is live."""
        if self.rec is None:
            return {'recording': False}
        snap = self.rec.snapshot(fields=fields, stride=stride, t0=t0, t1=t1)
        self.__dict__.setdefault('_trace_n', 0)
        self._trace_n += 1
        if fmt == 'csv':
            path = os.path.join(self.dir, 'trace_live_%02d.csv' % self._trace_n)
            self._write_csv(path, snap)
        else:
            path = os.path.join(self.dir, 'trace_live_%02d.npz' % self._trace_n)
            self._write_npz(path, snap)
        snap['path'] = path
        snap['format'] = 'csv' if fmt == 'csv' else 'npz'
        return snap

    def _write_npz(self, path, snap):
        arrs = {}
        for f in snap['fields']:
            vals = snap.get(f)
            if vals is None:
                continue
            try:
                arrs[f] = np.asarray(vals, float)
            except Exception:
                pass
        np.savez_compressed(path, **arrs)

    def _write_csv(self, path, snap):
        cols, names = [], []
        for f in snap['fields']:
            vals = snap.get(f)
            if vals is None:
                continue
            try:
                a = np.asarray(vals, float)
            except Exception:
                continue
            a = a.reshape(a.shape[0], -1)
            for k in range(a.shape[1]):
                names.append(f if a.shape[1] == 1 else '%s_%d' % (f, k))
                cols.append(a[:, k])
        if not cols:
            open(path, 'w').write('')
            return
        data = np.column_stack(cols)
        np.savetxt(path, data, delimiter=',', header=','.join(names),
                   comments='')

    # ---- stills / short mp4 around an event ---------------------------
    def stills(self, kind=None, index=0, pad=0.2, n=3, width=640,
               height=480, lookat=(1.7, 0.0, 0.9), dist=1.2, azim=-90.0,
               elev=15.0, mp4=False, fps=10, prefix='still'):
        """Render n frames around an event on SCRATCH MjData (the live sim is
        not modified).  Returns the PNG paths (and an mp4 path if asked)."""
        if self.rec is None:
            return {'recording': False}
        i, ev = self.rec._event(kind, index)
        if ev is None:
            return {'event': None, 'n_events': len(self.rec.events)}
        t = np.asarray(self.rec.t, float)
        qs = self.rec.q
        if not qs:
            return {'event': ev, 'error': 'no recorded steps'}
        lo_t, hi_t = ev['t'] - float(pad), ev['t'] + float(pad)
        sel = np.where((t >= lo_t) & (t <= hi_t))[0]
        if sel.size == 0:
            sel = np.array([int(np.argmin(np.abs(t - ev['t'])))])
        pick = np.linspace(0, sel.size - 1, max(1, int(n))).astype(int)
        idxs = [int(sel[k]) for k in pick]
        import mujoco
        m, _live = self.m, self.d
        r = mujoco.Renderer(m, int(height), int(width))
        cam = mujoco.MjvCamera()
        cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
        lookat_obj = isinstance(lookat, str) and lookat == 'object'
        if lookat_obj:
            # initial value; updated per frame from the recorded trace
            if self.obj_bid is not None:
                cam.lookat[:] = np.asarray(self.d.xpos[self.obj_bid],
                                           float)
        else:
            cam.lookat[:] = [float(x) for x in lookat]
        cam.distance = float(dist)
        cam.azimuth = float(azim)
        cam.elevation = float(elev)
        frames, paths = [], []
        try:
            for k, j in enumerate(idxs):
                if lookat_obj:
                    op = (self.rec.obj[j] if j < len(self.rec.obj)
                          and self.rec.obj[j] is not None else None)
                    if op is not None:
                        cam.lookat[:] = np.asarray(op, float)
                ds = mujoco.MjData(m)
                ds.qpos[:] = np.asarray(qs[j], float)
                mujoco.mj_forward(m, ds)   # scratch: live data untouched
                r.update_scene(ds, camera=cam)
                img = np.asarray(r.render())
                frames.append(np.array(img[:, :, :3], copy=True))
                p = os.path.join(self.dir, '%s_%s_%02d_t%.3f.png' % (
                    prefix, ev['kind'], k, float(t[j])))
                try:
                    import cv2
                    cv2.imwrite(p, frames[-1])
                except Exception:
                    from PIL import Image
                    Image.fromarray(frames[-1]).save(p)
                paths.append(p)
        finally:
            r.close()
        out = {'event': ev, 'frames': paths, 't': [float(t[j]) for j in idxs]}
        if mp4 and frames:
            try:
                import cv2
                p = os.path.join(self.dir, '%s_%s_clip.mp4' % (
                    prefix, ev['kind']))
                h, w = frames[0].shape[:2]
                vw = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*'mp4v'),
                                     float(fps), (w, h))
                for fr in frames:
                    vw.write(fr[:, :, ::-1])    # RGB -> BGR
                vw.release()
                out['mp4'] = p
            except Exception as exc:
                out['mp4_error'] = '%s: %s' % (type(exc).__name__, exc)
        return out
