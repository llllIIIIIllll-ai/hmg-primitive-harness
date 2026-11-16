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

    Meshes use their exact transformed vertices; box/sphere/capsule/cylinder
    use their oriented extent (|R| @ size).  This matches
    ``hmg_tools.Tools.object_aabb`` so the debug probe and a primitive read the
    same extent at the same instant.  Read-only.
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
        if gtype == int(mujoco.mjtGeom.mjGEOM_MESH):
            # Exact mesh vertices (2026-11-15, ll_v2 W1): the bounding-sphere
            # radius overestimated every mesh object (probe extent 0.330 m vs
            # exact 0.188 m on the 05 drill), so the debug probe and the
            # primitive's api.object_aabb() disagreed at the same instant.
            aid = int(m.geom_dataid[gi])
            v0 = int(m.mesh_vertadr[aid])
            v1 = v0 + int(m.mesh_vertnum[aid])
            pts = np.asarray(m.mesh_vert[v0:v1], float) @ Rg.T + cen
            blo, bhi = pts.min(axis=0), pts.max(axis=0)
        elif gtype in (int(mujoco.mjtGeom.mjGEOM_BOX),
                       int(mujoco.mjtGeom.mjGEOM_SPHERE),
                       int(mujoco.mjtGeom.mjGEOM_CAPSULE),
                       int(mujoco.mjtGeom.mjGEOM_CYLINDER)):
            ext = np.abs(Rg) @ sz
            pts = cen + np.array([[sx * ext[0], sy * ext[1], sz2 * ext[2]]
                                  for sx in (-1, 1) for sy in (-1, 1)
                                  for sz2 in (-1, 1)])
            blo, bhi = pts.min(axis=0), pts.max(axis=0)
        else:
            blo, bhi = cen, cen
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

    # ---- failure windows (trace analysis only) -------------------------
    # The signature definitions and the regime rule below are deliberately
    # narrow: they read the per-step recorder arrays only.  They never touch
    # the success predicate, the reward, or the live sim, and they never
    # command anything.
    FAILURE_SIGNALS = ('obj_z', 'obj_xy', 'hand_contacts', 'base_tilt',
                       'joint_track_err')
    FAILURE_RULES = ('below', 'above', 'drop_by', 'rise_by')

    @staticmethod
    def _joint_candidates(nm):
        """Model-name candidates for one contract joint name.

        The control contract uses bare names; the model puts hand joints
        under the gripper namespace (``gripper0_left_<name>`` for
        ``left_hand_*``, ``gripper0_right_<name>`` for ``right_hand_*``)
        and arm joints under ``robot0_<name>``.  Same candidate order as
        hmg_tools.
        """
        cands = []
        if nm.startswith('left_hand_'):
            cands.append('gripper0_left_' + nm)
        elif nm.startswith('right_hand_'):
            cands.append('gripper0_right_' + nm)
        cands += ['robot0_' + nm, nm]
        return cands

    def _failure_signal(self, signal, joints=None):
        """Return (values, meta).  ``meta`` carries a reference or an error."""
        n = len(self.rec.t)
        if signal == 'obj_z':
            vals = [np.nan if o is None else
                    float(np.asarray(o, float)[2]) for o in self.rec.obj]
            return np.asarray(vals, float), {}
        if signal == 'obj_xy':
            pts = [None if o is None else np.asarray(o, float)[:2]
                   for o in self.rec.obj]
            ref = None
            for p in pts:
                if p is not None:
                    ref = np.asarray(p, float)
                    break
            if ref is None:
                return np.full(n, np.nan), {'reference': None}
            vals = [np.nan if p is None else
                    float(np.linalg.norm(np.asarray(p, float) - ref))
                    for p in pts]
            return np.asarray(vals, float), {
                'reference': [round(float(ref[0]), 5),
                              round(float(ref[1]), 5)],
                'origin': 'first valid obj xy in the search range'}
        if signal == 'hand_contacts':
            return np.asarray([np.nan if h is None else float(h)
                               for h in self.rec.hand_n], float), {}
        if signal == 'base_tilt':
            return np.asarray(self.rec.tilt, float), {}
        if signal == 'joint_track_err':
            from .contract import ALLOWED_UPPER
            # The waist is observed_exogenous (HOMIE is its final writer),
            # so its commanded value is not a tracking target by default.
            # The hand joints live under the gripper namespace, so the
            # default set is the arm joints (resolvable as robot0_<name>).
            names = [nm for nm in ALLOWED_UPPER
                     if not nm.startswith('waist')
                     and '_hand_' not in nm]
            if joints:
                bad = [j for j in joints if j not in ALLOWED_UPPER]
                if bad:
                    return None, {'error': 'unknown joints %s (choose from '
                                  'ALLOWED_UPPER)' % bad}
                names = [str(j) for j in joints]
            import mujoco
            adr = []
            resolved = []
            for nm in names:
                jid = -1
                for cand in self._joint_candidates(nm):
                    jid = int(mujoco.mj_name2id(
                        self.m, mujoco.mjtObj.mjOBJ_JOINT, cand))
                    if jid >= 0:
                        break
                if jid < 0:
                    return None, {'error': 'joint not in model: %s'
                                  % nm}
                adr.append(int(self.m.jnt_qposadr[jid]))
                resolved.append(nm)
            ci = [ALLOWED_UPPER.index(nm) for nm in names]
            vals = np.full(n, np.nan)
            for k in range(n):
                try:
                    q = np.asarray(self.rec.q[k], float)
                    a = np.asarray(self.rec.a[k], float)
                except Exception:
                    continue
                err = np.abs(q[adr] - a[ci])
                if err.size:
                    vals[k] = float(np.max(err))
            return vals, {'joints': names,
                          'definition': 'max over joints of |qpos - commanded|'
                                        ' (rad); waist excluded by default'}
        return None, {'error': 'unknown signal %r (choose from %s)'
                      % (signal, list(self.FAILURE_SIGNALS))}

    @staticmethod
    def _failure_cond(vals, rule, threshold, delta):
        """Boolean condition per step; NaN never satisfies a rule."""
        ok = np.isfinite(vals)
        if rule == 'below':
            return ok & (vals <= float(threshold))
        if rule == 'above':
            return ok & (vals >= float(threshold))
        # Change rules compare against the running extremum of the values
        # already seen, so the anchor is always earlier than the crossing.
        cond = np.zeros(vals.shape[0], bool)
        anchor = np.nan
        for k in range(vals.shape[0]):
            v = vals[k]
            if not np.isfinite(v):
                continue
            if not np.isfinite(anchor):
                anchor = v
                continue
            if rule == 'drop_by':
                cond[k] = bool(v <= anchor - float(delta))
                anchor = max(anchor, v)
            else:  # rise_by
                cond[k] = bool(v >= anchor + float(delta))
                anchor = min(anchor, v)
        return cond

    def failure_window(self, signal='obj_z', rule='below', threshold=None,
                       delta=None, hold=1, t0=None, t1=None, pad=0.1,
                       max_samples=30, joints=None):
        """Onset/end of a failure signature in the recorded trace.

        ``rule`` is one of ``below``/``above`` (absolute ``threshold``) or
        ``drop_by``/``rise_by`` (``delta`` against the running extremum).
        ``hold`` is the number of consecutive steps that must satisfy the
        condition to confirm the onset (and, symmetrically, the recovery).
        When the condition never recovers before the last recorded step the
        window is ``terminal`` and ``end_step`` is that last step.  Only the
        recorder's per-step arrays are read; no success/reward is touched.
        """
        if self.rec is None:
            return {'recording': False}
        if not self.rec.t:
            return {'recording': True, 'error': 'no recorded steps'}
        if signal not in self.FAILURE_SIGNALS:
            return {'recording': True, 'error':
                    'unknown signal %r (choose from %s)'
                    % (signal, list(self.FAILURE_SIGNALS))}
        if rule not in self.FAILURE_RULES:
            return {'recording': True, 'error':
                    'unknown rule %r (choose from %s)'
                    % (rule, list(self.FAILURE_RULES))}
        if rule in ('below', 'above'):
            if threshold is None:
                return {'recording': True, 'error':
                        'rule %r needs an absolute threshold' % rule}
        elif delta is None:
            return {'recording': True, 'error':
                    'rule %r needs a delta' % rule}
        vals, meta = self._failure_signal(signal, joints=joints)
        if vals is None:
            return {'recording': True, 'error': meta.get('error', 'bad signal'),
                    'signal': signal}
        tt = np.asarray(self.rec.t, float)
        in_range = np.ones(tt.shape[0], bool)
        if t0 is not None:
            in_range &= (tt >= float(t0))
        if t1 is not None:
            in_range &= (tt <= float(t1))
        sel = np.where(in_range)[0]
        if sel.size == 0:
            return {'recording': True, 'error':
                    'no recorded steps in [%r, %r]' % (t0, t1),
                    'signal': signal}
        cond_all = self._failure_cond(vals, rule, threshold, delta)
        cond = cond_all[sel]
        n_sel = int(sel.size)
        hold_n = max(1, int(hold))

        # First confirmed crossing (hold consecutive satisfied steps).
        onset_k = None
        for a in range(n_sel):
            if not cond[a]:
                continue
            if a + hold_n > n_sel:
                break
            if bool(np.all(cond[a:a + hold_n])):
                onset_k = a
                break

        def _joint_detail(j):
            if signal != 'joint_track_err':
                return None
            import mujoco
            from .contract import ALLOWED_UPPER
            q = np.asarray(self.rec.q[int(j)], float)
            a = np.asarray(self.rec.a[int(j)], float)
            out = {}
            for nm in meta.get('joints') or []:
                jid = -1
                for cand in self._joint_candidates(nm):
                    jid = int(mujoco.mj_name2id(
                        self.m, mujoco.mjtObj.mjOBJ_JOINT, cand))
                    if jid >= 0:
                        break
                ci = ALLOWED_UPPER.index(nm)
                qa = int(self.m.jnt_qposadr[int(jid)])
                out[nm] = round(float(abs(q[qa] - a[ci])), 6)
            return out

        def _anchor(onset_k):
            # The value the change rule compared against when it crossed.
            if rule in ('drop_by', 'rise_by'):
                hist = vals[sel[:onset_k + 1]]
                hist = hist[np.isfinite(hist)]
                if not hist.size:
                    return None
                v = (np.max(hist) if rule == 'drop_by' else np.min(hist))
                return round(float(v), 6)
            if onset_k <= 0:
                return None
            return _row(int(sel[onset_k - 1]))['value']

        def _row(j):
            j = int(j)
            row = {'step': j, 't': round(float(tt[j]), 5),
                   'value': (None if not np.isfinite(vals[j])
                             else round(float(vals[j]), 6))}
            objs = {'obj': self.rec.obj, 'obj_lo': self.rec.obj_lo,
                    'obj_hi': self.rec.obj_hi}
            for key, src in objs.items():
                v = src[j] if j < len(src) else None
                row[key] = (None if v is None else
                            [round(float(x), 6) for x in
                             np.asarray(v, float).ravel()])
            row['base_z'] = (None if self.rec.base_z[j] is None else
                             round(float(self.rec.base_z[j]), 6))
            row['tilt'] = round(float(self.rec.tilt[j]), 6)
            hn = self.rec.hand_n[j] if j < len(self.rec.hand_n) else None
            row['hand_n'] = None if hn is None else int(hn)
            return row

        def _series(center):
            if center is None:
                return None
            pre = max(1, int(round(float(pad) / max(self.dt, 1e-9))))
            lo = max(0, int(center) - pre)
            hi = min(tt.shape[0] - 1, int(center) + pre)
            js = list(range(lo, hi + 1))
            cap = max(1, int(max_samples))
            if len(js) > cap:
                stride = int(np.ceil(len(js) / float(cap)))
                js = js[::stride]
                if int(center) not in js:
                    js.append(int(center))
                    js.sort()
            return [_row(j) for j in js]

        base = {'recording': True, 'signal': signal, 'rule': rule,
                'threshold': (None if threshold is None else
                              float(threshold)),
                'delta': None if delta is None else float(delta),
                'hold': hold_n, 'pad': float(pad),
                'n_steps': int(tt.shape[0]),
                'range': {'t0': None if t0 is None else float(t0),
                          't1': None if t1 is None else float(t1),
                          'n': n_sel},
                'signal_definition': meta}
        if onset_k is None:
            base.update({'onset_step': None, 'onset_t': None,
                         'onset_value': None, 'end_step': None,
                         'end_t': None, 'end_value': None,
                         'terminal': False, 'recovered': False,
                         'series_around_onset': None,
                         'series_around_end': None,
                         'note': 'condition never held for %d consecutive '
                                 'step(s)' % hold_n})
            return base

        # Recovery: first run of hold_n non-failing steps after the onset.
        recov_k = None
        last_fail_k = onset_k + hold_n - 1
        b = onset_k + hold_n
        while b < n_sel:
            if cond[b]:
                last_fail_k = b
                b += 1
                continue
            run = 0
            c = b
            while c < n_sel and not cond[c]:
                run += 1
                c += 1
            if run >= hold_n:
                recov_k = b
                break
            b = c
        g_on = int(sel[onset_k])
        if recov_k is None:
            g_end = int(sel[n_sel - 1])
            g_last_fail = int(sel[last_fail_k])
            terminal = True
        else:
            g_end = int(sel[recov_k])
            g_last_fail = int(sel[last_fail_k])
            terminal = False
        base.update({
            'onset_step': g_on,
            'onset_t': round(float(tt[g_on]), 5),
            'onset_value': _row(g_on)['value'],
            'end_step': g_end,
            'end_t': round(float(tt[g_end]), 5),
            'end_value': _row(g_end)['value'],
            'last_failure_step': g_last_fail,
            'last_failure_t': round(float(tt[g_last_fail]), 5),
            'terminal': bool(terminal),
            'recovered': not bool(terminal),
            'onset_anchor': _anchor(onset_k),
            'series_around_onset': _series(g_on),
            'series_around_end': _series(g_end),
        })
        detail = _joint_detail(g_on)
        if detail is not None:
            base['onset_joint_err'] = detail
        detail = _joint_detail(g_end)
        if detail is not None:
            base['end_joint_err'] = detail
        return base

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
    def _render_sequence(self, idxs, width, height, lookat, dist, azim,
                         elev, path_for):
        """Render recorded qpos snapshots on a SCRATCH MjData.

        The live sim is never written.  ``path_for(k, step, t)`` names each
        PNG.  Returns ``(frames, paths)``.
        """
        import mujoco
        rec_t = np.asarray(self.rec.t, float)
        qs = self.rec.q
        r = mujoco.Renderer(self.m, int(height), int(width))
        cam = mujoco.MjvCamera()
        cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
        lookat_obj = isinstance(lookat, str) and lookat == 'object'
        if lookat_obj:
            if self.obj_bid is not None:
                cam.lookat[:] = np.asarray(self.d.xpos[self.obj_bid], float)
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
                ds = mujoco.MjData(self.m)
                ds.qpos[:] = np.asarray(qs[j], float)
                mujoco.mj_forward(self.m, ds)  # scratch: live data untouched
                r.update_scene(ds, camera=cam)
                img = np.asarray(r.render())
                frames.append(np.array(img[:, :, :3], copy=True))
                p = os.path.join(self.dir, path_for(k, int(j),
                                                   float(rec_t[j])))
                try:
                    import cv2
                    cv2.imwrite(p, frames[-1])
                except Exception:
                    from PIL import Image
                    Image.fromarray(frames[-1]).save(p)
                paths.append(p)
        finally:
            r.close()
        return frames, paths

    def _stills_at_times(self, req_t=None, t_range=None, width=640,
                         height=480, lookat=(1.7, 0.0, 0.9), dist=1.2,
                         azim=-90.0, elev=15.0, mp4=False, fps=10,
                         prefix='still'):
        """Render the recorded frame nearest each requested time."""
        t = np.asarray(self.rec.t, float)
        if t.size == 0:
            return {'mode': 'times', 'recording': True,
                    'error': 'no recorded steps', 'frames': []}
        reqs = []
        if t_range is not None:
            try:
                rng = [float(x) for x in t_range]
                if len(rng) == 2:
                    rng = [rng[0], rng[1], 2]
                n = max(1, int(rng[2]))
                reqs.extend(np.linspace(rng[0], rng[1], n).tolist())
            except Exception as exc:
                return {'mode': 'times', 'recording': True,
                        'error': 'bad t_range: %s: %s'
                                 % (type(exc).__name__, exc), 'frames': []}
        if req_t is not None:
            vals = (req_t if isinstance(req_t, (list, tuple, np.ndarray))
                    else [req_t])
            try:
                reqs.extend([float(x) for x in vals])
            except Exception as exc:
                return {'mode': 'times', 'recording': True,
                        'error': 'bad t: %s: %s'
                                 % (type(exc).__name__, exc), 'frames': []}
        if not reqs:
            return {'mode': 'times', 'recording': True,
                    'error': 'no times requested', 'frames': []}
        idxs = [int(np.argmin(np.abs(t - float(x)))) for x in reqs]
        frames, paths = self._render_sequence(
            idxs, width, height, lookat, dist, azim, elev,
            lambda k, j, tt: '%s_t%.3f_%02d.png' % (prefix, tt, k))
        out = {'mode': 'times', 'recording': True, 'frames': paths,
               't': [round(float(t[j]), 5) for j in idxs],
               'requested_t': [round(float(x), 5) for x in reqs],
               'steps': idxs, 'dt': round(float(self.dt), 5)}
        if mp4 and frames:
            try:
                import cv2
                p = os.path.join(self.dir, '%s_times_clip.mp4' % prefix)
                h, w = frames[0].shape[:2]
                vw = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*'mp4v'),
                                     float(fps), (w, h))
                for fr in frames:
                    vw.write(fr[:, :, ::-1])
                vw.release()
                out['mp4'] = p
            except Exception as exc:
                out['mp4_error'] = '%s: %s' % (type(exc).__name__, exc)
        return out

    def stills(self, kind=None, index=0, pad=0.2, n=3, width=640,
               height=480, lookat=(1.7, 0.0, 0.9), dist=1.2, azim=-90.0,
               elev=15.0, mp4=False, fps=10, prefix='still',
               t=None, t_range=None):
        """Render frames on SCRATCH MjData (the live sim is not modified).

        Event mode (default; unchanged): n frames around one recorder event,
        selected by ``kind``/``index`` over ``[t_event - pad, t_event + pad]``.
        Time mode (new, additive): pass ``t=<s>``, ``t=[t1, t2, ...]`` or
        ``t_range=(t0, t1, n)`` to render the recorded qpos nearest each
        requested time.  Returns the PNG paths (and an mp4 path if asked).
        """
        if self.rec is None:
            return {'recording': False}
        if t is not None or t_range is not None:
            return self._stills_at_times(
                req_t=t, t_range=t_range, width=width, height=height,
                lookat=lookat, dist=dist, azim=azim, elev=elev, mp4=mp4,
                fps=fps, prefix=prefix)
        i, ev = self.rec._event(kind, index)
        if ev is None:
            return {'event': None, 'n_events': len(self.rec.events)}
        tt = np.asarray(self.rec.t, float)
        qs = self.rec.q
        if not qs:
            return {'event': ev, 'error': 'no recorded steps'}
        lo_t, hi_t = ev['t'] - float(pad), ev['t'] + float(pad)
        sel = np.where((tt >= lo_t) & (tt <= hi_t))[0]
        if sel.size == 0:
            sel = np.array([int(np.argmin(np.abs(tt - ev['t'])))])
        pick = np.linspace(0, sel.size - 1, max(1, int(n))).astype(int)
        idxs = [int(sel[k]) for k in pick]
        frames, paths = self._render_sequence(
            idxs, width, height, lookat, dist, azim, elev,
            lambda k, j, tval: '%s_%s_%02d_t%.3f.png' % (
                prefix, ev['kind'], k, tval))
        out = {'event': ev, 'frames': paths,
               't': [float(tt[j]) for j in idxs]}
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
