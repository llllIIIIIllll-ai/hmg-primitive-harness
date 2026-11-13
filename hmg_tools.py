"""Trusted primitives the actor composes instead of inventing joint values.

reach_box / close_grip / lift run the DLS IK and the WBC goal inside the one
persistent env.  Each returns a compact status the LLM can reason about.

Harness fixes (2026-11-09, verified against the live model):
  * Contract joint names (``left_shoulder_pitch_joint``) are resolved to the
    simulator's prefixed names (``robot0_left_shoulder_pitch_joint``,
    ``gripper0_left_left_hand_index_0_joint``) through ``self.addr``.  Before
    this the lookup always missed and every commanded upper-body target
    silently became 0 (runtime evidence: ``_upper_vec()`` returned 31 zeros
    while 28 prefixed joints were found).
  * The 31-D contract order is permuted into the WBC policy's dof order
    (``contract.PERM_TO_WBC``) at the env boundary; the policy indexes
    ``target_upper_body_pose`` with its own pinocchio order, which puts the
    left hand before the right arm.
  * ``reach_box`` now actually applies the IK solution (it used to compute it
    and then ramp to the unmodified current vector) and reports the achieved
    palm residual after the ramp, not the solver residual.
  * ``close_grip`` closes each finger towards its larger joint-range side
    (matches the reference ``grasp_qpos`` in
    models/grippers/g1_threefinger_hands.py); the old low-limit-sign rule
    flipped thumb_1 (left) and thumb_2 (both hands).
"""
import os

import numpy as np

from hmg_harness.contract import ALLOWED_UPPER, PERM_TO_WBC
from hmg_harness.fullbody import WBCGoal

# ---------------------------------------------------------------------------
# DLS IK fast path (2026-11-09, performance-only change).
#
# Profiling a representative primitive sequence showed that ~60% of the IK
# wall time sat inside ``mujoco.mj_forward`` on the scratch MjData (143 us per
# call on 01_box_lift_floor: mj_forward also runs collision detection and
# constraint assembly, none of which the IK reads) and most of the rest in
# the allocation-heavy numpy DLS block.  Two behaviour-preserving changes:
#   * the scratch-data update uses ``mj_kinematics`` + ``mj_comPos``, the
#     first two stages of mj_forward.  Every quantity the IK reads
#     (xpos/xquat/xmat, site_xpos/site_xmat, jacobians) is BIT-IDENTICAL to
#     the mj_forward result (verified), at 143 us -> 7.6 us per iteration.
#   * the DLS block reuses preallocated buffers and hoists the MjData views,
#     keeping every arithmetic operation in the same order, so its output is
#     bit-identical to the original code (verified over 64 real solves).
# ``_ik_arm_numpy`` is the original implementation verbatim; HMG_IK_LEGACY=1
# forces it.
#
# HMG_IK_NUMBA=1 additionally runs the DLS block in a numba kernel (20x faster
# than the numpy block).  It is OPT-IN and off by default because its result
# is only numerically equivalent, not bit-identical: BLAS/LAPACK use FMA and
# partial pivoting, so per-iteration differences of ~1e-17 amplify to up to
# 5e-5 rad on far/unreachable IK targets (converged targets stay <=2e-14;
# convergence decisions were identical in every test case).  That is far
# below any physical effect but not the bit-identical guarantee the default
# path gives, so it must be requested explicitly.
# ---------------------------------------------------------------------------
IK_LEGACY = os.environ.get('HMG_IK_LEGACY', '0') == '1'
IK_NUMBA = os.environ.get('HMG_IK_NUMBA', '0') == '1'

if IK_NUMBA:
    try:
        from numba import njit as _njit
    except Exception:                   # numba missing/incompatible
        _njit = None
else:
    _njit = None

if _njit is not None:
    @_njit(cache=True)
    def _dls_step(jp, jr, dof, spos, sxmat, tgt, R_des, use_rot, q, lo, hi):
        """One damped-least-squares IK step, in place on ``q``.

        Same arithmetic as the numpy reference in ``_ik_arm_numpy``:
        J = [jp[:, dof]; jr[:, dof]], e = [palm-position error; rotation
        error], dq = J^T (J J^T + 1e-2 I)^-1 e, |dq| capped at 0.08 rad,
        q clipped to the joint limits.  Returns (|ep|, |er|, cost).
        """
        e = np.empty(6)
        e[0] = tgt[0] - spos[0]
        e[1] = tgt[1] - spos[1]
        e[2] = tgt[2] - spos[2]
        e[3] = 0.0
        e[4] = 0.0
        e[5] = 0.0
        nrow = 3
        J = np.empty((6, 7))
        for j in range(7):
            dj = dof[j]
            J[0, j] = jp[0, dj]
            J[1, j] = jp[1, dj]
            J[2, j] = jp[2, dj]
            J[3, j] = jr[0, dj]
            J[4, j] = jr[1, dj]
            J[5, j] = jr[2, dj]
        if use_rot:
            # Re = R_des @ Rc^T with Rc = reshape(sxmat, 3, 3)
            Re = np.empty((3, 3))
            for i in range(3):
                for j in range(3):
                    s = 0.0
                    for k in range(3):
                        s += R_des[i, k] * sxmat[j * 3 + k]
                    Re[i, j] = s
            tr = Re[0, 0] + Re[1, 1] + Re[2, 2]
            c = (tr - 1.0) / 2.0
            if c > 1.0:
                c = 1.0
            elif c < -1.0:
                c = -1.0
            ang = np.arccos(c)
            if ang > 1e-8:
                f = ang / (2.0 * np.sin(ang))
                e[3] = f * (Re[2, 1] - Re[1, 2])
                e[4] = f * (Re[0, 2] - Re[2, 0])
                e[5] = f * (Re[1, 0] - Re[0, 1])
            nrow = 6
        # A = J J^T + 1e-2 I
        A = np.zeros((6, 6))
        for i in range(nrow):
            for j in range(nrow):
                s = 0.0
                for k in range(7):
                    s += J[i, k] * J[j, k]
                A[i, j] = s
        for i in range(nrow):
            A[i, i] += 1e-2
        # solve A y = e by partial-pivot Gaussian elimination
        M = np.empty((6, 7))
        for i in range(nrow):
            for j in range(nrow):
                M[i, j] = A[i, j]
            M[i, nrow] = e[i]
        for col in range(nrow):
            piv = col
            best = abs(M[col, col])
            for r in range(col + 1, nrow):
                v = abs(M[r, col])
                if v > best:
                    best = v
                    piv = r
            if piv != col:
                for j in range(col, nrow + 1):
                    tmp = M[col, j]
                    M[col, j] = M[piv, j]
                    M[piv, j] = tmp
            pivval = M[col, col]
            for r in range(col + 1, nrow):
                f = M[r, col] / pivval
                for j in range(col, nrow + 1):
                    M[r, j] -= f * M[col, j]
        y = np.empty(6)
        for i in range(nrow - 1, -1, -1):
            s = M[i, nrow]
            for j in range(i + 1, nrow):
                s -= M[i, j] * y[j]
            y[i] = s / M[i, i]
        # dq = J^T y
        dq = np.empty(7)
        mx = 0.0
        for j in range(7):
            s = 0.0
            for i in range(nrow):
                s += J[i, j] * y[i]
            dq[j] = s
            a = abs(s)
            if a > mx:
                mx = a
        if mx > 0.08:
            sc = 0.08 / mx
            for j in range(7):
                dq[j] *= sc
        for j in range(7):
            v = q[j] + dq[j]
            if v < lo[j]:
                v = lo[j]
            if v > hi[j]:
                v = hi[j]
            q[j] = v
        epn = np.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
        ern = np.sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5])
        return epn, ern, epn + 0.2 * ern
else:                                   # pragma: no cover
    _dls_step = None

ARM = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
       'wrist_roll', 'wrist_pitch', 'wrist_yaw')
HAND = ('thumb_0', 'thumb_1', 'thumb_2', 'index_0', 'index_1',
        'middle_0', 'middle_1')
WAIST = ('waist_yaw', 'waist_roll', 'waist_pitch')


def task_object(env):
    """(objects, primary) for the task env.

    Multi-object scenes expose ``mujoco_objects`` in declaration order, which
    is NOT the task object (e.g. 03_box_lift lists the static table first).
    The innermost env also declares the task's primary object through
    ``get_object()`` (``{'obj': {'obj_name': 'obj_main', ...}}``), so select
    the SceneObject whose ``root_body`` matches it.  If the declared body is a
    child of a SceneObject (e.g. the button on 02_push_button's control
    panel), walk the model's body ancestry to the owning SceneObject.  Falls
    back to the first object for single-object tasks / if nothing matches.
    """
    inner, objs, name = env, [], None
    for _ in range(6):
        if not objs:
            objs = getattr(inner, 'mujoco_objects', None) or objs
        if name is None:
            go = getattr(inner, 'get_object', None)
            if callable(go):
                try:
                    d = go()
                    if d:
                        name = next(iter(d.values())).get('obj_name')
                except Exception:
                    name = None
        if objs:
            break
        inner = getattr(inner, 'env', None)
        if inner is None:
            break
    if not objs:
        return [], None
    if name:
        for o in objs:
            if getattr(o, 'root_body', None) == name:
                return objs, o
        # The declared task body may be a child of a SceneObject (e.g. the
        # button on 02_push_button's control panel): walk the model body
        # ancestry until a SceneObject root is found.
        try:
            m = env.sim.model
            body = name
            for _ in range(8):
                bid = int(m.body_name2id(body))
                if bid < 0:
                    break
                for o in objs:
                    if getattr(o, 'root_body', None) == body:
                        return objs, o
                pid = int(m.body_parentid[bid])
                if pid <= 0:
                    break
                body = m.body(pid).name
        except Exception:
            pass
    return objs, objs[0]



class Tools:
    def __init__(self, env, state_fn):
        self.env = env
        self.state = state_fn           # () -> the observe() dict
        self.t = 0.0
        self.nav = np.zeros(3)
        self.height = 0.74
        self.last_cmd = None             # last contract-order 31-D target
        mm, dd = env.sim.model, env.sim.data
        m = getattr(mm, '_model', mm)
        self.raw = (m, getattr(dd, '_data', dd))

        import mujoco
        # every contract joint -> qpos/dof address in the sim model
        self.addr, self.dofadr, self.sim_name = {}, {}, {}
        for nm in ALLOWED_UPPER:
            cands = []
            if nm.startswith('left_hand_'):
                cands.append('gripper0_left_left_hand_' + nm[len('left_hand_'):])
            elif nm.startswith('right_hand_'):
                cands.append('gripper0_right_right_hand_' + nm[len('right_hand_'):])
            cands += ['robot0_' + nm, nm]
            jid = -1
            for cand in cands:
                jid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, cand))
                if jid >= 0:
                    break
            if jid < 0:
                raise RuntimeError('contract joint %r not found in model '
                                   '(tried %s)' % (nm, cands))
            j = m.joint(jid)
            self.addr[nm] = int(np.atleast_1d(j.qposadr)[0])
            self.dofadr[nm] = int(np.atleast_1d(j.dofadr)[0])
            self.sim_name[nm] = cand
        # legacy name-keyed maps (prefixed sim names) for debug output
        self.jq = {self.sim_name[nm]: self.addr[nm] for nm in ALLOWED_UPPER}
        self.jd = {self.sim_name[nm]: self.dofadr[nm] for nm in ALLOWED_UPPER}
        # contract-order qpos addresses, for a vectorised _upper_vec()
        self.qpos_addr = np.array([self.addr[nm] for nm in ALLOWED_UPPER])
        self.sid = {}
        for s_ in ('left', 'right'):
            for cand in ('gripper0_%s_grip_site' % s_, '%s_eef_target' % s_):
                i = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, cand))
                if i >= 0:
                    self.sid[s_] = i
                    break
        # capture the grip-site orientation at the reset pose, expressed in
        # the robot body frame.  The hand mount is rigid, so a "fingers down
        # by 90 deg, palm normal unchanged" target is R_y(90) applied to this.
        mujoco.mj_forward(m, self.raw[1])
        qb0 = np.asarray(self.raw[1].qpos[3:7], float)
        self.R_base0 = self._quat_to_mat(qb0)
        self.R_site0 = {}
        for s_, sid in self.sid.items():
            self.R_site0[s_] = (self.R_base0.T @ np.asarray(
                self.raw[1].site_xmat[sid], float).reshape(3, 3))

    @staticmethod
    def _quat_to_mat(q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    # ---- low level -----------------------------------------------------
    def observe(self):
        """The documented state snapshot (PROTOCOL.md: observe).  Previously
        absent, so any primitive calling ``api.observe()`` died with
        AttributeError: 'Tools' object has no attribute 'observe' (verified
        before the fix on 01_box_lift_floor)."""
        return self.state()

    def joints(self):
        """name -> current value for every contract joint (observed)."""
        q = self.env.sim.data.qpos
        out = {}
        for nm in self.jq:
            out[nm] = round(float(q[self.jq[nm]]), 3)
        return out

    def _send(self, upper, nav=None, height=None):
        """One control step.  `upper` is in contract (ALLOWED_UPPER) order;
        the model/policy needs WBC dof order."""
        nv = self.nav if nav is None else np.asarray(nav, float)
        # the same clamp set_height enforces: a raw step_goal(height=0.98)
        # used to bypass it and the verifier flagged that as an undeclared
        # model-limit violation.  Lower bound relaxed 0.50 -> 0.44 on
        # 2026-11-09 (actor_1, 01_box_lift_floor): no base-height limit is
        # declared anywhere in the task/sim package (grep: none), and the
        # comment below records a measured crouch to z=0.393 that the WBC DID
        # track.  The floor box (0.555 m tall) is 3-5 cm beyond arm reach from
        # a 0.50 m crouch, so this is the difference between reaching its side
        # faces and not.  Joint/model limits are untouched.
        h = float(np.clip(self.height if height is None else float(height),
                          0.44, 0.90))
        # The WBC keeps executing the last height command, so the Tools-side
        # bookkeeping must follow it: step_goal(height=h) previously sent h
        # for that call only, and a later reach_box/_ramp (which passes
        # height=None) silently reverted to the stale self.height.  Measured
        # on 01_box_lift_floor: a step_goal crouch to 0.38 left the base at
        # z=0.393, then reach_box stood the robot back up to z~0.75.
        self.height = h
        u = np.asarray(upper, float)
        if u.shape != (31,):
            raise ValueError('upper target must be 31-D, got %s' % (u.shape,))
        self.last_cmd = u.copy()          # contract order, for hold-and-lift
        self.env.step_goal(WBCGoal(upper=u[PERM_TO_WBC], nav=nv, height=h))
        self.t += self.env.control_timestep

    def step_goal(self, upper=None, nav=None, height=None, dur=0.2):
        u = self._upper_vec() if upper is None else np.asarray(upper, float)
        nav0 = nav
        n = max(1, int(dur / self.env.control_timestep))
        for _ in range(n):
            self._send(u, nav0, height)
        return {'t': round(self.t, 2)}

    def upper(self):
        """Current 31 upper-body joint positions in contract order."""
        return self._upper_vec()

    def commanded(self):
        """Last 31-D target actually sent (contract order) or None.  Lets a
        primitive hold an explicit grip target while changing base height
        instead of re-reading the contact-limited measured fingers."""
        return None if self.last_cmd is None else self.last_cmd.copy()

    def _upper_vec(self):
        qpos = self.env.sim.data.qpos
        return qpos[self.qpos_addr]

    def _apply(self, upper, dur=1.0):
        n = max(1, int(dur / self.env.control_timestep))
        for k in range(n):
            self._send(upper)
        return n

    def _ramp(self, target, dur):
        cur = self._upper_vec()
        n = max(1, int(dur / self.env.control_timestep))
        for k in range(n):
            a = min(1., (k + 1) / n)
            a = a * a * (3 - 2 * a)
            self._send(cur * (1 - a) + target * a)
        return n

    # ---- diagnostics ---------------------------------------------------
    def joint_ranges(self):
        """Read-only joint limits for the 31 contract joints (added 2026-11-09,
        actor_2).  Needed to interpret which commands a joint can actually
        follow (e.g. a finger joint whose range is one-sided clips a positive
        command to 0) and to choose close_grip signs.  Reads model limits
        only; it never changes them."""
        m = self.raw[0]
        import mujoco
        out = {}
        q = self.env.sim.data.qpos
        for nm in ALLOWED_UPPER:
            jid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,
                                        self.sim_name[nm]))
            lo, hi = m.jnt_range[jid]
            out[nm] = {'lo': round(float(lo), 3), 'hi': round(float(hi), 3),
                       'q': round(float(q[self.addr[nm]]), 3)}
        return out

    def hand_contacts_side(self):
        """Per-side hand-object contact count (added 2026-11-09, actor_2).
        ``observe()['hand_object_contacts']`` only gives the total; a two-hand
        grasp needs to know whether each hand actually touched the box before
        closing.  Counts sim contacts whose other body is a left/right hand
        body.  Same information class as the documented contact count."""
        m, d = self.raw
        objs, sel = task_object(self.env)
        if sel is None:
            return {'left': 0, 'right': 0}
        import mujoco
        bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                    sel.root_body))
        out = {'left': 0, 'right': 0, 'pairs': []}
        for ci in range(int(d.ncon)):
            c = d.contact[ci]
            b1 = int(m.geom_bodyid[c.geom1])
            b2 = int(m.geom_bodyid[c.geom2])
            if bid not in (b1, b2):
                continue
            other = b2 if b1 == bid else b1
            nm = m.body(other).name
            if not ('hand' in nm or 'gripper' in nm):
                continue          # knee/torso contacts are not a grasp
            side = 'left' if '_left_' in nm else ('right' if '_right_' in nm else None)
            if side is None:
                continue
            out[side] += 1
            out['pairs'].append([nm, m.geom(int(c.geom1)).name,
                                 m.geom(int(c.geom2)).name])
        return out

    def base_state(self):
        """Base pose in world: position, yaw and tilt (for feedback walking)."""
        d = self.raw[1]
        p = np.asarray(d.qpos[:3], float)
        R = self._quat_to_mat(np.asarray(d.qpos[3:7], float))
        return {'pos': np.round(p, 4).tolist(),
                'yaw_deg': round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 2),
                'tilt_deg': round(float(np.degrees(np.arccos(
                    np.clip(R[2, 2], -1.0, 1.0)))), 2)}

    def geom(self):
        """Ground-truth geometry for debugging: palm sites and the object
        body pose (position, rotation, up alignment, local box top).  Reads
        the model only; no reward/state of the predicate."""
        m, d = self.raw
        import mujoco
        mujoco.mj_forward(m, d)
        out = {'sites': {}, 'object': None, 'hand_bodies': {}}
        for i in range(int(m.nbody)):
            nm = m.body(i).name
            if 'hand' in nm and ('palm' in nm or '_1_link' in nm
                                or 'thumb_2_link' in nm):
                out['hand_bodies'][nm] = np.round(d.xpos[i], 4).tolist()
        # robot link positions for reach planning (read-only, added actor_2):
        # shoulders/elbows/feet decide whether a grasp height is reachable.
        out['robot_bodies'] = {}
        for i in range(int(m.nbody)):
            nm = m.body(i).name
            if nm.startswith(('robot0', 'gripper0')) and any(
                    k_ in nm for k_ in ('torso', 'pelvis', 'waist',
                                        'shoulder', 'elbow', 'knee',
                                        'ankle', 'foot')):
                out['robot_bodies'][nm] = np.round(d.xpos[i], 4).tolist()
        for s_ in ('left', 'right'):
            sid = self.sid.get(s_)
            if sid is None:
                continue
            out['sites'][s_] = {
                'pos': np.round(d.site_xpos[sid], 4).tolist(),
                'xmat': np.round(d.site_xmat[sid].reshape(3, 3), 3).tolist()}
        objs, sel = task_object(self.env)
        if sel is not None:
            bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                        sel.root_body))
            R = np.asarray(d.xmat[bid]).reshape(3, 3)
            top = 0.0
            for gi in range(m.ngeom):
                if int(m.geom_bodyid[gi]) != bid:
                    continue
                if int(m.geom_type[gi]) == int(mujoco.mjtGeom.mjGEOM_BOX):
                    top = max(top, float(m.geom_pos[gi][2] + m.geom_size[gi][2]))
            out['object'] = {
                'body': sel.root_body,
                'pos': np.round(d.xpos[bid], 4).tolist(),
                'xmat': np.round(R, 3).tolist(),
                'z_up': round(float(R[2, 2]), 4),
                'local_top': round(top, 4),
                'mass': round(float(m.body_mass[bid]), 5)}
        return out

    def geom_detail(self):
        """Read-only geometry dump for one object body (added 2026-11-09,
        actor_2): enumerate every geom of the object root body with its type,
        local pose/size and world AABB, plus the world-space extent of the
        object.  Purpose: ``geom()`` reported ``local_top=0.5565`` while the
        body origin sits at z~=0, which is ambiguous between a 0.55 m tall
        crate and a small box plus an offset geom; the grasp must not be
        planned on a guess.  Reads model geometry only (no reward/predicate).
        """
        m, d = self.raw
        import mujoco
        mujoco.mj_forward(m, d)
        objs, sel = task_object(self.env)
        if sel is None:
            return {'error': 'no object'}
        bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                    sel.root_body))
        geoms, lo, hi = [], None, None
        for gi in range(m.ngeom):
            if int(m.geom_bodyid[gi]) != bid:
                continue
            gtype = int(m.geom_type[gi])
            tname = {int(mujoco.mjtGeom.mjGEOM_BOX): 'box',
                     int(mujoco.mjtGeom.mjGEOM_SPHERE): 'sphere',
                     int(mujoco.mjtGeom.mjGEOM_CAPSULE): 'capsule',
                     int(mujoco.mjtGeom.mjGEOM_CYLINDER): 'cylinder',
                     int(mujoco.mjtGeom.mjGEOM_MESH): 'mesh',
                     int(mujoco.mjtGeom.mjGEOM_PLANE): 'plane'}.get(gtype, str(gtype))
            # world AABB from the geom frame (box/sphere exact; mesh via
            # its local aabb, which is enough for planning)
            cen = np.asarray(d.geom_xpos[gi], float)
            Rg = np.asarray(d.geom_xmat[gi], float).reshape(3, 3)
            sz = np.asarray(m.geom_size[gi], float)
            if tname == 'mesh':
                # mujoco 3.2.6 has no mesh_aabb; use the mesh vertices
                aid = int(m.geom_dataid[gi])
                v0 = int(m.mesh_vertadr[aid])
                v1 = v0 + int(m.mesh_vertnum[aid])
                verts = np.asarray(m.mesh_vert[v0:v1], float)
                pts = verts @ Rg.T + cen
            else:
                ext = np.abs(Rg) @ sz
                pts = cen + np.array([[sx * ext[0], sy * ext[1], sz2 * ext[2]]
                                      for sx in (-1, 1) for sy in (-1, 1)
                                      for sz2 in (-1, 1)])
            blo, bhi = pts.min(axis=0), pts.max(axis=0)
            lo = blo if lo is None else np.minimum(lo, blo)
            hi = bhi if hi is None else np.maximum(hi, bhi)
            geoms.append({'i': gi, 'name': m.geom(gi).name, 'type': tname,
                          'local_pos': np.round(m.geom_pos[gi], 4).tolist(),
                          'local_size': np.round(sz, 4).tolist(),
                          'world_pos': np.round(cen, 4).tolist(),
                          'world_aabb_lo': np.round(blo, 4).tolist(),
                          'world_aabb_hi': np.round(bhi, 4).tolist(),
                          'mass': round(float(m.body_mass[bid]), 5),
                          'friction': np.round(m.geom_friction[gi], 3).tolist()})
        return {'body': sel.root_body, 'body_pos': np.round(
                    d.xpos[bid], 4).tolist(),
                'world_lo': None if lo is None else np.round(lo, 4).tolist(),
                'world_hi': None if hi is None else np.round(hi, 4).tolist(),
                'geoms': geoms}

    def scene_objects(self, detail=False):
        """Read-only inventory of every ``mujoco_objects`` SceneObject in the
        task env, with world body pose and geom AABB (added 2026-11-09,
        actor_1, 03_box_lift).  Purpose: 03_box_lift loads a STATIC SUPPORT
        (the server comment says the static table is listed first) next to the
        box; the walking stance must not collide with it, and until now the
        only readable object was the task's primary one.  Reads model
        geometry only (no reward/predicate), same class as geom_detail().
        ``detail=True`` also lists every geom of the object (type, local
        pose/size, world AABB, world position) -- needed when the body AABB
        spans more than the support surface (e.g. the 03 table AABB reaches
        z=1.27 although the box rests at z=0.80).
        """
        m, d = self.raw
        import mujoco
        mujoco.mj_forward(m, d)
        inner, objs = self.env, []
        for _ in range(6):
            got = getattr(inner, 'mujoco_objects', None)
            if got:
                objs = got
                break
            inner = getattr(inner, 'env', None)
            if inner is None:
                break
        out = []
        for o in objs:
            rb = getattr(o, 'root_body', None)
            if not rb:
                continue
            bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, rb))
            if bid < 0:
                continue
            lo = hi = None
            ngeom = 0
            geoms = []
            for gi in range(int(m.ngeom)):
                if int(m.geom_bodyid[gi]) != bid:
                    continue
                ngeom += 1
                cen = np.asarray(d.geom_xpos[gi], float)
                Rg = np.asarray(d.geom_xmat[gi], float).reshape(3, 3)
                sz = np.asarray(m.geom_size[gi], float)
                gtype = int(m.geom_type[gi])
                if gtype == int(mujoco.mjtGeom.mjGEOM_MESH):
                    aid = int(m.geom_dataid[gi])
                    v0 = int(m.mesh_vertadr[aid])
                    v1 = v0 + int(m.mesh_vertnum[aid])
                    pts = np.asarray(m.mesh_vert[v0:v1], float) @ Rg.T + cen
                elif gtype in (int(mujoco.mjtGeom.mjGEOM_BOX),
                               int(mujoco.mjtGeom.mjGEOM_SPHERE),
                               int(mujoco.mjtGeom.mjGEOM_CAPSULE),
                               int(mujoco.mjtGeom.mjGEOM_CYLINDER)):
                    ext = np.abs(Rg) @ sz
                    pts = cen + np.array([[sx * ext[0], sy * ext[1], sz2 * ext[2]]
                                          for sx in (-1, 1) for sy in (-1, 1)
                                          for sz2 in (-1, 1)])
                else:
                    pts = cen.reshape(1, 3)
                blo, bhi = pts.min(axis=0), pts.max(axis=0)
                lo = blo if lo is None else np.minimum(lo, blo)
                hi = bhi if hi is None else np.maximum(hi, bhi)
                if detail:
                    tname = {int(mujoco.mjtGeom.mjGEOM_BOX): 'box',
                             int(mujoco.mjtGeom.mjGEOM_SPHERE): 'sphere',
                             int(mujoco.mjtGeom.mjGEOM_CAPSULE): 'capsule',
                             int(mujoco.mjtGeom.mjGEOM_CYLINDER): 'cylinder',
                             int(mujoco.mjtGeom.mjGEOM_MESH): 'mesh',
                             int(mujoco.mjtGeom.mjGEOM_PLANE): 'plane'}.get(
                                 gtype, str(gtype))
                    geoms.append({'i': gi, 'name': m.geom(gi).name,
                                  'type': tname,
                                  'local_pos': np.round(m.geom_pos[gi], 4).tolist(),
                                  'local_size': np.round(sz, 4).tolist(),
                                  'world_pos': np.round(cen, 4).tolist(),
                                  'aabb_lo': np.round(blo, 4).tolist(),
                                  'aabb_hi': np.round(bhi, 4).tolist()})
            rec = {'name': str(getattr(o, 'name', rb)),
                   'root_body': rb, 'ngeom': ngeom,
                   'body_pos': np.round(d.xpos[bid], 4).tolist(),
                   'body_mass': round(float(m.body_mass[bid]), 5),
                   'aabb_lo': None if lo is None else np.round(lo, 4).tolist(),
                   'aabb_hi': None if hi is None else np.round(hi, 4).tolist()}
            if detail:
                rec['geoms'] = geoms
            out.append(rec)
        return out

    def hand_geoms(self):
        """Read-only: every geom attached to a hand body (palm/finger links)
        with type, local pose/size and world pose (added 2026-11-09, actor_1,
        03_box_lift).  The palm grip site sits exactly at the palm_link body
        origin, so the palm-pad collision geom's world offset from that body
        origin is the pad distance that decides how far outside a box face the
        palm site target must be.  Reads model geometry only (same class as
        geom()); no reward/predicate access.
        """
        m, d = self.raw
        import mujoco
        mujoco.mj_forward(m, d)
        out = {}
        for i in range(int(m.nbody)):
            nm = m.body(i).name
            if not (nm.startswith('gripper0') and 'hand' in nm):
                continue
            geoms = []
            for gi in range(int(m.ngeom)):
                if int(m.geom_bodyid[gi]) != i:
                    continue
                gtype = int(m.geom_type[gi])
                tname = {int(mujoco.mjtGeom.mjGEOM_BOX): 'box',
                         int(mujoco.mjtGeom.mjGEOM_SPHERE): 'sphere',
                         int(mujoco.mjtGeom.mjGEOM_CAPSULE): 'capsule',
                         int(mujoco.mjtGeom.mjGEOM_CYLINDER): 'cylinder',
                         int(mujoco.mjtGeom.mjGEOM_MESH): 'mesh'}.get(
                             gtype, str(gtype))
                geoms.append({'name': m.geom(gi).name, 'type': tname,
                              'local_pos': np.round(m.geom_pos[gi], 4).tolist(),
                              'local_size': np.round(m.geom_size[gi], 4).tolist(),
                              'world_pos': np.round(d.geom_xpos[gi], 4).tolist(),
                              # added 2026-11-09 (actor_1, 02_push_button):
                              # world orientation of the geom.  The palm-pad
                              # mesh is a thin plate (local_size[0]=0.0217 vs
                              # 0.044/0.0444), so column 0 is its face normal;
                              # without it the press direction is a guess.
                              'xmat': np.round(np.asarray(
                                  d.geom_xmat[gi], float).reshape(3, 3),
                                  4).tolist()})
            if geoms:
                out[nm] = {'body_pos': np.round(d.xpos[i], 4).tolist(),
                           'geoms': geoms}
        return out

    def bodies(self, match='', parents_depth=6):
        """Read-only body/geom/joint inventory (added 2026-11-09, actor_1,
        02_push_button).  Needed because 02_push_button's task object is a
        child body of the ``control_panel`` SceneObject whose root body has
        ZERO geoms (scene_objects(detail=True) returned ngeom=0), so the button
        position/color is invisible through the existing helpers.  Lists the
        world pose, ancestor chain, joints and every direct geom (with rgba,
        size and world AABB) for bodies whose name contains ``match``.  Reads
        model geometry only -- same information class as geom()/geom_detail();
        no reward/predicate access.
        """
        m, d = self.raw
        import mujoco
        mujoco.mj_forward(m, d)
        out = []
        for i in range(int(m.nbody)):
            nm = m.body(i).name
            if match and match not in nm:
                continue
            chain = []
            p = i
            for _ in range(int(parents_depth)):
                p = int(m.body_parentid[p])
                if p <= 0:
                    break
                chain.append(m.body(p).name)
            geoms = []
            for gi in range(int(m.ngeom)):
                if int(m.geom_bodyid[gi]) != i:
                    continue
                gtype = int(m.geom_type[gi])
                tname = {int(mujoco.mjtGeom.mjGEOM_BOX): 'box',
                         int(mujoco.mjtGeom.mjGEOM_SPHERE): 'sphere',
                         int(mujoco.mjtGeom.mjGEOM_CAPSULE): 'capsule',
                         int(mujoco.mjtGeom.mjGEOM_CYLINDER): 'cylinder',
                         int(mujoco.mjtGeom.mjGEOM_MESH): 'mesh',
                         int(mujoco.mjtGeom.mjGEOM_PLANE): 'plane'}.get(
                             gtype, str(gtype))
                cen = np.asarray(d.geom_xpos[gi], float)
                Rg = np.asarray(d.geom_xmat[gi], float).reshape(3, 3)
                sz = np.asarray(m.geom_size[gi], float)
                if gtype == int(mujoco.mjtGeom.mjGEOM_MESH):
                    aid = int(m.geom_dataid[gi])
                    v0 = int(m.mesh_vertadr[aid])
                    v1 = v0 + int(m.mesh_vertnum[aid])
                    pts = np.asarray(m.mesh_vert[v0:v1], float) @ Rg.T + cen
                elif gtype in (int(mujoco.mjtGeom.mjGEOM_BOX),
                               int(mujoco.mjtGeom.mjGEOM_SPHERE),
                               int(mujoco.mjtGeom.mjGEOM_CAPSULE),
                               int(mujoco.mjtGeom.mjGEOM_CYLINDER)):
                    ext = np.abs(Rg) @ sz
                    pts = cen + np.array([[sx * ext[0], sy * ext[1], s2 * ext[2]]
                                          for sx in (-1, 1) for sy in (-1, 1)
                                          for s2 in (-1, 1)])
                else:
                    pts = cen.reshape(1, 3)
                blo, bhi = pts.min(axis=0), pts.max(axis=0)
                rec = {'name': m.geom(gi).name, 'type': tname,
                       'rgba': np.round(np.asarray(m.geom_rgba[gi], float),
                                        3).tolist(),
                       'local_pos': np.round(m.geom_pos[gi], 4).tolist(),
                       'local_size': np.round(sz, 4).tolist(),
                       'world_pos': np.round(cen, 4).tolist(),
                       'aabb_lo': np.round(blo, 4).tolist(),
                       'aabb_hi': np.round(bhi, 4).tolist()}
                try:
                    if gtype == int(mujoco.mjtGeom.mjGEOM_MESH):
                        aid = int(m.geom_dataid[gi])
                        rec['mesh'] = m.mesh(aid).name
                except Exception:
                    pass
                geoms.append(rec)
            jrec = []
            for jid in range(int(m.njnt)):
                if int(m.jnt_bodyid[jid]) != i:
                    continue
                jt = int(m.jnt_type[jid])
                prec = {'name': m.joint(jid).name,
                        'type': {0: 'free', 1: 'ball', 2: 'slide',
                                 3: 'hinge'}.get(jt, str(jt)),
                        'range': np.round(np.asarray(m.jnt_range[jid], float),
                                          4).tolist(),
                        'axis': np.round(np.asarray(m.jnt_axis[jid], float),
                                         4).tolist(),
                        # why the button can travel past its jnt_range:
                        # MuJoCo only enforces ``range`` when limited=1.
                        'limited': int(m.jnt_limited[jid]),
                        'stiffness': round(float(m.jnt_stiffness[jid]), 4),
                        'damping': round(float(m.dof_damping[
                            int(m.jnt_dofadr[jid])]), 4),
                        'qposadr': int(m.jnt_qposadr[jid])}
                if jt != 0:
                    prec['q'] = round(float(d.qpos[int(m.jnt_qposadr[jid])]), 4)
                jrec.append(prec)
            out.append({'body': nm, 'pos': np.round(np.asarray(d.xpos[i], float),
                                                    4).tolist(),
                        'xmat': np.round(np.asarray(d.xmat[i], float).reshape(
                            3, 3), 4).tolist(),
                        'parents': chain,
                        'mass': round(float(m.body_mass[i]), 5),
                        'joints': jrec, 'geoms': geoms})
        return out

    def contact_pairs(self, limit=40):
        """Read-only list of the current sim contacts as body/geom name pairs
        plus contact position (added 2026-11-09, actor_1, 03_box_lift).
        ``observe()['ncon']`` alone cannot tell whether the robot is touching
        the floor, the static support or the box; this is the same information
        class as the documented contact counts.  No reward/predicate access.
        """
        m, d = self.raw
        out = []
        for ci in range(int(d.ncon)):
            if len(out) >= limit:
                break
            c = d.contact[ci]
            b1 = int(m.geom_bodyid[c.geom1])
            b2 = int(m.geom_bodyid[c.geom2])
            out.append({'b1': m.body(b1).name, 'b2': m.body(b2).name,
                        'g1': m.geom(int(c.geom1)).name,
                        'g2': m.geom(int(c.geom2)).name,
                        'pos': np.round(np.asarray(c.pos, float), 4).tolist()})
        return out

    def contact_detail(self, limit=80, match=''):
        """Read-only per-contact detail incl. penetration and contact force
        (added 2026-11-09, actor_1, 07_box_table_to_shelf).

        ``contact_pairs`` gives body/geom names and position only, so a
        failed grasp could not be told apart from a light touch: a pad that
        merely grazes a box face and a pad that presses it look the same, and
        the CLOSING fingers' real grip force is invisible.  This reports, for
        every sim contact whose body names contain ``match`` ('' = all, but
        floor/table contacts are always dropped unless they involve the
        robot), the contact distance (<0 = penetration), the contact-frame
        normal and the 6-D contact force from ``mj_contactForce`` (frame
        order: normal, two frictions, three torques).  Same information class
        as the documented contact counts; reads model/data only, no reward or
        predicate access.
        """
        m, d = self.raw
        import mujoco
        out = []
        for ci in range(int(d.ncon)):
            if len(out) >= limit:
                break
            c = d.contact[ci]
            b1 = int(m.geom_bodyid[c.geom1])
            b2 = int(m.geom_bodyid[c.geom2])
            n1, n2 = m.body(b1).name, m.body(b2).name
            if match and (match not in n1 and match not in n2):
                continue
            fr = np.zeros(6)
            try:
                mujoco.mj_contactForce(m, d, ci, fr)
                force = np.round(fr, 3).tolist()
            except Exception:
                force = None
            out.append({'b1': n1, 'b2': n2,
                        'g1': m.geom(int(c.geom1)).name,
                        'g2': m.geom(int(c.geom2)).name,
                        'pos': np.round(np.asarray(c.pos, float), 4).tolist(),
                        'dist': round(float(c.dist), 5),
                        'normal': np.round(np.asarray(
                            d.contact[ci].frame, float)[:3], 3).tolist(),
                        'force': force})
        return out

    def snapshot(self, path='/tmp/hmg.png', lookat=(1.7, 0.0, 0.9),
                 dist=1.2, azim=-90.0, elev=15.0, width=640, height=480):
        """Read-only offscreen render of the CURRENT sim state (added
        2026-11-09, actor_1, 05_drill_lift).  The tool server's env is created
        with render=False, so no offscreen context exists; a fresh
        mujoco.Renderer is built here on demand with EGL.  Same information
        class as the other geometry diagnostics (it reads model/data only);
        no reward/predicate access.  Used to see the task object's actual
        shape and the hand/object clearance before planning a grasp.
        """
        import mujoco
        m, d = self.raw
        mujoco.mj_forward(m, d)
        r = mujoco.Renderer(m, int(height), int(width))
        cam = mujoco.MjvCamera()
        cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
        cam.lookat[:] = [float(x) for x in lookat]
        cam.distance = float(dist)
        cam.azimuth = float(azim)
        cam.elevation = float(elev)
        r.update_scene(d, camera=cam)
        img = np.asarray(r.render())
        r.close()
        # Bug fix (2026-11-09, actor_1, 01_box_lift_floor): mujoco.Renderer
        # already returns a top-down image (row 0 = top of the view); the
        # historical img[::-1] produced an upside-down picture.  Verified
        # against body world positions on 01_box_lift_floor: the robot stands
        # upright (ankles z=0.044, shoulders z=1.085) while the old snapshot
        # showed it head-down.  The sim.render() path in hmg_render.py does
        # need the flip; this direct-Renderer path does not.
        out = np.array(img[:, :, :3], copy=True)
        try:
            import cv2
            cv2.imwrite(path, out)
            return {'path': path, 'shape': list(out.shape)}
        except Exception as exc:
            return {'error': '%s: %s' % (type(exc).__name__, exc)}

    def mesh_world(self, match='bottle', max_pts=4000):
        """Read-only world-space vertices of the task object's mesh geoms
        (added 2026-11-09, actor_1, 05_drill_lift).  geom_detail gives only an
        AABB; a 19x8x20 cm object could be a bottle, a handle, or a lying
        drill and the grasp depends on which.  Reports every mesh geom's
        vertex cloud in world coords (subsampled) and the per-axis extent, so
        the actual silhouette can be inspected without a renderer.
        """
        m, d = self.raw
        import mujoco
        mujoco.mj_forward(m, d)
        objs, sel = task_object(self.env)
        out = []
        if sel is None:
            return {'error': 'no object'}
        bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, sel.root_body))
        # mesh geoms can live on child bodies too
        for gi in range(int(m.ngeom)):
            if int(m.geom_type[gi]) != int(mujoco.mjtGeom.mjGEOM_MESH):
                continue
            if not match or match not in m.geom(gi).name:
                continue
            aid = int(m.geom_dataid[gi])
            v0 = int(m.mesh_vertadr[aid])
            v1 = v0 + int(m.mesh_vertnum[aid])
            verts = np.asarray(m.mesh_vert[v0:v1], float)
            cen = np.asarray(d.geom_xpos[gi], float)
            Rg = np.asarray(d.geom_xmat[gi], float).reshape(3, 3)
            pts = verts @ Rg.T + cen
            sel_i = np.arange(0, len(pts), max(1, len(pts) // max_pts))
            out.append({'geom': m.geom(gi).name, 'nvert': len(pts),
                        'lo': np.round(pts.min(axis=0), 4).tolist(),
                        'hi': np.round(pts.max(axis=0), 4).tolist(),
                        'pts': np.round(pts[sel_i], 4).tolist()})
        return {'body': sel.root_body, 'meshes': out}

    # ---- primitives ----------------------------------------------------
    def move_base(self, x=0.0, y=0.0, yaw=0.0, dur=1.0):
        """Walk for `dur` seconds, then STOP.  The nav command does not
        persist across tool calls: a persistent command kept the base walking
        between turns (measured y -1.67 m runaway) and pushed the box away."""
        p0 = self.env.sim.data.qpos[:2].copy()
        self.nav = np.array([x, y, yaw], float)
        self._apply(self._upper_vec(), dur)
        self.nav = np.zeros(3)
        p1 = self.env.sim.data.qpos[:2].copy()
        return {'t': round(self.t, 2),
                'moved_xy': np.round(p1 - p0, 3).tolist(),
                'state': self.state()}

    def _ik_arm(self, m, ds, mujoco, sid, idx, dof, lo, hi, tgt, R_des,
                q0, iters):
        """DLS IK for one 7-DoF arm on scratch data.  Returns
        (cost, q, pos_err, rot_err).

        Default fast path ``_ik_arm_fast`` (kinematics-only scratch update +
        buffer-reusing numpy DLS, bit-identical to ``_ik_arm_numpy``).
        ``HMG_IK_LEGACY=1`` forces the original implementation verbatim;
        ``HMG_IK_NUMBA=1`` uses the JIT kernel instead (see module comment)."""
        if IK_LEGACY:
            return self._ik_arm_numpy(m, ds, mujoco, sid, idx, dof, lo, hi,
                                      tgt, R_des, q0, iters)
        if _dls_step is not None:
            return self._ik_arm_numba(m, ds, mujoco, sid, idx, dof, lo, hi,
                                      tgt, R_des, q0, iters)
        return self._ik_arm_fast(m, ds, mujoco, sid, idx, dof, lo, hi,
                                 tgt, R_des, q0, iters)

    def _ik_arm_fast(self, m, ds, mujoco, sid, idx, dof, lo, hi, tgt,
                     R_des, q0, iters):
        """Kinematics-only scratch forward + buffer-reusing numpy DLS.

        Every arithmetic operation is performed in the same order as
        ``_ik_arm_numpy`` (mj_kinematics + mj_comPos is exactly the site /
        jacobian part of mj_forward), so the returned tuple is bit-identical
        to the original implementation.  Only allocations and the unread
        collision/constraint stages of mj_forward are removed.
        """
        q = np.clip(np.asarray(q0, float).copy(), lo, hi)
        best = (1e9, q.copy(), 1e9, 1e9)
        jp = np.zeros((3, m.nv))
        jr = np.zeros((3, m.nv))
        # MjData array properties return the same (shared-memory) view on
        # every access, so they are hoisted out of the loop.
        qpos = ds.qpos
        spos = ds.site_xpos[sid]
        sxmat = ds.site_xmat[sid]
        # scratch buffers (same shapes AND memory order as the temporaries
        # they replace: jp[:, dof] is F-ordered, so vstack of it is F-ordered
        # too, and J.T @ y takes the matching BLAS path only when J6 is F)
        J3 = np.empty((3, 7), order='F')
        J6 = np.empty((6, 7), order='F')
        A3 = np.empty((3, 3))
        A6 = np.empty((6, 6))
        EYE3 = 1e-2 * np.eye(3)
        EYE6 = 1e-2 * np.eye(6)
        e6 = np.empty(6)
        ep = np.empty(3)
        er = np.zeros(3)
        Rc9 = sxmat.reshape(3, 3)
        Re = np.empty((3, 3))
        for _ in range(iters):
            qpos[idx] = q
            # mj_forward's first two stages: the site pose and the jacobians
            # the IK reads are bit-identical to the mj_forward result, while
            # collision detection / constraint assembly (not read here) is
            # skipped.
            mujoco.mj_kinematics(m, ds)
            mujoco.mj_comPos(m, ds)
            mujoco.mj_jacSite(m, ds, jp, jr, sid)
            np.subtract(tgt, spos, out=ep)          # ep = tgt - site_xpos
            if R_des is None:
                J3[:, :] = jp[:, dof]
                np.matmul(J3, J3.T, out=A3)
                np.add(A3, EYE3, out=A3)
                dq = J3.T @ np.linalg.solve(A3, ep)
            else:
                np.matmul(R_des, Rc9.T, out=Re)     # Re = R_des @ Rc.T
                ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
                if ang > 1e-8:
                    er[:] = ang / (2 * np.sin(ang)) * np.array(
                        [Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0],
                         Re[1, 0] - Re[0, 1]])
                else:
                    er[:] = 0.0
                J6[:3] = jp[:, dof]
                J6[3:] = jr[:, dof]
                e6[:3] = ep
                e6[3:] = er
                np.matmul(J6, J6.T, out=A6)
                np.add(A6, EYE6, out=A6)
                dq = J6.T @ np.linalg.solve(A6, e6)
            mx = np.abs(dq).max()
            if mx > 0.08:
                dq *= 0.08 / mx
            np.add(q, dq, out=q)
            np.clip(q, lo, hi, out=q)
            epn = float(np.linalg.norm(ep))
            ern = float(np.linalg.norm(er))
            cost = epn + 0.2 * ern
            if cost < best[0]:
                best = (cost, q.copy(), epn, ern)
            if epn < 5e-4 and ern < 1e-2:
                break
        return best

    def _ik_arm_numba(self, m, ds, mujoco, sid, idx, dof, lo, hi, tgt,
                      R_des, q0, iters):
        """Opt-in (HMG_IK_NUMBA=1) JIT variant of ``_ik_arm_fast``: the DLS
        block runs in one numba kernel.  Numerically equivalent to, not
        bit-identical with, the numpy path (see the module comment)."""
        q = np.clip(np.asarray(q0, float).copy(), lo, hi)
        tgt = np.ascontiguousarray(tgt, dtype=np.float64)
        use_rot = R_des is not None
        Rd = (np.zeros((3, 3)) if R_des is None
              else np.ascontiguousarray(R_des, dtype=np.float64))
        dof = np.ascontiguousarray(dof, dtype=np.int64)
        lo = np.ascontiguousarray(lo, dtype=np.float64)
        hi = np.ascontiguousarray(hi, dtype=np.float64)
        jp = np.zeros((3, m.nv))
        jr = np.zeros((3, m.nv))
        qpos = ds.qpos
        spos = ds.site_xpos[sid]
        sxmat = ds.site_xmat[sid]
        best = (1e9, q.copy(), 1e9, 1e9)
        for _ in range(iters):
            qpos[idx] = q
            mujoco.mj_kinematics(m, ds)
            mujoco.mj_comPos(m, ds)
            mujoco.mj_jacSite(m, ds, jp, jr, sid)
            epn, ern, cost = _dls_step(jp, jr, dof, spos, sxmat, tgt, Rd,
                                       use_rot, q, lo, hi)
            if cost < best[0]:
                best = (cost, q.copy(), epn, ern)
            if epn < 5e-4 and ern < 1e-2:
                break
        return best

    def _ik_arm_numpy(self, m, ds, mujoco, sid, idx, dof, lo, hi, tgt,
                      R_des, q0, iters):
        """Original DLS IK implementation (numpy + mj_forward), kept verbatim
        as the bit-for-bit reference (HMG_IK_LEGACY=1)."""
        q = np.clip(np.asarray(q0, float).copy(), lo, hi)
        epn, ern = 1e9, 1e9
        best = (1e9, q.copy(), 1e9, 1e9)
        for _ in range(iters):
            ds.qpos[idx] = q
            mujoco.mj_forward(m, ds)
            jp = np.zeros((3, m.nv))
            jr = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, ds, jp, jr, sid)
            ep = tgt - ds.site_xpos[sid]
            J = jp[:, dof]
            e = ep
            er = np.zeros(3)
            if R_des is not None:
                Rc = np.asarray(ds.site_xmat[sid]).reshape(3, 3)
                Re = R_des @ Rc.T
                ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
                if ang > 1e-8:
                    er = ang / (2 * np.sin(ang)) * np.array(
                        [Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0],
                         Re[1, 0] - Re[0, 1]])
                J = np.vstack([jp[:, dof], jr[:, dof]])
                e = np.r_[ep, er]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-2 * np.eye(J.shape[0]), e)
            mx = np.abs(dq).max()
            if mx > 0.08:
                dq *= 0.08 / mx
            q = np.clip(q + dq, lo, hi)
            epn = float(np.linalg.norm(ep))
            ern = float(np.linalg.norm(er))
            cost = epn + 0.2 * ern
            if cost < best[0]:
                best = (cost, q.copy(), epn, ern)
            if epn < 5e-4 and ern < 1e-2:
                break
        return best

    def reach_box(self, gap=0.155, z_off=0.45, dur=1.6, iters=400,
                  x_off=0.0, sides=('left', 'right'), orientation='down',
                  targets=None, prefer_current=True, R_body=None,
                  hand_hold=None):
        """Both palms to the box sides via DLS IK on the live model.

        Target for side s is object_pos + [x_off, +/-gap, z_off] (the box root
        frame is world-aligned when the box stands upright).  The IK runs on a
        scratch MjData and also drives the palm orientation:
          * ``'down'``   -- +90 deg about the body y axis of the reset-pose
            site frame: fingers point down, but the palm-pad normal (site x,
            up on the left / down on the right) turns to +x, i.e. forward.
          * ``'inward'`` -- +90 deg about the body x axis: the palm normals
            become -y (left) / +y (right), so the two pads face each other
            across the box and the fingers point forward.
          * ``'inward_down'`` -- palms still face each other +/-y but the
            fingers also point down (derived rotation, same for both mirror
            hands).  The fingers hang along the box side faces and their
            curl is directed into the face: the side-pinch geometry.
          * ``'keep'`` holds the current palm orientation, ``'none'`` is
            position-only.
        The solution is written into the 31-D target in contract order; the
        return reports the solver residual and the palm residual actually
        achieved after the ramp.

        ``targets`` (added 2026-11-09, actor_2): optional explicit world palm
        targets ``{'left': [x, y, z], 'right': [...]}``.  Needed because the
        default offsets are world-axis-aligned while the box in
        01_box_lift_floor can be yawed by the robot's knees; the caller then
        cannot express a face-of-the-box target with x_off/gap/z_off alone.
        """
        st = self.state()
        ob = st.get('object') or {}
        if not ob.get('pos'):
            return {'error': 'no object visible'}
        c = np.asarray(ob['pos'], float)
        if targets:
            tgt = {k: np.asarray(v, float) for k, v in targets.items()}
        else:
            tgt = {'left': c + np.array([x_off, +gap, z_off]),
                   'right': c + np.array([x_off, -gap, z_off])}
        m, d = self.raw
        import mujoco
        ds = mujoco.MjData(m)
        ds.qpos[:] = d.qpos
        # kinematics-only forward: only site/geom/body poses are read below,
        # and those are bit-identical to mj_forward (collision skipped)
        mujoco.mj_kinematics(m, ds)
        mujoco.mj_comPos(m, ds)
        # body frame from the floating-base quaternion (w,x,y,z)
        Rb = self._quat_to_mat(np.asarray(d.qpos[3:7], float))
        R_Y90 = np.array([[0., 0, 1], [0, 1, 0], [-1, 0, 0]])  # +x -> -z (fingers down), palm normal -> +x
        # 'inward': rotate the reset site frame +90 deg about body x.  The
        # palm normal (site x: up on the left, down on the right because the
        # hands are mirror-mounted) becomes -y on the left and +y on the
        # right, i.e. both palms face the box's +/-y side faces and the
        # fingers keep pointing forward -- the natural two-sided grasp.
        R_X90 = np.array([[1., 0, 0], [0, 0, -1], [0, 1, 0]])
        # 'inward_down': palms face each other (+/-y) AND the fingers point
        # down.  Derived from the reset site frames of both (mirror-mounted)
        # hands: the same body-frame rotation yields palm normal -y (left) /
        # +y (right) with finger direction -z on both sides.  Fingers then
        # hang along the box side faces and their curl is directed into the
        # face, which is the geometry needed for a side pinch.
        R_ID = np.array([[0., 1, 0], [0, 0, -1], [-1, 0, 0]])
        target = self._upper_vec()
        ik_err, ik_rot, ik_q, ik_seed = {}, {}, {}, {}
        MIRROR = np.array([1., -1., -1., 1., -1., 1., -1.])
        GENERIC = np.array([-0.23, 0.19, -0.22, 0.71, -0.48, 1.04, 0.18])
        for s_ in sides:
            sid = self.sid.get(s_)
            if sid is None:
                continue
            names = ['%s_%s_joint' % (s_, n) for n in ARM]
            idx = [self.addr[n] for n in names]
            dof = np.array([self.dofadr[n] for n in names], dtype=int)
            lo = np.array([m.jnt_range[m.joint(self.sim_name[n]).id][0]
                           for n in names])
            hi = np.array([m.jnt_range[m.joint(self.sim_name[n]).id][1]
                           for n in names])
            Rb_side = R_body
            if isinstance(R_body, dict):
                # per-side orientation (2026-11-09, actor_1, 03_box_lift):
                # the two G1 hands share the SAME world palm pose at reset
                # (both pad normals +y, measured), so a two-sided pad grasp
                # needs OPPOSITE body-frame rotations (left R_X180, right
                # identity).  One matrix for both cannot express that, and
                # moving the arms in two separate calls lets the WBC step
                # between them and drag a contacted object.
                Rb_side = R_body.get(s_)
            if Rb_side is not None:
                # 2026-11-09, actor_1 (03_box_lift): a caller-supplied
                # rotation in the ROBOT BODY frame applied to the reset-pose
                # site frame.  Needed because the named orientations do not
                # produce the grasp they claim: the palm PAD's flat face is
                # NOT the site x axis (measured on 03_box_lift: with
                # orientation='inward' the left pad's world normal came out
                # (-0.19, 0.13, 0.97) = UP, not -y, so the documented
                # two-sided palm-pad grasp never contacts the box side).
                # With R_body = diag(1,-1,-1) (180 deg about body x) the
                # left pad normal becomes -y (right +y) and the fingers keep
                # pointing forward: the actual palm-pad side grasp.
                R_des = Rb @ np.asarray(Rb_side, float) @ self.R_site0[s_]
            elif orientation == 'down':
                R_des = Rb @ R_Y90 @ self.R_site0[s_]
            elif orientation == 'inward':
                R_des = Rb @ R_X90 @ self.R_site0[s_]
            elif orientation == 'inward_down':
                R_des = Rb @ R_ID @ self.R_site0[s_]
            elif orientation == 'keep':
                R_des = np.asarray(ds.site_xmat[sid], float).reshape(3, 3).copy()
            else:
                R_des = None
            # multi-start: current pose, mirrored other arm, generic folded
            seeds = [np.clip(ds.qpos[idx].copy(), lo, hi)]
            other = 'right' if s_ == 'left' else 'left'
            if other in ik_q:
                oq = ik_q[other]
            else:
                oidx = [self.addr['%s_%s_joint' % (other, n)] for n in ARM]
                oq = np.asarray(ds.qpos[oidx], float)
            seeds.append(np.clip(oq * MIRROR, lo, hi))
            seeds.append(np.clip(GENERIC * MIRROR if s_ == 'right' else GENERIC,
                                 lo, hi))
            best = None
            seed_used = -1
            for si, q0 in enumerate(seeds):
                cand = self._ik_arm(m, ds, mujoco, sid, idx, dof, lo, hi,
                                    tgt[s_], R_des, q0, iters)
                if best is None or cand[0] < best[0]:
                    best = cand
                    seed_used = si
                # Bug/robustness fix (2026-11-09, actor_1, 02_push_button):
                # the multi-start loop used to keep the numerically lowest
                # cost over all three seeds.  With position-only IK
                # (orientation='none') several branches can solve the same
                # point, so a mirrored/generic seed with a marginally lower
                # residual replaced the branch continuing from the CURRENT
                # pose and the arm flipped configuration between two calls
                # only 5 cm apart.  Measured on 02_push_button: successive
                # 5 cm site steps threw the right index link from
                # [0.957,-0.103,0.843] to [0.777,-0.022,0.686] and later to
                # z=1.25, and one of the flailing poses shoved the button to
                # q=-0.0505 (its nominal range is [-0.015, 0]).  If the
                # current-pose seed already solves the target, keep it: the
                # motion is then continuous and the WBC can track it.
                if si == 0 and prefer_current and cand[2] < 2e-3 and cand[3] < 5e-2:
                    break
            q = best[1]
            ik_q[s_] = q
            ik_seed[s_] = seed_used
            ds.qpos[idx] = q
            mujoco.mj_kinematics(m, ds)
            mujoco.mj_comPos(m, ds)
            ik_err[s_] = round(float(np.linalg.norm(
                ds.site_xpos[sid] - tgt[s_])), 4)
            if R_des is not None:
                Rc = np.asarray(ds.site_xmat[sid]).reshape(3, 3)
                Re = R_des @ Rc.T
                ik_rot[s_] = round(float(np.arccos(np.clip(
                    (np.trace(Re) - 1) / 2, -1, 1))), 4)
            for i, nm in enumerate(names):
                target[ALLOWED_UPPER.index(nm)] = q[i]
        if hand_hold is not None:
            # 2026-11-09, actor_1 (03_box_lift): keep the HAND joints at the
            # caller's explicit values (e.g. a closed-grip 31-vector) while
            # only the arm joints ramp to the IK solution.  Without this the
            # hand targets are the measured (contact-limited) values, so the
            # grip force decays while the arm moves; and lifting by shoulder
            # rotation also tilts the palm, which was measured to drop a box
            # that had already risen 0.09 m (verifier seed 7).
            hh = np.asarray(hand_hold, float)
            if hh.shape != (31,):
                raise ValueError('hand_hold must be a 31-D contract vector')
            for i, nm in enumerate(ALLOWED_UPPER):
                if '_hand_' in nm:
                    target[i] = hh[i]
        # Clamp EVERY target joint to its model range (2026-11-09, actor_1,
        # 03_box_lift).  ``target`` starts from self._upper_vec() (the MEASURED
        # vector), which the WBC can leave a few mrad outside a limit; those
        # measured values were then commanded back as targets (verifier v4
        # non-gate note: wrist-roll 0.0005-0.003 rad and right elbow 0.0056
        # rad over the model range).  IK-solved joints are already clipped.
        for i_, nm_ in enumerate(ALLOWED_UPPER):
            jid_ = int(m.joint(self.sim_name[nm_]).id)
            lo_, hi_ = m.jnt_range[jid_]
            target[i_] = np.clip(target[i_], lo_, hi_)
        self._ramp(target, dur)
        # achieved residual after the WBC has actually tracked the target
        mujoco.mj_forward(m, d)
        palm = {}
        # Bug fix (2026-11-09, actor_1, 02_push_button): this loop used to
        # iterate the fixed pair ('left', 'right') and index tgt[...] for
        # both sides.  A one-hand reach (``sides=('right',)`` with
        # ``targets={'right': ...}``) therefore raised KeyError: 'left'
        # AFTER the ramp had already run (measured: call returned
        # {"ok": false, "error": "KeyError: 'left'"} while the arm had
        # moved).  Only report the sides that were actually targeted.
        for s_ in tgt:
            sid = self.sid.get(s_)
            palm[s_] = None if sid is None else np.round(
                tgt[s_] - d.site_xpos[sid], 3).tolist()
        # Palm-PAD pose (2026-11-09, actor_1, 03_box_lift): the pad is a thin
        # plate whose local x axis is its normal, and it is NOT centred on the
        # grip site (measured 8.4 cm along the hand's forward axis), so the
        # site residual alone cannot tell where the pad face ended up.  Read
        # the collision geom of the pad directly (same information class as
        # hand_geoms(); no reward/predicate access).
        pad = {}
        for s_ in tgt:
            gi = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM,
                                       'gripper0_%s_g0_col' % s_))
            if gi < 0:
                pad[s_] = None
                continue
            Rg = np.asarray(d.geom_xmat[gi], float).reshape(3, 3)
            pad[s_] = {'pos': np.round(d.geom_xpos[gi], 4).tolist(),
                       'normal': np.round(Rg[:, 0], 3).tolist(),
                       'half_thick': round(float(m.geom_size[gi][0]), 4)}
        return {'ik_err': ik_err, 'ik_rot': ik_rot, 'ik_seed': ik_seed,
                'palm_residual': palm, 'pad': pad,
                'target': {k: np.round(v, 3).tolist() for k, v in tgt.items()},
                'upper_target': [round(float(x), 4) for x in target],
                't': round(self.t, 2), 'state': self.state()}

    def ik_preview(self, xs=(-0.10, -0.04, 0.02), gaps=(0.155, 0.175, 0.20),
                   zs=(0.46, 0.52, 0.58, 0.64, 0.70), orients=('down',),
                   iters=150, sides=('left', 'right'), R_body=None,
                   targets=None, prefer_current=True):
        """Read-only reach map: solve the same multi-start DLS IK as
        ``reach_box`` on scratch data and report where the palm/finger/thumb
        link origins would land, WITHOUT sending any command.  Used to pick
        grasp targets the arm can actually reach from the current base pose
        (the live state is never modified).

        ``R_body`` (2026-11-09, actor_1, 03_box_lift): same body-frame
        rotation override as reach_box; when given it replaces ``orients``
        and each row also reports the palm-PAD collision geom's world pose
        and normal (the pad, not the site, is what contacts an object).

        Branch fields (2026-11-10, coordinator, 01_box_lift_floor): the
        decision this map could not answer before was WHICH multi-start
        branch a target will solve to, before commanding it.  ``reach_box``
        only reports ``ik_seed`` after the arm has moved; a closed-loop
        retry target (``2*pad - pad_now``) can be 3.6 cm from the pad yet
        solve on the mirrored branch (seed 2) and swing the pad 0.36 m.
        ``targets`` = explicit world palm points ``{'left': [x,y,z], ...}``
        (same semantics as ``reach_box``) so any point can be screened
        without a command; ``prefer_current`` replicates reach_box's seed-0
        early break (if the current-pose branch already solves, reach_box
        keeps it -- a numeric-best-over-seeds preview would mispredict).
        Each row reports ``seed`` (winning seed: 0 current pose, 1 mirrored
        other arm, 2 generic folded), ``dq`` (max |q - q_now| over the 7 arm
        joints, rad) and ``pad_travel`` (m, pad geom displacement from its
        LIVE position).  Pass ``iters=400`` to match reach_box's solver
        budget when predicting ITS branch."""
        st = self.state()
        ob = st.get('object') or {}
        if not ob.get('pos'):
            return {'error': 'no object visible'}
        c = np.asarray(ob['pos'], float)
        m, d = self.raw
        import mujoco
        ds = mujoco.MjData(m)
        ds.qpos[:] = d.qpos
        mujoco.mj_kinematics(m, ds)
        mujoco.mj_comPos(m, ds)
        # pad position at the LIVE pose: reference for ``pad_travel``
        # (2026-11-10, coordinator, 01_box_lift_floor branch decision)
        pad0 = {}
        for s_ in sides:
            gi0 = int(mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_GEOM, 'gripper0_%s_g0_col' % s_))
            pad0[s_] = (None if gi0 < 0
                        else np.asarray(ds.geom_xpos[gi0], float).copy())
        Rb = self._quat_to_mat(np.asarray(d.qpos[3:7], float))
        R_Y90 = np.array([[0., 0, 1], [0, 1, 0], [-1, 0, 0]])
        R_X90 = np.array([[1., 0, 0], [0, 0, -1], [0, 1, 0]])
        R_ID = np.array([[0., 1, 0], [0, 0, -1], [-1, 0, 0]])
        MIRROR = np.array([1., -1., -1., 1., -1., 1., -1.])
        GENERIC = np.array([-0.23, 0.19, -0.22, 0.71, -0.48, 1.04, 0.18])
        rows = []
        for s_ in sides:
            sid = self.sid.get(s_)
            if sid is None:
                continue
            names = ['%s_%s_joint' % (s_, n) for n in ARM]
            idx = [self.addr[n] for n in names]
            dof = np.array([self.dofadr[n] for n in names], dtype=int)
            lo = np.array([m.jnt_range[m.joint(self.sim_name[n]).id][0]
                           for n in names])
            hi = np.array([m.jnt_range[m.joint(self.sim_name[n]).id][1]
                           for n in names])
            sign = 1.0 if s_ == 'left' else -1.0
            other = 'right' if s_ == 'left' else 'left'
            oidx = [self.addr['%s_%s_joint' % (other, n)] for n in ARM]
            oq = np.asarray(ds.qpos[oidx], float)
            # live pose snapshot, taken before any row writes: seed 0 must
            # stay "solve from where the robot is NOW" (the docstring
            # contract, and what reach_box means by seed 0).  Rows below do
            # write ds.qpos, which used to silently make seed 0 = the
            # PREVIOUS row's solution.
            q_base = np.asarray(ds.qpos[idx], float).copy()
            seeds = [np.clip(q_base.copy(), lo, hi),
                     np.clip(oq * MIRROR, lo, hi),
                     np.clip(GENERIC * MIRROR if s_ == 'right' else GENERIC,
                             lo, hi)]
            for oi in orients:
                Rb_side = (R_body.get(s_) if isinstance(R_body, dict)
                           else R_body)
                if Rb_side is not None:
                    R_des = Rb @ np.asarray(Rb_side, float) @ self.R_site0[s_]
                elif oi == 'down':
                    R_des = Rb @ R_Y90 @ self.R_site0[s_]
                elif oi == 'inward':
                    R_des = Rb @ R_X90 @ self.R_site0[s_]
                elif oi == 'inward_down':
                    R_des = Rb @ R_ID @ self.R_site0[s_]
                elif oi == 'keep':
                    R_des = np.asarray(
                        ds.site_xmat[sid], float).reshape(3, 3).copy()
                else:
                    R_des = None
                if targets is not None and s_ not in targets:
                    continue
                # explicit targets: one row per (side, orient) instead of the
                # object-relative grid
                _xs = [None] if targets is not None else xs
                _gs = [None] if targets is not None else gaps
                _zs = [None] if targets is not None else zs
                for xo in _xs:
                    for gp in _gs:
                        for zo in _zs:
                            if targets is not None:
                                tgt = np.asarray(targets[s_], float)
                            else:
                                tgt = c + np.array([xo, sign * gp, zo])
                            best = None
                            si_used = -1
                            for si, q0 in enumerate(seeds):
                                cand = self._ik_arm(
                                    m, ds, mujoco, sid, idx, dof, lo, hi,
                                    tgt, R_des, q0, iters)
                                if best is None or cand[0] < best[0]:
                                    best = cand
                                    si_used = si
                                # same early break as reach_box: if the
                                # current-pose branch already solves, it is
                                # the branch reach_box will command
                                if (si == 0 and prefer_current
                                        and cand[2] < 2e-3
                                        and cand[3] < 5e-2):
                                    break
                            ds.qpos[idx] = best[1]
                            mujoco.mj_kinematics(m, ds)
                            mujoco.mj_comPos(m, ds)
                            perr = float(np.linalg.norm(
                                ds.site_xpos[sid] - tgt))
                            rot = None
                            if R_des is not None:
                                Re = R_des @ np.asarray(
                                    ds.site_xmat[sid]).reshape(3, 3).T
                                rot = float(np.arccos(np.clip(
                                    (np.trace(Re) - 1) / 2, -1, 1)))
                            bodies = {}
                            for bid in range(int(m.nbody)):
                                nm = m.body(bid).name
                                if s_ in nm and 'hand' in nm and (
                                        'palm_link' in nm
                                        or '_1_link' in nm
                                        or 'thumb_2_link' in nm):
                                    key = nm.split('_hand_')[-1]
                                    bodies[key] = np.round(
                                        ds.xpos[bid] - c, 3).tolist()
                            pad = None
                            gi = int(mujoco.mj_name2id(
                                m, mujoco.mjtObj.mjOBJ_GEOM,
                                'gripper0_%s_g0_col' % s_))
                            if gi >= 0:
                                Rg = np.asarray(ds.geom_xmat[gi],
                                                float).reshape(3, 3)
                                pad = {'pos': np.round(
                                            ds.geom_xpos[gi] - c, 3).tolist(),
                                       'normal': np.round(Rg[:, 0], 3).tolist()}
                            ptrav = None
                            if pad is not None and pad0.get(s_) is not None:
                                ptrav = round(float(np.linalg.norm(
                                    ds.geom_xpos[gi] - pad0[s_])), 4)
                            rows.append({
                                'side': s_, 'o': oi, 'xo': xo, 'gap': gp,
                                'z': zo, 'err': round(perr, 3),
                                'rot': None if rot is None else round(rot, 3),
                                'seed': si_used,
                                'dq': round(float(np.abs(
                                    best[1] - q_base).max()), 4),
                                'pad_travel': ptrav,
                                'pad': pad,
                                'palm': bodies.get('palm_link'),
                                'idx1': bodies.get('index_1_link'),
                                'mid1': bodies.get('middle_1_link'),
                                'th2': bodies.get('thumb_2_link')})
        return {'box': np.round(c, 3).tolist(),
                'base': self.base_state(), 'rows': rows}

    def close_grip(self, angle=0.6, dur=1.0):
        q = self._upper_vec()
        m_ = self.raw[0]
        import mujoco as _mj
        for i, nm in enumerate(ALLOWED_UPPER):
            if '_hand_' not in nm:
                continue
            jid = _mj.mj_name2id(m_, _mj.mjtObj.mjOBJ_JOINT, self.sim_name[nm])
            lo, hi = m_.jnt_range[jid]
            # close towards the larger-range side (reference grasp_qpos:
            # left [-1.2 fingers, +1/+1.25 thumb], mirrored on the right);
            # a symmetric joint (thumb_0) is left where it is.
            if abs(hi) > abs(lo) + 1e-9:
                sgn = 1.0
            elif abs(lo) > abs(hi) + 1e-9:
                sgn = -1.0
            else:
                sgn = 0.0
            # Bug fix (2026-11-09, actor_1, 01_box_lift_floor; verifier v1 gate
            # B): this used to write sgn*angle with no per-joint clamp, so
            # any angle > |range| emitted an OUT-OF-RANGE command.  Measured:
            # close_grip(1.25) commanded left thumb_1 +1.25 against its
            # [ -0.7243, +1.0472 ] range (excess 0.2028 rad, 33 control sends
            # at t=13.40-13.78 on seed 3) and the mirrored -1.25 on the right.
            # The realized position stayed inside only because the PD saturates
            # at the limit; the commanded target itself violated the model
            # limit.  Clamp here so every close angle is legal on every path.
            q[i] = float(np.clip(sgn * angle, lo, hi))
        self._ramp(q, dur)
        return {'t': round(self.t, 2), 'contacts': self.state().get(
            'hand_object_contacts'), 'state': self.state(),
            'upper_target': [round(float(x), 4) for x in q]}

    def grasp_scratch(self, targets, sides=('left', 'right'), orientation='down',
                      R_body=None, grip_angle=0.6, iters=300, open_angle=None):
        """READ-ONLY grasp check on scratch data (added 2026-11-09, actor_2,
        05_drill_lift).  Why: a candidate palm target can be perfectly
        reachable (reach_box ik_err ~0) yet the closed fingers never touch the
        object, and the only way to know before commanding the real robot was
        to move it -- which on 05_drill_lift pushed the drill off its sampled
        pose twice.  This solves the same multi-start DLS IK as ``reach_box``
        on a scratch MjData, writes the resulting arm joints AND the
        ``close_grip`` finger angles into it, runs ``mj_forward`` (collision
        detection) and reports the actual contacts between hand geoms and the
        task object plus the achieved palm/pad/finger geometry.  It never
        modifies the live sim state (no command is sent), so it is the same
        information class as ik_preview()/hand_geoms() (model geometry and
        contact pairs), not a reward/predicate read.
        """
        m, d = self.raw
        import mujoco
        ds = mujoco.MjData(m)
        ds.qpos[:] = d.qpos
        mujoco.mj_kinematics(m, ds)
        mujoco.mj_comPos(m, ds)
        Rb = self._quat_to_mat(np.asarray(d.qpos[3:7], float))
        R_Y90 = np.array([[0., 0, 1], [0, 1, 0], [-1, 0, 0]])
        R_X90 = np.array([[1., 0, 0], [0, 0, -1], [0, 1, 0]])
        R_ID = np.array([[0., 1, 0], [0, 0, -1], [-1, 0, 0]])
        MIRROR = np.array([1., -1., -1., 1., -1., 1., -1.])
        GENERIC = np.array([-0.23, 0.19, -0.22, 0.71, -0.48, 1.04, 0.18])
        out = {'sides': {}, 'contacts': [], 'ncon_all': 0}
        for s_ in sides:
            sid = self.sid.get(s_)
            names = ['%s_%s_joint' % (s_, n) for n in ARM]
            idx = [self.addr[n] for n in names]
            dof = np.array([self.dofadr[n] for n in names], dtype=int)
            lo = np.array([m.jnt_range[m.joint(self.sim_name[n]).id][0]
                           for n in names])
            hi = np.array([m.jnt_range[m.joint(self.sim_name[n]).id][1]
                           for n in names])
            Rb_side = (R_body.get(s_) if isinstance(R_body, dict) else R_body)
            if Rb_side is not None:
                R_des = Rb @ np.asarray(Rb_side, float) @ self.R_site0[s_]
            elif orientation == 'down':
                R_des = Rb @ R_Y90 @ self.R_site0[s_]
            elif orientation == 'inward':
                R_des = Rb @ R_X90 @ self.R_site0[s_]
            elif orientation == 'inward_down':
                R_des = Rb @ R_ID @ self.R_site0[s_]
            elif orientation == 'keep':
                R_des = np.asarray(ds.site_xmat[sid], float).reshape(3, 3).copy()
            else:
                R_des = None
            other = 'right' if s_ == 'left' else 'left'
            oidx = [self.addr['%s_%s_joint' % (other, n)] for n in ARM]
            oq = np.asarray(ds.qpos[oidx], float)
            seeds = [np.clip(ds.qpos[idx].copy(), lo, hi),
                     np.clip(oq * MIRROR, lo, hi),
                     np.clip(GENERIC * MIRROR if s_ == 'right' else GENERIC,
                             lo, hi)]
            best = None
            seed_used = -1
            for si, q0 in enumerate(seeds):
                cand = self._ik_arm(m, ds, mujoco, sid, idx, dof, lo, hi,
                                    np.asarray(targets[s_], float), R_des,
                                    q0, iters)
                if best is None or cand[0] < best[0]:
                    best = cand
                    seed_used = si
            ds.qpos[idx] = best[1]
            out['sides'][s_] = {'ik_cost': round(float(best[0]), 4),
                                'ik_err': round(float(best[2]), 4),
                                'ik_rot': round(float(best[3]), 4),
                                'seed': seed_used}
        # finger joints: same sign rule as close_grip, held at grip_angle
        fing = {}
        for i, nm in enumerate(ALLOWED_UPPER):
            if '_hand_' not in nm:
                continue
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,
                                    self.sim_name[nm])
            lo, hi = m.jnt_range[jid]
            if abs(hi) > abs(lo) + 1e-9:
                sgn = 1.0
            elif abs(lo) > abs(hi) + 1e-9:
                sgn = -1.0
            else:
                sgn = 0.0
            # same per-joint clamp as close_grip (2026-11-09, actor_1,
            # 01_box_lift_floor): the scratch screen must not place the
            # simulated fingers outside the model range either, or its contact
            # report can disagree with what the controller is allowed to do.
            val = float(np.clip(sgn * (grip_angle if open_angle is None
                                       else open_angle), lo, hi))
            ds.qpos[self.addr[nm]] = val
            fing[nm] = round(float(val), 3)
        mujoco.mj_kinematics(m, ds)
        mujoco.mj_comPos(m, ds)
        mujoco.mj_forward(m, ds)
        out['ncon_all'] = int(ds.ncon)
        out['all_object_contacts'] = []
        # 2026-11-09, actor_1 (07_box_table_to_shelf): this filter used the
        # literal 'bottle_' from 05_drill_lift, so on every other task the
        # scratch screen reported ZERO object contacts no matter what the
        # grasp did (measured here: a candidate whose palm pad overlapped the
        # box wall still returned contacts=[]).  Resolve the task object's
        # geom set from the model (root body and its descendant bodies), the
        # same object task_object() selects, instead of a task-specific name.
        objs_t, sel_t = task_object(self.env)
        obj_geoms = set()
        if sel_t is not None:
            root = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                         sel_t.root_body))
            if root >= 0:
                for gi in range(int(m.ngeom)):
                    b = int(m.geom_bodyid[gi])
                    for _ in range(8):
                        if b == root:
                            obj_geoms.add(gi)
                            break
                        if b <= 0:
                            break
                        b = int(m.body_parentid[b])
        for ci in range(int(ds.ncon)):
            c = ds.contact[ci]
            n1 = m.geom(int(c.geom1)).name
            n2 = m.geom(int(c.geom2)).name
            hh = ('_hand_' in n1) or ('_hand_' in n2) or n1.startswith('gripper0') \
                or n2.startswith('gripper0')
            o1 = int(c.geom1) in obj_geoms
            o2 = int(c.geom2) in obj_geoms
            oo = o1 or o2
            if oo:
                # added 2026-11-09 (actor_2, 05_drill_lift): the measured
                # failure was the RIGHT WRIST (robot0_*_wrist_*_link) pushing
                # the barrel's back end during the approach, which the hand-
                # only screen could not see.  Report every object contact so
                # a candidate can be rejected when an arm link, not a finger,
                # is what touches the drill.
                out['all_object_contacts'].append({
                    'g1': n1, 'g2': n2,
                    'pos': np.round(np.asarray(c.pos, float), 4).tolist(),
                    'dist': round(float(c.dist), 4),
                    'other_body': m.body(int(
                        m.geom_bodyid[c.geom2 if o1 else c.geom1])
                    ).name})
            if hh and oo:
                out['contacts'].append({
                    'g1': n1, 'g2': n2,
                    'pos': np.round(np.asarray(c.pos, float), 4).tolist(),
                    'dist': round(float(c.dist), 4)})
        for s_ in sides:
            sid = self.sid.get(s_)
            rec = out['sides'][s_]
            rec['palm_site'] = np.round(ds.site_xpos[sid], 4).tolist()
            # arm-link world poses, so a plan can require wrist/elbow
            # clearance from the object (see all_object_contacts note).
            arm = {}
            for bid in range(int(m.nbody)):
                nm = m.body(bid).name
                if nm.startswith('robot0_') and ('wrist' in nm or 'elbow' in nm):
                    arm[nm] = np.round(ds.xpos[bid], 4).tolist()
            rec['arm_links'] = arm
            bodies = {}
            for bid in range(int(m.nbody)):
                nm = m.body(bid).name
                if nm.startswith('gripper0_%s_' % s_) and 'hand' in nm:
                    bodies[nm.split('_hand_')[-1]] = np.round(
                        ds.xpos[bid], 4).tolist()
            rec['links'] = bodies
            gi = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM,
                                       'gripper0_%s_g0_col' % s_))
            if gi >= 0:
                Rg = np.asarray(ds.geom_xmat[gi], float).reshape(3, 3)
                rec['pad'] = {'pos': np.round(ds.geom_xpos[gi], 4).tolist(),
                              'normal': np.round(Rg[:, 0], 3).tolist()}
        out['fingers'] = fing
        return out

    def set_height(self, h, dur=1.2):
        self.height = float(np.clip(h, 0.44, 0.90))   # see _send comment
        self._apply(self._upper_vec(), dur)
        return {'height': round(self.height, 3), 't': round(self.t, 2),
                'state': self.state()}

    def lift(self, dz=0.08, dur=1.2):
        # WARNING (2026-11-09, verified on 01_box_lift_floor): ``_apply``
        # re-reads the MEASURED joint vector every control step.  For the
        # finger joints that means each step's target becomes the current
        # (contact-limited, hence relaxed) finger angle, so the grip force
        # decays to zero and a grasped object is released during the lift.
        # Measured: a 19 cm lift only worked when the explicit closed-finger
        # targets were held fixed for the whole height ramp (a primitive
        # doing step_goal(upper=<fixed 31-vector>) instead of upper=None).
        self.height = float(min(0.95, self.height + dz))
        self._apply(self._upper_vec(), dur)
        return {'height': round(self.height, 3), 't': round(self.t, 2),
                'state': self.state()}
