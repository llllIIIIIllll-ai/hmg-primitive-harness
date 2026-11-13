"""arm_ramp: move a named subset of upper-body joints onto an explicit target
vector by a SEGMENTED ramp, and report the achieved tracking error.

WHY THIS EXISTS (measured)
    On 05_drill_lift the second pinch reach lands the left arm in one of two
    IK branches of the SAME palm-pad site target.  The three passing seeds
    settle the 7 left-arm joints at
        s0 [-2.1349,-0.0500, 2.6191, 0.9569, 0.7257, 1.5494, 1.5067]
        s2 [-2.0997,-0.0502, 2.6190, 1.0545, 0.7044, 1.5382, 1.5127]
        s3 [-2.0631,-0.0530, 2.6191, 1.1111, 0.6937, 1.5439, 1.5042]
    (j5 identical at +2.619 on all three), while the failing seed 1 settles at
        s1 [-2.0469, 0.1448, 0.4555, 2.0815,-1.3545, 1.3028, 1.5748]
    -- 2.16 rad away in j5 and ~1.4 rad in j6/j7.  Every centimetre-scale pad
    correction (pad_servo, pad_place, target offsets, a base walk) failed,
    because it was asked to remove a 2 rad JOINT-SPACE branch difference with
    millimetre moves.
    The obstacle to fixing it in joint space was tracking: a single GENERIC
    0.9 s ramp to a far vector landed far from its target (v15), which made
    "the WBC does not track arbitrary joint vectors" look like a hard limit.
    It is not a limit on the SIZE of the move, it is a limit on the RAMP
    STRUCTURE: the same 2.15 rad j5 move delivered as 10 smoothstep segments of
    0.30 s each tracks to within 6 mrad.
        v28 seed 1, target j5 +2.619, reached +2.613 (seg 10 x 0.30 s)
    and the episode then passes.  So between the two poses the controller needs
    a chain of nearby sub-goals, not one large one.

MECHANISM
    1. START.  ``api.commanded()`` when available -- the last 31-D goal the
       harness actually sent in contract order -- else ``api.upper()`` (the
       measured vector).  Using the COMMANDED start matters: it is the pose the
       controller is already converging to, so segment 1 does not fight a
       pending convergence.
    2. TARGET.  Copy of the start; the joints named in ``joints`` are replaced
       from ``target``.  ``target`` may be either a vector of exactly
       ``len(joints)`` values (in the same order as ``joints``), or a full
       31-D contract-order vector (then ``joints`` only selects what is
       reported).  All other joints, the nav command and the base height are
       left untouched by construction.
    3. SEGMENTED RAMP.  ``seg`` sub-goals on the straight line start->target,
       each commanded by ``api._ramp`` for ``seg_dur`` seconds.  Segment ``i``
       ends at ``start + (target-start)*(i+1)/seg``, so the LAST segment is
       exactly the target and the final sent command equals it.  ``seg``
       defaults to ``ceil(max_joint_delta / max_step)`` with ``max_step`` 0.25
       rad -- just above the 0.216 rad/segment that was measured to track --
       so a caller who only knows WHERE it wants the arm does not also have to
       choose a segmentation.
    4. CERTIFY.  Read ``api.upper()`` after the last segment and report
       ``achieved``, per-joint ``err`` and ``max_err`` on the moved joints, and
       ``tracked`` (1 iff max_err <= tol, default 0.06 rad).  This is the
       measurement that distinguishes "the arm is where I asked" from "the
       command was sent": a caller that skips it cannot tell a tracked move
       from a v15-style lag.
    5. OPTIONAL SETTLE READ (``n_read > 0``).  Dwell on the target with
       ``api._ramp(target, dt)`` per read and record pad/object/base/arm, so
       the caller can see whether the pad has settled and where it went.
       ``pad_before`` is the pad at the first read and ``pad_after`` at the
       last; ``drift`` is the max distance of the tail half of the reads to
       the last one (the same settle statistic ``pad_hold_audit`` uses).  This
       is a CONVENIENCE read, not a certification: a pad that is still
       oscillating needs the longer, averaged window of ``pad_hold_audit``.

FAILURE SIGNATURE IT ADDRESSES
    "The reach reported small ik_err but the arm is in a different joint
    branch (or a different joint-space pose) than the one that works, and a
    corrective move is not tracked by the whole-body controller."
    Concretely: a large (>1 rad) single-joint or multi-joint target that a
    one-shot ramp leaves far short, where the same target delivered as nearby
    sub-goals is reached.  It is a POSE-TRANSPORT skill -- it moves the arm to
    a vector you supply -- and it makes no claim about where the object is.

CONTRACT
    preconditions
      - ``api._ramp`` and ``api.commanded``/``api.upper`` are available (the
        skill is called through api.skill, so it shares the program's tools);
      - ``target`` is not None and its length is either ``len(joints)`` or 31;
      - ``joints`` are contract-order upper-body indices (0..30); the caller
        is responsible for not driving a joint past its range;
      - ``seg_dur`` is long enough for the controller to converge one segment
        (0.30 s measured on 05_drill_lift; 0.10 s halves are a different,
        untested regime).
    postconditions
      - the 31-D goal last sent is ``target`` laid onto the ``start`` vector
        (exactly, on the final segment), followed by any settle dwells;
      - ``achieved``/``err``/``max_err`` report the MEASURED vector, and
        ``tracked`` says whether max_err <= tol;
      - no other joint, the nav command or the base height is modified.
    side effects
      - commands the 31-D upper-body goal; advances sim time by
        ``seg * seg_dur + n_read * dt`` s (``+ hold`` when hold > 0);
      - the arm MOVES through the whole intermediate path: the hand and any
        carried or nearby object may be swept -- this is a transport move and
        it does not plan around the object;
      - read-only api.upper()/api.state()/api.hand_geoms() reads (the last runs
        mj_forward; none changes qpos).

ARGS
    side      'left'/'right'; selects the default ``joints``  (left)
    target    vector of joint targets, length len(joints) or 31   (required)
    joints    contract-order indices to move; None -> [3..9] for left,
              [10..16] for right                                  (None)
    seg       number of sub-goals; None -> ceil(max_delta/max_step)  (None)
    seg_dur   duration of each sub-goal, s                          (0.30)
    max_step  largest per-segment joint motion used for auto ``seg``, rad (0.25)
    hold      extra dwell on the target after the last segment, s   (0.0)
    n_read    optional settle reads after the move; 0 skips          (0)
    dt        dwell/read interval for the settle reads, s            (0.20)
    tol       tracking radius on the measured joint error, rad       (0.06)
    phase     label copied into every trace record

RETURNS
    {'ok': 0/1, 'side', 'joints', 'start', 'target', 'achieved', 'err',
     'max_err', 'delta', 'max_delta', 'seg', 'seg_dur', 'dur_total',
     'tracked', 'tol', 'hold',
     'pad_before', 'pad_after', 'dpad', 'rel', 'drift', 'settled',
     'trace': [{'k','arm','pad','rel','base','yaw','obj','t'}], 't'}
    or {'ok': 0, 'error': <str>, 'trace': []} on a bad target/side.
    ``pad_*``/``rel``/``drift``/``settled`` are None when n_read == 0.

SCOPE (believed)
    In scope: transporting a 6-DOF arm (or any subset of the 31 contract
    joints) to an explicitly known joint vector across a large joint-space
    distance -- >1 rad per joint -- on the G1 whole-body controller used by
    this harness, including crossing between IK branches of the same end
    effector site.  Verified on 05_drill_lift for a 2.15 rad j5 move on 4
    seeds (10 x 0.30 s).  Per-joint track error on that move: j5 6 mrad, j9
    8 mrad, the WORST joint (j6, which is asked to move 1.04 rad) 72 mrad -- so
    with the default tol=0.06 this move reports tracked=0 even though the pad
    settles within 1 mm and the grasp succeeds.  Read ``err`` per joint and
    choose ``tol`` accordingly; ``tracked`` is a strict max-over-moved-joints
    statement, not a pass/fail of the task.  Applied harmlessly on the 3 seeds
    that did not need it.
    Out of scope: choosing the target (this skill never inspects the object,
    so it cannot decide that a branch is wrong -- the caller supplies the
    vector, e.g. from a passing seed's held pose or a nominal IK solve); fine
    pad placement (use pad_hold_audit / pad_place / pad_servo afterwards);
    obstacle avoidance; faster-than-seg_dur transport; and any claim that the
    END POSE is good -- only that it was REACHED.
"""


def _read(api, side, pad_geom):
    """(pad world pos, pad normal, palm body origin) or three Nones."""
    hg = api.hand_geoms()
    pw = None
    pn = None
    pp = None
    for b in hg:
        if ('_%s_' % side) not in b:
            continue
        if 'palm_link' in b:
            pp = np.array(hg[b]['body_pos'], dtype='float64')
        for gg in hg[b]['geoms']:
            if gg['name'] == pad_geom:
                pw = np.array(gg['world_pos'], dtype='float64')
                pn = np.array(gg['xmat'], dtype='float64')[:, 0]
    return pw, pn, pp


def _obj(api):
    st = api.state() or {}
    op = (st.get('object') or {}).get('pos')
    if op is None:
        return None
    return np.array(op, dtype='float64')


def run(api, side='left', target=None, joints=None, seg=None, seg_dur=0.30,
        max_step=0.25, hold=0.0, n_read=0, dt=0.20, tol=0.06,
        phase='arm_ramp'):
    tr = []
    if target is None:
        return {'ok': 0, 'error': 'target is required', 'trace': tr}
    if side != 'left' and side != 'right':
        return {'ok': 0, 'error': 'side must be left or right', 'trace': tr}
    if joints is None:
        jl = []
        if side == 'left':
            for j in np.arange(3, 10):
                jl.append(j.item())
        else:
            for j in np.arange(10, 17):
                jl.append(j.item())
    else:
        jl = []
        for j in joints:
            jl.append(j)
    pad_geom = 'gripper0_%s_g0_col' % side
    cmd0 = api.commanded()
    lbl = 'commanded'
    if cmd0 is None:
        cmd0 = api.upper()
        lbl = 'measured_fallback'
    start = np.array(cmd0, dtype='float64')
    tv = np.array(target, dtype='float64')
    tgt = np.array(start, dtype='float64')
    nj = jl.__len__()
    if tv.shape[0] == nj:
        for i in np.arange(nj):
            tgt[jl[i]] = tv[i]
    elif tv.shape[0] == 31:
        tgt = np.array(tv, dtype='float64')
    else:
        return {'ok': 0, 'error': 'target must have len(joints) or 31 values',
                'trace': tr}
    # ---- segmentation ------------------------------------------------------
    mx = 0.0
    for j in jl:
        d = np.abs(tgt[j] - start[j]).item()
        if d > mx:
            mx = d
    ns = seg
    if ns is None:
        ns = 1
        if mx > max_step:
            ns = math.ceil(mx / max_step)
        if ns < 1:
            ns = 1
    for i in np.arange(ns):
        a = (i.item() + 1.0) / ns
        c = start + (tgt - start) * a
        api._ramp(c, seg_dur)
    achieved = np.array(api.upper(), dtype='float64')
    err = np.zeros(31, dtype='float64')
    delt = np.zeros(31, dtype='float64')
    me = 0.0
    md = 0.0
    for j in jl:
        e = np.abs(tgt[j] - achieved[j]).item()
        err[j] = e
        delt[j] = np.abs(tgt[j] - start[j]).item()
        if e > me:
            me = e
        if delt[j] > md:
            md = delt[j]
    tracked = 0
    if me <= tol:
        tracked = 1
    out = {'ok': 1, 'side': side, 'joints': jl, 'start_label': lbl,
           'start': np.round(start[jl], 4).tolist(),
           'target': np.round(tgt[jl], 4).tolist(),
           'achieved': np.round(achieved[jl], 4).tolist(),
           'err': np.round(err[jl], 4).tolist(),
           'max_err': np.round(me, 4).item(),
           'delta': np.round(delt[jl], 4).tolist(),
           'max_delta': np.round(md, 4).item(),
           'seg': ns, 'seg_dur': seg_dur,
           'dur_total': np.round(ns * seg_dur, 3).item(),
           'tracked': tracked, 'tol': tol, 'hold': hold,
           'phase': phase}
    # ---- optional dwell on the final target --------------------------------
    if hold > 0.0:
        api._ramp(tgt, hold)
    pb = None
    pa = None
    rel = None
    obj = None
    drift = None
    settled = None
    if n_read > 0:
        reads = []
        for i in np.arange(n_read):
            api._ramp(tgt, dt)
            pw, pn, pp = _read(api, side, pad_geom)
            ob = _obj(api)
            bs = api.base_state() or {}
            bp = bs.get('pos')
            r = {'k': 'hold', 'i': i.item(),
                 'arm': np.round(api.upper()[0:31], 3).tolist(),
                 't': np.round(api.t, 2).item()}
            if pw is not None:
                r['pad'] = np.round(pw, 4).tolist()
                if ob is not None:
                    r['rel'] = np.round(pw - ob, 4).tolist()
            if ob is not None:
                r['obj'] = np.round(ob, 4).tolist()
                obj = ob
            if bp is not None:
                r['base'] = np.round(np.array(bp, dtype='float64'),
                                     4).tolist()
                r['yaw'] = np.round(bs.get('yaw_deg', 0.0), 2).item()
            reads.append(r)
            tr.append(r)
        k = 0
        for r in reads:
            if 'pad' not in r:
                continue
            q = np.array(r['pad'], dtype='float64')
            if pb is None:
                pb = q
            pa = q
            k += 1
        if pb is not None:
            out['pad_before'] = np.round(pb, 4).tolist()
            out['pad_after'] = np.round(pa, 4).tolist()
            out['dpad'] = np.round(pa - pb, 4).tolist()
            if ob is not None:
                out['rel'] = np.round(pa - ob, 4).tolist()
        if obj is not None:
            out['obj'] = np.round(obj, 4).tolist()
        if k >= 2:
            tail = reads[reads.__len__() // 2:]
            last = None
            for r in tail:
                if 'pad' in r:
                    last = np.array(r['pad'], dtype='float64')
            dr = 0.0
            if last is not None:
                for r in tail:
                    if 'pad' not in r:
                        continue
                    d = np.linalg.norm(np.array(r['pad'],
                                                dtype='float64') - last).item()
                    if d > dr:
                        dr = d.item()
            drift = np.round(dr, 4).item()
            settled = 0
            if dr < 0.005:
                settled = 1
        out['drift'] = drift
        out['settled'] = settled
    if 'obj' not in out:
        out['obj'] = None
    out['trace'] = tr
    out['t'] = np.round(api.t, 2).item()
    return out
