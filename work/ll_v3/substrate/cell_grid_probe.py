def _box_dist(p, c, R, sz):
    q = R.T @ (p - c)
    e0 = q[0] - sz[0]
    e1 = q[1] - sz[1]
    e2 = q[2] - sz[2]
    if e0 < 0.0:
        e0 = 0.0
    if e1 < 0.0:
        e1 = 0.0
    if e2 < 0.0:
        e2 = 0.0
    return (e0 * e0 + e1 * e1 + e2 * e2) ** 0.5


def _cap_dist(p, c, R, sz):
    q = R.T @ (p - c)
    t = q[2]
    if t > sz[1]:
        t = sz[1]
    if t < -sz[1]:
        t = -sz[1]
    dx = q[0]
    dy = q[1]
    dz = q[2] - t
    return (dx * dx + dy * dy + dz * dz) ** 0.5 - sz[0]


def _yaw_idx(api):
    jj = api.addr['left_shoulder_yaw_joint']
    k = 0
    while k < 31:
        if api.qpos_addr[k] == jj:
            return k
        k = k + 1
    return None


def _cell(api, G, h, side, approach, frac, roll, palm_off, q0, yaw_i):
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    b = api.base_state()
    dxv = obj[0] - b['pos'][0]
    dyv = obj[1] - b['pos'][1]
    dd = (dxv * dxv + dyv * dyv) ** 0.5
    fx = dxv / dd
    fy = dyv / dd
    px = -fy
    py = fx
    if approach == 'front':
        f = np.array([fx, fy, 0.0])
        n0 = np.array([-px, -py, 0.0]) if side == 'left' else np.array(
            [px, py, 0.0])
    else:
        f = np.array([px, py, -0.5])
        n0 = np.array([fx, fy, 0.0])
    f = f / np.linalg.norm(f)
    n0 = n0 - np.dot(n0, f) * f
    n0 = n0 / np.linalg.norm(n0)
    rr = np.radians(roll)
    n = n0 * np.cos(rr) + np.cross(f, n0) * np.sin(rr)
    n = n - np.dot(n, f) * f
    n = n / np.linalg.norm(n)
    axis = h['R'][:, 2]
    pt = h['pos'] + (frac - 0.5) * 2.0 * h['size'][2] * axis
    tgt_p = pt - f * palm_off
    tgt = {side: tgt_p.tolist()}
    g = api.geom()
    hg = api.hand_geoms()
    site = g['sites'].get(side)
    R_site = np.array(site['xmat'])
    pad_pos = None
    pad_n = None
    for body in hg:
        for gg in hg[body]['geoms']:
            if gg['name'] == 'gripper0_%s_g0_col' % side:
                pad_pos = np.array(gg['world_pos'])
                pad_n = np.array(gg['xmat'])[:, 0]
    fwd_w = pad_pos - np.array(site['pos'])
    fwd_w = fwd_w / np.linalg.norm(fwd_w)
    e1 = R_site.T @ fwd_w
    e2 = R_site.T @ pad_n
    e2 = e2 - np.dot(e2, e1) * e1
    e2 = e2 / np.linalg.norm(e2)
    e3 = np.cross(e1, e2)
    B_local = np.column_stack([e1, e2, e3])
    B_des = np.column_stack([f, n, np.cross(f, n)])
    R_des = B_des @ B_local.T
    U, S, Vt = np.linalg.svd(R_des)
    R_des = U @ Vt
    R_site0 = np.asarray(api.R_site0[side])
    Rb = api._quat_to_mat(np.asarray(api.raw[1].qpos[3:7]))
    Rbod = Rb.T @ R_des @ R_site0.T
    U, S, Vt = np.linalg.svd(Rbod)
    rb = {side: (U @ Vt).tolist()}
    ip = api.ik_preview(targets=tgt, R_body=rb, orients=('keep',),
                        iters=150, sides=(side,))
    seed = {}
    for row in (ip.get('rows') or []):
        qn = row.get('q_named') or {}
        for kk in qn:
            seed[kk] = qn[kk]
    out = {'side': side, 'approach': approach, 'frac': frac, 'roll': roll,
           'target': tgt[side]}
    best = None
    for tag in ('unseeded', 'seeded'):
        if tag == 'seeded':
            r = api.reach_box_qp(task_dim=12, targets=tgt, R_body=rb,
                                 orientation='keep', dur=0.02, apply=False,
                                 seed_q=seed, sides=(side,))
        else:
            r = api.reach_box_qp(task_dim=12, targets=tgt, R_body=rb,
                                 orientation='keep', dur=0.02, apply=False,
                                 sides=(side,))
        ex = (r.get('ik_err') or {}).get(side)
        ws = r.get('wrist_solved') or {}
        wn = (('robot0_g27_col', 'robot0_g28_col') if side == 'left'
              else ('robot0_g34_col', 'robot0_g35_col'))
        wmin = None
        wname = None
        for nm in wn:
            if nm not in ws:
                continue
            p = np.array(ws[nm])
            for gn in G:
                gg = G[gn]
                if gg['type'] == 6:
                    dd2 = _cap_dist(p, gg['pos'], gg['R'], gg['size'])
                else:
                    dd2 = _box_dist(p, gg['pos'], gg['R'], gg['size'])
                if wmin is None or dd2 < wmin:
                    wmin = dd2
                    wname = gn
        mg = r.get('joint_margins') or {}
        zero = []
        for nm in mg:
            v = mg[nm]
            if v is not None and v < 1e-4:
                zero.append(nm)
        ut = r.get('upper_target')
        yd = None
        if ut is not None and yaw_i is not None:
            yd = ut[yaw_i] - q0[yaw_i]
        row = {'branch': tag, 'err': ex, 'margin': r.get('joint_margin_min'),
               'margin_joint': r.get('joint_margin_joint'),
               'bounds': zero, 'wrist_min': wmin, 'wrist_collider': wname,
               'yaw_delta': yd}
        out[tag] = row
        if ex is not None and (best is None or ex < best['err']):
            best = row
            best['branch_name'] = tag
    out['best'] = best
    return out


def run(api, side='left', palm_off=0.07, fracs=(0.3, 0.5, 0.7, 0.9),
        rolls=(0, 90), approaches=('front', 'diag'), **kw):
    m, d = api.raw
    names = ('collider_box_f302734d', 'collider_box_8e968ea9',
             'collider_cylinder99aba387', 'collider_box_a1f69b3d')
    G = {}
    gi = 0
    while gi < m.ngeom:
        nm = m.geom(gi).name
        for cand in names:
            if cand in nm:
                G[cand] = {'pos': np.asarray(d.geom_xpos[gi]).copy(),
                           'R': np.asarray(d.geom_xmat[gi]).reshape(3, 3).copy(),
                           'size': np.asarray(m.geom_size[gi]).copy(),
                           'type': m.geom_type[gi]}
        gi = gi + 1
    h = G.get('collider_box_f302734d')
    if h is None:
        return {'ok': False, 'error': 'handle missing'}
    q0 = api.upper()
    yaw_i = _yaw_idx(api)
    rows = []
    for ap in approaches:
        for fr in fracs:
            for ro in rolls:
                rows.append(_cell(api, G, h, side, ap, fr, ro, palm_off,
                                  q0, yaw_i))
    return {'ok': True, 'side': side, 'yaw_now': None if yaw_i is None
            else q0[yaw_i], 'rows': rows,
            'base': api.base_state(),
            'object': (api.observe().get('object') or {}).get('pos')}
