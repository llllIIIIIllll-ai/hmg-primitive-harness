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


def run(api, side='right', frac=0.5, roll=0, zoff=0.07, **kw):
    m, d = api.raw
    names = ('collider_box_f302734d', 'collider_box_8e968ea9',
             'collider_cylinder99aba387', 'collider_box_a1f69b3d')
    G = {}
    gi = 0
    while gi < m.ngeom:
        nm = m.geom(gi).name
        for cand in names:
            if cand in nm:
                G[cand] = {'pos': np.asarray(
                    d.geom_xpos[gi]).copy(),
                    'R': np.asarray(d.geom_xmat[gi]).reshape(
                        3, 3).copy(),
                    'size': np.asarray(m.geom_size[gi]).copy(),
                    'type': m.geom_type[gi]}
        gi = gi + 1
    h = G.get('collider_box_f302734d')
    if h is None:
        return {'ok': False, 'error': 'handle geom missing'}
    axis = h['R'][:, 2]
    pt = h['pos'] + (frac - 0.5) * 2.0 * h['size'][2] * axis
    tgt = {side: [pt[0], pt[1], pt[2] + zoff]}
    q0 = np.asarray(api.raw[1].qpos[3:7])
    Rb = api._quat_to_mat(q0)
    rr = np.radians(roll)
    Rz = np.array([[np.cos(rr), -np.sin(rr), 0.0],
                   [np.sin(rr), np.cos(rr), 0.0],
                   [0.0, 0.0, 1.0]])
    R_Y90 = np.array([[0.0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    R_site0 = np.asarray(api.R_site0[side])
    R_des = Rz @ Rb @ R_Y90 @ R_site0
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
    r = api.reach_box_qp(task_dim=12, targets=tgt, R_body=rb,
                         orientation='keep', dur=0.02, apply=False,
                         seed_q=seed, sides=(side,))
    ws = r.get('wrist_solved') or {}
    wn = (('robot0_g27_col', 'robot0_g28_col') if side == 'left'
          else ('robot0_g34_col', 'robot0_g35_col'))
    wrist_min = None
    wrist_by = {}
    for nm in wn:
        if nm not in ws:
            continue
        p = np.array(ws[nm])
        mn = None
        for gn in G:
            g = G[gn]
            if g['type'] == 6:
                dd = _cap_dist(p, g['pos'], g['R'], g['size'])
            else:
                dd = _box_dist(p, g['pos'], g['R'], g['size'])
            if mn is None or dd < mn:
                mn = dd
        wrist_by[nm] = mn
        if wrist_min is None or mn < wrist_min:
            wrist_min = mn
    padn = 'gripper0_%s_g0_col' % side
    pf = (r.get('pad_frame') or {}).get(padn)
    pad_surf = None
    pad_n = None
    if pf is not None:
        pc = np.array(pf['pos'])
        R = np.array(pf['xmat'])
        pad_n = R[:, 0]
        hgt = 0.0
        hg = api.hand_geoms()
        for body in hg:
            for gg in hg[body]['geoms']:
                if gg['name'] == padn:
                    hgt = (gg.get('local_size') or [0.0])[0]
        ps = pc - pad_n * hgt
        pad_surf = _box_dist(ps, h['pos'], h['R'], h['size'])
    return {'ok': True, 'side': side, 'frac': frac, 'roll': roll,
            'target': tgt[side], 'handle_pos': h['pos'].tolist(),
            'ik': (r.get('ik_err') or {}).get(side),
            'margin': r.get('joint_margin_min'),
            'joint': r.get('joint_margin_joint'),
            'wrist_min': wrist_min, 'wrist_by': wrist_by,
            'pad_surface': pad_surf,
            'pad_normal': None if pad_n is None else pad_n.tolist()}
