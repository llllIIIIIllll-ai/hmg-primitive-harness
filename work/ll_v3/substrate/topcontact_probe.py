def run(api, side='right', dx=-0.03, dy=0.0, z=1.05, roll=0, **kw):
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
    tgt = {side: [obj[0] + dx * fx + dy * px,
                  obj[1] + dx * fy + dy * py, z]}
    rb = None
    o = 'down'
    if roll != 0:
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
        o = 'keep'
    ip = api.ik_preview(targets=tgt, R_body=rb, orients=(o,), iters=150,
                        sides=(side,))
    seed = {}
    for row in (ip.get('rows') or []):
        qn = row.get('q_named') or {}
        for kk in qn:
            seed[kk] = qn[kk]
    r = api.reach_box_qp(task_dim=12, targets=tgt, R_body=rb,
                         orientation=o, dur=0.02, apply=False,
                         seed_q=seed, sides=(side,))
    a = api.object_aabb()
    lo = np.array(a['lo'])
    hi = np.array(a['hi'])
    pf = (r.get('pad_frame') or {}).get('gripper0_%s_g0_col' % side)
    h = 0.0
    hg = api.hand_geoms()
    for body in hg:
        for gg in hg[body]['geoms']:
            if gg['name'] == 'gripper0_%s_g0_col' % side:
                sz = gg.get('local_size') or [0.0]
                h = sz[0]
    out = {'ok': True, 'side': side, 'dx': dx, 'dy': dy, 'z': z,
           'roll': roll,
           'ik': (r.get('ik_err') or {}).get(side),
           'margin': r.get('joint_margin_min'),
           'joint': r.get('joint_margin_joint')}
    if pf is not None:
        pc = np.array(pf['pos'])
        R = np.array(pf['xmat'])
        n = R[:, 0]
        ps = pc - n * h
        near = np.clip(ps, lo, hi)
        out['pad_center'] = pf['pos']
        out['pad_normal'] = n.tolist()
        out['half_thick'] = h
        out['pad_surface_dist'] = np.linalg.norm(ps - near)
        ws = r.get('wrist_solved') or {}
        names = (('robot0_g27_col', 'robot0_g28_col') if side == 'left'
                 else ('robot0_g34_col', 'robot0_g35_col'))
        wmin = None
        for nm in names:
            if nm not in ws:
                continue
            pp = np.array(ws[nm])
            nn = np.clip(pp, lo, hi)
            s = np.linalg.norm(pp - nn)
            if wmin is None or s < wmin:
                wmin = s
        out['wrist_surface'] = wmin
        face = []
        if ps[0] < lo[0]:
            face.append('-x')
        if ps[0] > hi[0]:
            face.append('+x')
        if ps[1] < lo[1]:
            face.append('-y')
        if ps[1] > hi[1]:
            face.append('+y')
        if ps[2] < lo[2]:
            face.append('-z')
        if ps[2] > hi[2]:
            face.append('+z')
        out['faces'] = face
    return out
