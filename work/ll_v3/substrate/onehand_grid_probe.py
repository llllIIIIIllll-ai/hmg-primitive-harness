def run(api, side='left', dx=-0.08, dy=0.0, z=0.92, orient='front', **kw):
    """Read-only one-hand grasp feasibility cell."""
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
    o = orient
    if orient == 'front':
        g = api.geom()
        hg = api.hand_geoms()
        q0 = np.asarray(api.raw[1].qpos[3:7])
        Rb = api._quat_to_mat(q0)
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
        f = np.array([fx, fy, 0.0])
        f = f / np.linalg.norm(f)
        n = np.array([-px, -py, 0.0]) if side == 'left' else np.array(
            [px, py, 0.0])
        n = n - np.dot(n, f) * f
        n = n / np.linalg.norm(n)
        B_des = np.column_stack([f, n, np.cross(f, n)])
        R_des = B_des @ B_local.T
        U, S, Vt = np.linalg.svd(R_des)
        R_des = U @ Vt
        Rbod = Rb.T @ R_des @ np.asarray(api.R_site0[side]).T
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
    ws = r.get('wrist_solved') or {}
    names = ('robot0_g27_col', 'robot0_g28_col') if side == 'left' else (
        'robot0_g34_col', 'robot0_g35_col')
    wmin = None
    for nm in names:
        if nm not in ws:
            continue
        p = np.array(ws[nm])
        near = np.clip(p, lo, hi)
        s = np.linalg.norm(p - near)
        if wmin is None or s < wmin:
            wmin = s
    ik = r.get('ik_err') or {}
    return {'ok': True, 'side': side, 'dx': dx, 'dy': dy, 'z': z,
            'orient': orient, 'ik_err': ik.get(side),
            'joint_margin_min': r.get('joint_margin_min'),
            'joint_margin_joint': r.get('joint_margin_joint'),
            'wrist_surface': wmin}
