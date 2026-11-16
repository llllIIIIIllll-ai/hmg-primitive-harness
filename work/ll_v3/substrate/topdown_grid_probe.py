def run(api, z=1.06, dy=0.07, roll=0.0, dx=-0.02, **kw):
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
    tgt = {'left': [obj[0] + dx * fx + dy * px,
                    obj[1] + dx * fy + dy * py, z],
           'right': [obj[0] + dx * fx - dy * px,
                     obj[1] + dx * fy - dy * py, z]}
    q0 = np.asarray(api.raw[1].qpos[3:7])
    Rb = api._quat_to_mat(q0)
    rr = np.radians(roll)
    Rz = np.array([[np.cos(rr), -np.sin(rr), 0.0],
                   [np.sin(rr), np.cos(rr), 0.0],
                   [0.0, 0.0, 1.0]])
    R_Y90 = np.array([[0.0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    rb = {}
    for side in ('left', 'right'):
        R_site0 = np.asarray(api.R_site0[side])
        R_des = Rz @ Rb @ R_Y90 @ R_site0
        Rbod = Rb.T @ R_des @ R_site0.T
        U, S, Vt = np.linalg.svd(Rbod)
        rb[side] = (U @ Vt).tolist()
    ip = api.ik_preview(targets=tgt, R_body=rb, orients=('keep',),
                        iters=150, sides=('left', 'right'))
    seed = {}
    for row in (ip.get('rows') or []):
        qn = row.get('q_named') or {}
        for kk in qn:
            seed[kk] = qn[kk]
    r = api.reach_box_qp(task_dim=12, targets=tgt, R_body=rb,
                         orientation='keep', dur=0.02, apply=False,
                         seed_q=seed)
    return {'ok': True, 'z': z, 'dy': dy, 'roll': roll,
            'ik_err': r.get('ik_err'),
            'joint_margin_min': r.get('joint_margin_min'),
            'joint_margin_joint': r.get('joint_margin_joint'),
            'base': b['pos']}
