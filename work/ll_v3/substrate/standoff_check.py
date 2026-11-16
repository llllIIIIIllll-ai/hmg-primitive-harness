def run(api, dx=-0.02, dy=0.07, z=1.06, orient='down', **kw):
    """Read-only: at the CURRENT standoff, solve the grasp target with the
    DLS preview and the seeded QP; report residuals, the binding joint and
    the base-to-drill margin (torso clearance proxy)."""
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
    ip = api.ik_preview(targets=tgt, orients=(orient,), iters=150,
                        sides=('left', 'right'))
    seed = {}
    perr = {}
    for row in (ip.get('rows') or []):
        qn = row.get('q_named') or {}
        for kk in qn:
            seed[kk] = qn[kk]
        perr[row.get('side')] = row.get('err')
    r = api.reach_box_qp(task_dim=12, targets=tgt, orientation=orient,
                         dur=0.02, apply=False, seed_q=seed)
    a = api.object_aabb()
    lo = np.array(a['lo'])
    hi = np.array(a['hi'])
    bp = np.array(b['pos'])
    near = np.clip(bp, lo, hi)
    base_margin = np.linalg.norm(bp - near)
    return {'ok': True, 'targets': tgt, 'preview_err': perr,
            'ik_err': r.get('ik_err'),
            'joint_margin_min': r.get('joint_margin_min'),
            'joint_margin_joint': r.get('joint_margin_joint'),
            'joint_margins': r.get('joint_margins'),
            'base_margin': base_margin, 'base': b, 'aabb': a}
