def run(api, z=1.16, dx=0.0, dy=0.07, **kw):
    """Read-only: solve a top-down target (palms facing down) from the live
    pose and report ik_err, margin and the predicted wrist surface distance."""
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    tgt = {'left': [obj[0] + dx, obj[1] + dy, z],
           'right': [obj[0] + dx, obj[1] - dy, z]}
    r = api.reach_box_qp(task_dim=12, targets=tgt, orientation='down',
                         dur=0.02, apply=False)
    a = api.object_aabb()
    lo = np.array(a['lo'])
    hi = np.array(a['hi'])
    ag = api.arm_geoms()
    radii = {}
    for body in ag:
        for gg in ag[body]['geoms']:
            nm = gg['name']
            if (nm == 'robot0_g27_col' or nm == 'robot0_g28_col'
                    or nm == 'robot0_g34_col' or nm == 'robot0_g35_col'):
                sz = gg.get('local_size') or [0.0]
                tp = gg.get('type') or ''
                rad = sz[0]
                if tp == 'box' or tp == 'cylinder':
                    rad = (sz[0] * sz[0] + sz[1] * sz[1]
                           + sz[2] * sz[2]) ** 0.5
                radii[nm] = rad
    ws = r.get('wrist_solved') or {}
    out = {}
    for nm in ws:
        p = np.array(ws[nm])
        near = np.clip(p, lo, hi)
        out[nm] = np.linalg.norm(p - near) - radii.get(nm, 0.0)
    return {'ok': True, 't': api.t, 'targets': tgt,
            'ik_err': r.get('ik_err'), 'ik_rot': r.get('ik_rot'),
            'joint_margin_min': r.get('joint_margin_min'),
            'joint_margin_joint': r.get('joint_margin_joint'),
            'wrist_pred_surface': out}
