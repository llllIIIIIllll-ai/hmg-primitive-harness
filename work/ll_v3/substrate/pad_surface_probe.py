def run(api, side='right', dx=-0.12, dy=0.0, z=0.92, orient='down', **kw):
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
    ip = api.ik_preview(targets=tgt, orients=(orient,), iters=150,
                        sides=(side,))
    seed = {}
    for row in (ip.get('rows') or []):
        qn = row.get('q_named') or {}
        for kk in qn:
            seed[kk] = qn[kk]
    r = api.reach_box_qp(task_dim=12, targets=tgt, orientation=orient,
                         dur=0.02, apply=False, seed_q=seed, sides=(side,))
    a = api.object_aabb()
    lo = np.array(a['lo'])
    hi = np.array(a['hi'])
    ws = r.get('wrist_solved') or {}
    padn = 'gripper0_%s_g0_col' % side
    pad = ws.get(padn)
    out = {'ok': True, 'target': tgt[side], 'aabb': a,
           'pad_center': pad}
    if pad is not None:
        p3 = np.array(pad)
        near = np.clip(p3, lo, hi)
        out['pad_center_dist'] = np.linalg.norm(p3 - near)
        face = []
        if p3[0] < lo[0]:
            face.append('-x')
        if p3[0] > hi[0]:
            face.append('+x')
        if p3[1] < lo[1]:
            face.append('-y')
        if p3[1] > hi[1]:
            face.append('+y')
        if p3[2] < lo[2]:
            face.append('-z')
        if p3[2] > hi[2]:
            face.append('+z')
        out['pad_outside_faces'] = face
    hg = api.hand_geoms()
    for body in hg:
        for gg in hg[body]['geoms']:
            if gg['name'] == padn:
                out['pad_size'] = gg.get('local_size')
                out['pad_normal'] = np.array(gg['xmat'])[:, 0].tolist()
    out['ik_err'] = r.get('ik_err')
    out['wrist_solved'] = ws
    return out
