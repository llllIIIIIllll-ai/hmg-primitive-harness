def run(api, dx=-0.126, dy=0.07, z=1.16, orient='down', steps=60,
        cmd_clamp=0.02, **kw):
    """Preview-vs-live contract test.

    Compute the target, run the DLS preview, map the winning q into the
    contract order, apply it DIRECTLY to the live sim (clamped step_goal)
    and measure the live site error.  If the live error converges toward the
    preview error, the preview model matches the live model and the
    interpolation path is at fault; if it stays far, the DLS q is wrong in
    the live model (order/waist/hand coupling).
    """
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
    rows = ip.get('rows') or []
    seed = {}
    perr = {}
    pseed = {}
    for row in rows:
        qn = row.get('q_named') or {}
        for kk in qn:
            seed[kk] = qn[kk]
        perr[row.get('side')] = row.get('err')
        pseed[row.get('side')] = row.get('seed')
    q0 = api.upper()
    sv = q0.tolist()
    for nm in seed:
        if nm in api.addr:
            jj = api.addr[nm]
            kk = 0
            while kk < 31:
                if api.qpos_addr[kk] == jj:
                    sv[kk] = seed[nm]
                    break
                kk = kk + 1
    names = []
    for nm in seed:
        names.append(nm)
    names.sort()
    sample = []
    j = 0
    while j < 5 and j < 14:
        nm = names[j]
        jj = api.addr[nm]
        kk = 0
        while kk < 31:
            if api.qpos_addr[kk] == jj:
                sample.append({'joint': nm, 'idx': kk,
                               'dls': seed[nm], 'live': q0[kk]})
                break
            kk = kk + 1
        j = j + 1
    try:
        owner = api.waist_owner()
    except:
        owner = None
    tr = []
    k = 0
    while k < steps:
        q = api.upper()
        cmd = q.copy()
        i = 0
        while i < 31:
            d = sv[i] - q[i]
            if d > cmd_clamp:
                d = cmd_clamp
            if d < -cmd_clamp:
                d = -cmd_clamp
            cmd[i] = q[i] + d
            i = i + 1
        api.step_goal(upper=cmd, nav=[0.0, 0.0, 0.0], dur=0.02)
        g = api.geom()
        pe = {}
        for side in tgt:
            site = g['sites'].get(side)
            t = tgt[side]
            v = [site['pos'][0] - t[0], site['pos'][1] - t[1],
                 site['pos'][2] - t[2]]
            pe[side] = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
        tr.append({'k': k, 'site': pe, 't': api.t,
                   'base': api.base_state()['pos']})
        k = k + 1
    jr = {}
    q1 = api.upper()
    for nm in seed:
        jj = api.addr[nm]
        kk = 0
        while kk < 31:
            if api.qpos_addr[kk] == jj:
                jr[nm] = {'want': seed[nm], 'live_now': q1[kk],
                          'live_before': q0[kk]}
                break
            kk = kk + 1
    return {'ok': True, 'targets': tgt, 'preview_err': perr,
            'preview_seed': pseed, 'sample_joints': sample,
            'waist_owner': owner, 'base_before': b,
            'trace': tr, 'joints_after': jr}
