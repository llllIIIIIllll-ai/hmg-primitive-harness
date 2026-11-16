def run(api, dx=-0.126, dy=0.07, z=1.16, side='left', **kw):
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
    T = [obj[0] + dx * fx + dy * px, obj[1] + dx * fy + dy * py, z]
    ip = api.ik_preview(targets={side: T}, orients=('down',), iters=150,
                        sides=(side,))
    rr = api.joint_ranges()
    out = {'ok': True, 't': api.t, 'target': T, 'rows': []}
    for r in (ip.get('rows') or []):
        qn = r.get('q_named') or {}
        d = {}
        for nm in qn:
            if nm in rr:
                qnow = rr[nm].get('q')
                d[nm] = {'q_now': qnow, 'q_seed': qn[nm],
                         'delta': (None if qnow is None else qn[nm] - qnow)}
        out['rows'].append({'side': r.get('side'), 'err': r.get('err'),
                            'dq': r.get('dq'), 'delta': d})
    return out
