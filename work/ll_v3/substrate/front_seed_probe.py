def run(api, target=None, side='left', **kw):
    t = target or [1.5270310832760756, 0.009465126967308838, 0.954]
    tg = {side: t}
    ip = api.ik_preview(targets=tg, orients=('keep',), iters=150,
                        sides=(side,))
    rows = ip.get('rows') or []
    q = api.upper().tolist()
    rr = api.joint_ranges()
    order = []
    for nm in rr:
        order.append(nm)
    out = {'ok': True, 't': api.t, 'target': t, 'rows': [], 'q': q}
    for r in rows:
        qn = r.get('q_named') or {}
        d = {}
        for nm in qn:
            if nm in rr:
                qnow = rr[nm].get('q')
                d[nm] = {'q_now': qnow, 'q_seed': qn[nm],
                         'delta': (None if qnow is None
                                   else qn[nm] - qnow)}
        out['rows'].append({'side': r.get('side'), 'err': r.get('err'),
                            'dq': r.get('dq'), 'delta': d})
    return out
