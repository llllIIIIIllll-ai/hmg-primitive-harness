def run(api, **kw):
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    tgt = {'left': [obj[0] - 0.10, obj[1] + 0.10, obj[2] + 0.15],
           'right': [obj[0] - 0.10, obj[1] - 0.10, obj[2] + 0.15]}
    ip = api.ik_preview(targets=tgt, orients=('keep',), iters=50,
                        sides=('left', 'right'))
    rows = ip.get('rows') or []
    out = []
    for row in rows:
        out.append({'side': row.get('side'), 'err': row.get('err'),
                    'seed': row.get('seed'), 'qn': row.get('q_named')})
    return {'ok': True, 'rows': out}
