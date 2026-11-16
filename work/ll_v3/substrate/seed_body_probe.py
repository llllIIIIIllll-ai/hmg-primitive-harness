def run(api, **kw):
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    tgt = {'left': [obj[0] - 0.10, obj[1] + 0.10, obj[2] + 0.15],
           'right': [obj[0] - 0.10, obj[1] - 0.10, obj[2] + 0.15]}
    ip = api.ik_preview(targets=tgt, orients=('keep',), iters=50,
                        sides=('left', 'right'))
    seed = {}
    for row in (ip.get('rows') or []):
        qn = row.get('q_named') or {}
        for k in qn:
            seed[k] = qn[k]
    rb = {'left': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
          'right': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}
    r = api.reach_box_qp(task_dim=12, targets=tgt, R_body=rb,
                         orientation='keep', dur=0.02, apply=False,
                         seed_q=seed)
    return {'ok': True, 'ik_err': r.get('ik_err'),
            'margin': r.get('joint_margin_min'),
            'joint': r.get('joint_margin_joint'),
            'n_seed': 0}
