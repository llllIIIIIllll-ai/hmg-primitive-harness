def run(api, fwd=0.35, dy=0.07, z=1.16, orient='down', **kw):
    """Free-space multi-seed validation: ik_preview winning q per side, then
    seeded vs unseeded QP at the same target."""
    b = api.base_state()
    tgt = {'left': [b['pos'][0] + fwd, b['pos'][1] + dy, z],
           'right': [b['pos'][0] + fwd, b['pos'][1] - dy, z]}
    ip = api.ik_preview(targets=tgt, orients=(orient,), iters=150,
                        sides=('left', 'right'))
    seed = {}
    preview = {}
    for row in (ip.get('rows') or []):
        nm = row.get('q_named') or {}
        for k in nm:
            seed[k] = nm[k]
        preview[row.get('side')] = {'seed': row.get('seed'),
                                    'err': row.get('err'),
                                    'dq': row.get('dq')}
    r0 = api.reach_box_qp(task_dim=12, targets=tgt, orientation=orient,
                          dur=0.02, apply=False)
    r1 = api.reach_box_qp(task_dim=12, targets=tgt, orientation=orient,
                          dur=0.02, apply=False, seed_q=seed)
    return {'ok': True, 'targets': tgt, 'preview': preview,
            'unseeded': {'ik_err': r0.get('ik_err'),
                         'margin': r0.get('joint_margin_min'),
                         'joint': r0.get('joint_margin_joint')},
            'seeded': {'ik_err': r1.get('ik_err'),
                       'margin': r1.get('joint_margin_min'),
                       'joint': r1.get('joint_margin_joint')}}
