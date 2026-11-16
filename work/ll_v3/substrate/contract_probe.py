def run(api, apply=False, dur=0.06, **kw):
    """Call reach_box_qp once and return its raw result and the sim time."""
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    tgt = {'left': [obj[0] - 0.10, obj[1] + 0.10, obj[2] + 0.10],
           'right': [obj[0] - 0.10, obj[1] - 0.10, obj[2] + 0.10]}
    r = api.reach_box_qp(task_dim=12, targets=tgt, orientation='keep',
                         dur=dur, apply=apply)
    return {'ok': True, 't_after': api.t, 'qp': r}
