def run(api, steps=100, **kw):
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    k = 0
    while k < steps:
        tgt = {'left': [obj[0] - 0.20, obj[1] + 0.15, obj[2] + 0.20],
               'right': [obj[0] - 0.20, obj[1] - 0.15, obj[2] + 0.20]}
        r = api.reach_box_qp(task_dim=12, targets=tgt, orientation='keep',
                             dur=0.02, apply=False)
        k = k + 1
    return {'ok': True, 'steps': steps, 't': api.t}
