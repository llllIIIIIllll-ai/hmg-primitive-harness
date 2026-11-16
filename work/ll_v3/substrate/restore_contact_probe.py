def run(api, q=None, steps=90, **kw):
    out = {'ok': True, 'restored': False}
    if q:
        n = 0
        while n < steps:
            api.step_goal(upper=q, nav=[0.0, 0.0, 0.0], dur=0.02)
            n = n + 1
        out['restored'] = True
        out['steps'] = steps
    out['t'] = api.t
    out['upper'] = api.upper().tolist()
    out['joint_ranges'] = api.joint_ranges()
    out['contacts_all'] = api.contact_detail(limit=80)
    out['base'] = api.base_state()
    st = api.observe()
    out['object'] = st.get('object')
    agg = api.arm_geoms()
    w = {}
    for body in agg:
        for gg in agg[body]['geoms']:
            if gg['name'] in ('robot0_g27_col', 'robot0_g28_col'):
                w[gg['name']] = gg['world_pos']
    out['wrist_geoms'] = w
    return out
