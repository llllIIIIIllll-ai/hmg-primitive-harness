def run(api, **kw):
    """Raise the base height while re-commanding the last commanded upper-body
    vector, so a held object rises without using arm reach.

    Mechanism: reach-independent height boost under a frozen 31-D upper
    command -- step_goal with the commanded vector and a taller base, then
    read the object.  Re-sending the commanded vector keeps the grip from
    being relaxed by the height ramp.  Removes the failure where an arm lift
    stalls at the arm reach limit while the object is still below the pick
    height.
    """
    height = kw.get('height', 0.87)
    dur = kw.get('dur', 1.6)
    side = kw.get('side', 'left')
    obj_tag = kw.get('obj_tag', 'drill')
    hold = api.commanded()
    trace = []
    if hold is None:
        return {'raised': False, 'trace': trace}
    api.step_goal(upper=hold, nav=None, height=height, dur=dur)
    st = api.observe()
    g3 = api.geom()
    o3 = g3.get('object') or {}
    R3 = np.array(o3.get('xmat', np.eye(3)))
    allc = []
    for c in api.contact_pairs(limit=32):
        dd = (obj_tag in c['b1']) or (obj_tag in c['b2'])
        if dd:
            allc.append([c['b1'], c['b2']])
    trace.append(['raise', st['object']['pos'], np.round(R3[2, 2], 3).tolist(),
                  st['object']['hand'][side], st['base'], allc])
    return {'raised': True, 'trace': trace, 'drill': st['object']['pos'],
            'hand': st['object']['hand'], 'base': st['base']}
