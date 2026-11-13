def run(api, **kw):
    """With the last commanded grip held, translate the hand until a stop
    condition on one world coordinate of the object fires.

    Mechanism: capped hand motion under a frozen hand command -- one capped
    move per live object read, stopped by the object coordinate crossing a
    bound in a chosen direction, optionally by the IK residual growing past
    ik_stop, or by the step cap.

    This one skill subsumes the two catalogued variants:
      stop='le' with clear=...   reproduces extract_held_horizontally
      stop='ge' with lift_z=...  reproduces lift_held_vertically
    The threshold keyword that is present selects the variant when 'stop' is
    not passed explicitly (extract's kwargs carry only 'clear', lift's kwargs
    carry only 'lift_z'), so each variant's own kwargs reproduce it verbatim.
    """
    hold = kw.get('hold')
    side = kw.get('side', 'left')
    # ---- stop mode: explicit 'stop' wins; else the threshold keyword that is
    #      present selects the variant.
    stop = kw.get('stop')
    if stop is None:
        if 'lift_z' in kw:
            stop = 'ge'
        else:
            stop = 'le'
    if stop == 'ge':
        stop_value = kw.get('lift_z', 0.82)
        axis_default = 2
        label_default = 'lift'
        d_default = [-0.004, 0.0, 0.024]
    else:
        stop_value = kw.get('clear', 0.34)
        axis_default = 0
        label_default = 'extract'
        d_default = [-0.032, 0.0, 0.0]
    axis = kw.get('axis', axis_default)
    label = kw.get('label', label_default)
    d = np.array(kw.get('d', d_default))
    ik_stop = kw.get('ik_stop')
    dur = kw.get('dur', 0.8)
    iters = kw.get('iters', 250)
    max_steps = kw.get('max_steps', 8)
    obj_tag = kw.get('obj_tag', 'drill')
    bar_tag = kw.get('bar_tag', 'holder')
    RB = kw.get('R_body', [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])

    def hand_cts():
        out = []
        for c in api.contact_pairs(limit=32):
            dd = (obj_tag in c['b1']) or (obj_tag in c['b2'])
            hh = (c['b1'] == 'holder_main') or (c['b2'] == 'holder_main')
            if dd and not hh:
                out.append([c['g1'], c['g2']])
        return out

    trace = []
    k = 0
    while k < max_steps:
        st = api.observe()
        drill = np.array(st['object']['pos'])
        if stop == 'ge':
            if drill[axis] >= stop_value:
                break
        else:
            if drill[axis] <= stop_value:
                break
        site = np.array(st['object']['hand'][side])
        r = api.reach_box(targets={side: (site + d).tolist()}, sides=(side,),
                          R_body=RB, dur=dur, iters=iters, hand_hold=hold)
        st = api.observe()
        g2 = api.geom()
        o2 = g2.get('object') or {}
        R2 = np.array(o2.get('xmat', np.eye(3)))
        hold_ct = []
        for c in api.contact_pairs(limit=32):
            dd = (obj_tag in c['b1'] or obj_tag in c['b2'])
            hh = (bar_tag in c['b1'] or bar_tag in c['b2'])
            if dd and hh:
                hold_ct.append([c['g1'], c['g2'],
                                np.round(np.array(c['pos']), 3).tolist()])
        trace.append([label, np.round(site + d, 3).tolist(),
                      st['object']['pos'], np.round(R2[2, 2], 3).tolist(),
                      r.get('ik_err'), hand_cts(), hold_ct])
        if ik_stop is not None:
            ike = r.get('ik_err') or {}
            if ike.get(side, 0.0) > ik_stop:
                break
        k += 1
    return {'steps': k, 'trace': trace}
