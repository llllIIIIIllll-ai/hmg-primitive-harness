def run(api, **kw):
    """Creep a measured hand site toward a standoff point expressed in the
    object's own body frame, aborting if the object is displaced.

    Mechanism: contact-guarded incremental approach -- one small step per
    live object-frame read, stopped by the measured hand-site tolerance or by
    the object having moved more than move_tol.  Removes the failure where a
    coarse reach knocks a held object out of its cradle before the fingers
    are near it.
    """
    H_OFF = np.array(kw.get('h_off', [-0.0266, -0.0003, 0.1179]))
    off = np.array(kw.get('offset', [-0.083, 0.047, 0.012]))
    side = kw.get('side', 'left')
    tol = kw.get('tol', 0.004)
    step_cap = kw.get('step', 0.005)
    dur = kw.get('dur', 0.35)
    iters = kw.get('iters', 220)
    max_steps = kw.get('max_steps', 40)
    move_tol = kw.get('move_tol', 0.02)
    obj_tag = kw.get('obj_tag', 'drill')
    holder_tag = kw.get('holder_tag', 'holder_main')
    RB = kw.get('R_body', [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])

    def handle_now():
        g = api.geom()
        o = g.get('object') or {}
        R = np.array(o.get('xmat', np.eye(3)))
        return np.array(o.get('pos')) + R @ H_OFF

    def hand_cts():
        out = []
        for c in api.contact_pairs(limit=32):
            dd = (obj_tag in c['b1']) or (obj_tag in c['b2'])
            hh = (c['b1'] == holder_tag) or (c['b2'] == holder_tag)
            if dd and not hh:
                out.append([c['g1'], c['g2']])
        return out

    drill_start = np.array(api.observe()['object']['pos'])
    j = 0
    stopped = None
    trace = []
    site = drill_start
    while j < max_steps:
        st = api.observe()
        site = np.array(st['object']['hand'][side])
        drill = np.array(st['object']['pos'])
        hnd = handle_now()
        drc = hand_cts()
        dstart = np.sqrt(np.sum((drill - drill_start) ** 2))
        if dstart > move_tol:
            stopped = 'moved'
            trace.append(['creep_stop', j, np.round(site, 4).tolist(),
                          np.round(dstart * 1e3, 2).tolist(), drc])
            break
        tgt = hnd + off
        err = tgt - site
        errn = np.sqrt(np.sum(err * err))
        if errn < tol:
            stopped = 'target'
            trace.append(['creep_target', j, np.round(site, 4).tolist(), drc])
            break
        step = err * (step_cap / errn) if errn > step_cap else err
        api.reach_box(targets={side: (site + step).tolist()}, sides=(side,),
                      R_body=RB, dur=dur, iters=iters)
        j += 1
    trace.append(['creep_done', stopped, j])
    return {'steps': j, 'stopped': stopped, 'trace': trace}
