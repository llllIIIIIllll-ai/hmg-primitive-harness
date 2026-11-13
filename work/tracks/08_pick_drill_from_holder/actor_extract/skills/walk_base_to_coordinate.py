def run(api, **kw):
    """Closed-loop base translation to one world coordinate.

    Mechanism: one burst of move_base per fresh base-state read, repeated
    until the measured coordinate reaches a target.  Removes the open-loop
    failure where a fixed walk command overshoots or stops short because the
    displacement the WBC tracks is not the commanded one.
    """
    axis = kw.get('axis', 0)
    target = kw.get('target', 0.05)
    v = kw.get('v', 0.18)
    lat = kw.get('lat', 0.0)
    yawv = kw.get('yaw', 0.0)
    dur = kw.get('dur', 0.5)
    max_bursts = kw.get('max_bursts', 8)
    trace = []
    i = 0
    while i < max_bursts:
        b = api.base_state()
        trace.append(['walk', b['pos'], b['yaw_deg']])
        if b['pos'][axis] >= target:
            break
        api.move_base(x=v, y=lat, yaw=yawv, dur=dur)
        i += 1
    return {'bursts': i, 'base': api.base_state(), 'trace': trace}
