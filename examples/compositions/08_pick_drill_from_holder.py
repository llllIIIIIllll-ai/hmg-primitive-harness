def run(api, **kw):
    # Task program: walk to the holder -> servo to a clear mid point above and
    # +y of the handle -> creep in to the pad site -> ramp the fingers closed
    # -> extract horizontally clear of the holder -> lift -> boost the base.
    # Only api.skill() calls and glue; every geometrical target is derived
    # from a live object read.
    H_OFF = np.array([-0.0266, -0.0003, 0.1179])   # drill_main -> handle centre
    RB = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    obj_tag = kw.get('obj_tag', 'drill')
    holder_tag = kw.get('holder_tag', 'holder_main')
    log = []

    def hand_cts():
        out = []
        for c in api.contact_pairs(limit=32):
            dd = (obj_tag in c['b1']) or (obj_tag in c['b2'])
            hh = (c['b1'] == holder_tag) or (c['b2'] == holder_tag)
            if dd and not hh:
                out.append([c['g1'], c['g2']])
        return out

    # ---- 1) walk forward to the pre-grasp standoff
    w = api.skill('walk_base_to_coordinate', axis=0,
                  target=kw.get('walk_x', 0.05),
                  v=kw.get('walk_v', 0.18), lat=0.0, yaw=0.0,
                  dur=0.5, max_bursts=8)
    log.extend(w['trace'])
    # ---- 2) mid point well +y of the handle (fingers clear), from ONE
    #         live object read
    g = api.geom()
    o = g.get('object') or {}
    R = np.array(o.get('xmat', np.eye(3)))
    h0 = np.array(o.get('pos')) + R @ H_OFF
    mid = h0 + np.array([-0.079, 0.144, 0.012])
    s = api.skill('servo_hand_site_to_point', target=mid.tolist(), side='left',
                  tol=0.006, step=0.022, dur=kw.get('durS', 0.5), iters=220,
                  max_steps=12, R_body=RB)
    st = api.observe()
    log.append(['mid_done', s['steps'], st['object']['hand']['left'],
                st['object']['pos'], hand_cts()])
    # ---- 3) creep straight in to the pad-contact site
    c = api.skill('creep_to_object_standoff', h_off=H_OFF.tolist(),
                  offset=[-0.083, 0.047, 0.012], side='left', tol=0.004,
                  step=0.005, dur=kw.get('durC', 0.35), iters=220,
                  max_steps=kw.get('maxC', 40),
                  move_tol=kw.get('move_tol', 0.02), R_body=RB,
                  obj_tag=obj_tag, holder_tag=holder_tag)
    log.extend(c['trace'])
    # ---- 4) close in stages
    r4 = api.skill('ramp_close_fingers',
                   angles=kw.get('angles', [0.3, 0.6, 0.9, 1.2]), dur=0.5,
                   side='left', obj_tag=obj_tag, holder_tag=holder_tag)
    log.extend(r4['trace'])
    # ---- 5) pull out and lift with the closed-hand vector held
    hold = api.commanded()
    if hold is None:
        return {'error': 'no commanded vector after close', 'log': log}
    e = api.skill('move_held', hold=hold, axis=0,
                  clear=kw.get('clear_x', 0.34), d=[-0.032, 0.0, 0.0],
                  side='left', dur=kw.get('durP', 0.8), iters=250,
                  max_steps=kw.get('maxX', 8), R_body=RB, obj_tag=obj_tag,
                  bar_tag='holder')
    log.extend(e['trace'])
    lf = api.skill('move_held', hold=hold, axis=2,
                   lift_z=kw.get('lift_z', 0.82),
                   d=[kw.get('lift_dx', -0.004), 0.0, 0.024], side='left',
                   dur=kw.get('durP', 0.8), iters=250,
                   max_steps=kw.get('maxL', 8),
                   ik_stop=kw.get('ik_stop', 0.003), R_body=RB,
                   obj_tag=obj_tag, bar_tag='holder')
    log.extend(lf['trace'])
    # ---- 6) base-height boost with the closed-hand command preserved
    b = api.skill('boost_base_height_holding_command',
                  height=kw.get('pick_height', 0.87),
                  dur=kw.get('durH', 1.6), side='left', obj_tag=obj_tag)
    log.extend(b['trace'])
    st = api.observe()
    g = api.geom()
    o = g.get('object') or {}
    return {'log': log, 'final_drill': o.get('pos'),
            'final_xmat': o.get('xmat'), 'final_hand': st['object']['hand'],
            'final_t': api.t, 'base': api.base_state()}
