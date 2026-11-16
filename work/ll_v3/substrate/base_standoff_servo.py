def run(api, standoff=0.25, max_bursts=60, burst=0.4, speed=0.18,
        height=None, via=None, tilt_limit=8.0, finger_clearance=0.06,
        **kw):
    """Closed-loop standoff servo from the live object AABB.

    Waypoints (world [x, y]) are optional; the final target is the object AABB
    nearest-point standoff.  Guards: base tilt and fingertip clearance.
    """
    events = []
    if height is not None:
        api.set_height(height, 1.5)
        b = api.base_state()
        events.append({'event': 'height', 'height': height,
                       'tilt_deg': b['tilt_deg']})
        if b['tilt_deg'] > tilt_limit:
            return {'ok': False,
                    'binding_constraint': 'tilt_guard_after_height',
                    'events': events, 'tilt_deg': b['tilt_deg']}

    if via is not None:
        for p in via:
            k = 0
            while k < max_bursts:
                b = api.base_state()
                dx = p[0] - b['pos'][0]
                dy = p[1] - b['pos'][1]
                d = (dx * dx + dy * dy) ** 0.5
                if d <= 0.15:
                    break
                yaw = np.radians(b['yaw_deg'])
                dxr = dx * np.cos(yaw) + dy * np.sin(yaw)
                dyr = -dx * np.sin(yaw) + dy * np.cos(yaw)
                ye = np.arctan2(dyr, dxr)
                while ye > 3.14159:
                    ye = ye - 6.28318
                while ye < -3.14159:
                    ye = ye + 6.28318
                wz = np.clip(0.6 * ye, -0.15, 0.15)
                vy = np.clip(0.5 * dyr, -0.08, 0.08)
                vx = np.clip(0.5 * (d - 0.15), 0.04, speed)
                api.move_base(x=vx, y=vy, yaw=wz, dur=burst)
                b = api.base_state()
                if b['tilt_deg'] > tilt_limit:
                    return {'ok': False,
                            'binding_constraint': 'tilt_guard_waypoint',
                            'events': events, 'tilt_deg': b['tilt_deg'],
                            'waypoint': p}
                k = k + 1
            events.append({'event': 'waypoint', 'point': p, 'bursts': k,
                           'tilt_deg': api.base_state()['tilt_deg']})

    k = 0
    while k < max_bursts:
        st = api.observe()
        ob = st.get('object') or {}
        b = api.base_state()
        a = api.object_aabb()
        lo = np.array(a['lo'])
        hi = np.array(a['hi'])
        bp = np.array(b['pos'])
        near = np.clip(bp, lo, hi)
        da = np.linalg.norm(bp - near)
        if da <= standoff + 0.02:
            break
        hand = ob.get('hand') or {}
        for side in hand:
            hp = np.array(hand[side])
            hnear = np.clip(hp, lo, hi)
            if np.linalg.norm(hp - hnear) < finger_clearance:
                return {'ok': False,
                        'binding_constraint': 'fingertip_guard',
                        'events': events, 'standoff_aabb': da,
                        'base': b, 'object': ob.get('pos'),
                        'guard_side': side}
        dx = ob['pos'][0] - b['pos'][0]
        dy = ob['pos'][1] - b['pos'][1]
        yaw = np.radians(b['yaw_deg'])
        dxr = dx * np.cos(yaw) + dy * np.sin(yaw)
        dyr = -dx * np.sin(yaw) + dy * np.cos(yaw)
        ye = np.arctan2(dyr, dxr)
        while ye > 3.14159:
            ye = ye - 6.28318
        while ye < -3.14159:
            ye = ye + 6.28318
        wz = np.clip(0.6 * ye, -0.15, 0.15)
        vy = np.clip(0.5 * dyr, -0.08, 0.08)
        vx = np.clip(0.5 * (da - standoff), 0.04, speed)
        api.move_base(x=vx, y=vy, yaw=wz, dur=burst)
        b = api.base_state()
        if b['tilt_deg'] > tilt_limit:
            return {'ok': False, 'binding_constraint': 'tilt_guard',
                    'events': events, 'standoff_aabb': da, 'base': b,
                    'object': ob.get('pos')}
        k = k + 1

    st = api.observe()
    ob = st.get('object') or {}
    b = api.base_state()
    a = api.object_aabb()
    lo = np.array(a['lo'])
    hi = np.array(a['hi'])
    bp = np.array(b['pos'])
    near = np.clip(bp, lo, hi)
    da = np.linalg.norm(bp - near)
    away = bp - near
    dn = (away[0] * away[0] + away[1] * away[1]) ** 0.5
    if dn > 1e-6:
        ux = away[0] / dn
        uy = away[1] / dn
        desired = [near[0] + standoff * ux, near[1] + standoff * uy]
    else:
        desired = [b['pos'][0], b['pos'][1]]
    xy_err = ((b['pos'][0] - desired[0]) ** 2
              + (b['pos'][1] - desired[1]) ** 2) ** 0.5
    yaw_des = np.degrees(np.arctan2(ob['pos'][1] - b['pos'][1],
                                    ob['pos'][0] - b['pos'][0]))
    yaw_err = np.abs(b['yaw_deg'] - yaw_des)
    if yaw_err > 180.0:
        yaw_err = 360.0 - yaw_err
    ok = (da <= standoff + 0.05 and xy_err <= 0.08
          and b['tilt_deg'] <= tilt_limit)
    binding = None
    if not ok:
        if b['tilt_deg'] > tilt_limit:
            binding = 'tilt_guard'
        elif da > standoff + 0.05:
            binding = 'standoff'
        else:
            binding = 'xy_error'
    return {'ok': ok, 'standoff_aabb': da,
            'standoff_error': da - standoff,
            'final_xy_error': xy_err, 'yaw_error_deg': yaw_err,
            'tilt_deg': b['tilt_deg'], 'base': b,
            'object': ob.get('pos'),
            'object_aabb': {'lo': a['lo'], 'hi': a['hi']},
            'bursts': k, 'events': events,
            'binding_constraint': binding}
