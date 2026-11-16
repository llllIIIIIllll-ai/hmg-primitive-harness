def run(api, standoff=0.25, lateral_target=None, max_bursts=60, burst=0.4,
        speed=0.18, height=None, via=None, tilt_limit=8.0,
        finger_clearance=0.06, lateral_tol=0.02, **kw):
    """W2b standoff servo: the base lateral pose follows the task target
    (palm midpoint y) instead of only the object AABB."""
    events = []
    if height is not None:
        api.set_height(height, 1.5)
        b = api.base_state()
        events.append({'event': 'height', 'height': height,
                       'tilt_deg': b['tilt_deg']})
        if b['tilt_deg'] > tilt_limit:
            return {'ok': False, 'binding_constraint': 'tilt_guard_after_height',
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
                    return {'ok': False, 'binding_constraint': 'tilt_guard_waypoint',
                            'events': events, 'tilt_deg': b['tilt_deg'],
                            'waypoint': p}
                k = k + 1
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
        lat_err = 0.0
        if lateral_target is not None:
            lat_err = np.abs(b['pos'][1] - lateral_target)
        if da <= standoff + 0.02 and lat_err <= lateral_tol:
            break
        hand = ob.get('hand') or {}
        for side in hand:
            hp = np.array(hand[side])
            hnear = np.clip(hp, lo, hi)
            if np.linalg.norm(hp - hnear) < finger_clearance:
                return {'ok': False, 'binding_constraint': 'fingertip_guard',
                        'events': events, 'standoff_aabb': da, 'base': b,
                        'object': ob.get('pos'), 'guard_side': side}
        dx = ob['pos'][0] - b['pos'][0]
        dy = (lateral_target - b['pos'][1]) if lateral_target is not None \
            else (ob['pos'][1] - b['pos'][1])
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
    lat_err = 0.0 if lateral_target is None else np.abs(
        b['pos'][1] - lateral_target)
    ok = (da <= standoff + 0.05 and lat_err <= 0.04
          and b['tilt_deg'] <= tilt_limit)
    binding = None
    if not ok:
        if b['tilt_deg'] > tilt_limit:
            binding = 'tilt_guard'
        elif da > standoff + 0.05:
            binding = 'standoff'
        else:
            binding = 'lateral_error'
    return {'ok': ok, 'standoff_aabb': da, 'standoff_error': da - standoff,
            'lateral_error': lat_err, 'lateral_target': lateral_target,
            'tilt_deg': b['tilt_deg'], 'base': b,
            'object': ob.get('pos'),
            'object_aabb': {'lo': a['lo'], 'hi': a['hi']},
            'bursts': k, 'events': events,
            'binding_constraint': binding}
