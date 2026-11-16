def run(api, standoff=0.25, lateral_target=None, max_bursts=60, burst=0.4,
        speed=0.18, height=None, via=None, tilt_limit=8.0,
        finger_clearance=0.06, lateral_tol=0.02, object_match=None,
        trace=True, **kw):
    """W2d standoff servo with a phase-resolved object/contact trace.

    The base lateral pose follows the task-target midpoint; every burst logs
    the live object pose and the robot-object contact pairs so the phase and
    colliding link that move the object are named.
    """
    events = []
    tr = []
    if trace:
        st0 = api.observe()
        ob0 = st0.get('object') or {}
        tr.append({'phase': 'servo_start', 't': api.t,
                   'object': ob0.get('pos'),
                   'contacts': api.contact_detail(
                       limit=40, match=object_match) if object_match else []})
    if height is not None:
        api.set_height(height, 1.5)
        b = api.base_state()
        events.append({'event': 'height', 'height': height,
                       'tilt_deg': b['tilt_deg']})
        if b['tilt_deg'] > tilt_limit:
            return {'ok': False, 'binding_constraint': 'tilt_guard_after_height',
                    'events': events, 'trace': tr,
                    'tilt_deg': b['tilt_deg']}
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
                if trace:
                    st = api.observe()
                    ob = st.get('object') or {}
                    tr.append({'phase': 'servo_via', 'burst': k, 't': api.t,
                               'waypoint': p, 'object': ob.get('pos'),
                               'contacts': api.contact_detail(
                                   limit=40, match=object_match)
                               if object_match else []})
                if b['tilt_deg'] > tilt_limit:
                    return {'ok': False, 'binding_constraint': 'tilt_guard_waypoint',
                            'events': events, 'trace': tr,
                            'tilt_deg': b['tilt_deg'], 'waypoint': p}
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
                        'events': events, 'trace': tr, 'standoff_aabb': da,
                        'base': b, 'object': ob.get('pos'),
                        'guard_side': side}
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
        if trace:
            st = api.observe()
            ob = st.get('object') or {}
            hg = api.arm_geoms()
            pz = None
            pd = None
            cd = None
            wl = None
            wr = None
            for body in hg:
                for gg in hg[body]['geoms']:
                    p3 = np.array(gg['world_pos'])
                    near3 = np.clip(p3, lo, hi)
                    dd3 = np.linalg.norm(p3 - near3)
                    if cd is None or dd3 < cd:
                        cd = dd3
                    nm_ = gg['name']
                    if (nm_ == 'robot0_g27_col' or nm_ == 'robot0_g28_col'
                            or nm_ == 'robot0_g34_col'
                            or nm_ == 'robot0_g35_col'):
                        # SURFACE distance: subtract the collision geom's
                        # radius.  The wrist capsule has r ~ 0.047 m, so a
                        # centre distance of 0.05 is already contact.
                        sz3 = gg.get('local_size') or [0.0]
                        tp3 = gg.get('type') or ''
                        rad3 = sz3[0]
                        if tp3 == 'box' or tp3 == 'cylinder':
                            rad3 = (sz3[0] * sz3[0] + sz3[1] * sz3[1]
                                    + sz3[2] * sz3[2]) ** 0.5
                        s3 = dd3 - rad3
                        if (nm_ == 'robot0_g27_col'
                                or nm_ == 'robot0_g28_col'):
                            if wl is None or s3 < wl:
                                wl = s3
                        else:
                            if wr is None or s3 < wr:
                                wr = s3
                    if nm_.startswith('gripper0_') and nm_.endswith(
                            '_g0_col'):
                        if pd is None or dd3 < pd:
                            pd = dd3
                        zz = gg['world_pos'][2]
                        if pz is None or zz < pz:
                            pz = zz
            tr.append({'phase': 'servo_burst', 'burst': k, 't': api.t,
                       'object': ob.get('pos'),
                       'pad_z_min': pz,
                       'pad_clearance': None if pz is None else pz - hi[2],
                       'pad_aabb_dist': pd,
                       'chain_aabb_dist': cd,
                       'wrist_aabb_dist': {'left': wl, 'right': wr},
                       'contacts': api.contact_detail(
                           limit=40, match=object_match)
                       if object_match else []})
        if b['tilt_deg'] > tilt_limit:
            return {'ok': False, 'binding_constraint': 'tilt_guard',
                    'events': events, 'trace': tr, 'standoff_aabb': da,
                    'base': b, 'object': ob.get('pos')}
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
    if trace:
        tr.append({'phase': 'servo_end', 't': api.t,
                   'object': ob.get('pos'),
                   'contacts': api.contact_detail(
                       limit=40, match=object_match) if object_match else []})
    ok = True if (da <= standoff + 0.05 and lat_err <= 0.04
                   and b['tilt_deg'] <= tilt_limit) else False
    binding = None
    if not ok:
        if b['tilt_deg'] > tilt_limit:
            binding = 'tilt_guard'
        elif da > standoff + 0.05:
            binding = 'standoff'
        else:
            binding = 'lateral_error'
    clear_min = None
    dist_min = None
    wrist_min = None
    wrist_end = None
    for e in tr:
        c = e.get('pad_clearance')
        if c is not None and (clear_min is None or c < clear_min):
            clear_min = c
        dd = e.get('pad_aabb_dist')
        if dd is not None and (dist_min is None or dd < dist_min):
            dist_min = dd
        w = e.get('wrist_aabb_dist')
        if w:
            for side in w:
                v = w[side]
                if v is None:
                    continue
                if wrist_min is None or v < wrist_min:
                    wrist_min = v
            wrist_end = w
    return {'ok': ok, 'standoff_aabb': da, 'standoff_error': da - standoff,
            'pad_clearance_min': clear_min,
            'pad_aabb_dist_min': dist_min,
            'wrist_aabb_dist': wrist_end,
            'wrist_aabb_dist_min': wrist_min,
            'lateral_error': lat_err, 'lateral_target': lateral_target,
            'tilt_deg': b['tilt_deg'], 'base': b, 'object': ob.get('pos'),
            'object_aabb': {'lo': a['lo'], 'hi': a['hi']},
            'bursts': k, 'events': events, 'trace': tr,
            'binding_constraint': binding}
