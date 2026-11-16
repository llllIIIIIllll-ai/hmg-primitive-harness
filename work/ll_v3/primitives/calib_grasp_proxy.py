def run(api, standoff=0.15, grasp_dx=-0.0838, grasp_dy=0.07,
        grasp_z=0.94, lift_dz=0.15, max_bursts=50, burst=0.5,
        speed=0.20, **kw):
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
        vx = np.clip(0.5 * (da - standoff), 0.05, speed)
        api.move_base(x=vx, y=vy, yaw=wz, dur=burst)
        k = k + 1
    st = api.observe()
    ob = st.get('object') or {}
    obj0 = ob.get('pos')
    tgt = {'left': [obj0[0] + grasp_dx, obj0[1] + grasp_dy, grasp_z],
           'right': [obj0[0] + grasp_dx, obj0[1] - grasp_dy, grasp_z]}
    r1 = api.reach_box_qp(task_dim=12, targets=tgt, orientation='keep',
                          dur=1.0, iters=80)
    c1 = api.close_grip(angle=0.6, dur=1.0)
    closed = api.commanded()
    st1 = api.observe()
    ob1 = (st1.get('object') or {})
    obj1 = ob1.get('pos')
    hs = api.hand_contacts_side()
    tgt2 = {'left': [tgt['left'][0], tgt['left'][1],
                     tgt['left'][2] + lift_dz],
            'right': [tgt['right'][0], tgt['right'][1],
                      tgt['right'][2] + lift_dz]}
    r2 = api.reach_box_qp(task_dim=12, targets=tgt2, orientation='keep',
                          hand_hold=closed, dur=1.0, iters=80)
    st2 = api.observe()
    ob2 = (st2.get('object') or {})
    obj2 = ob2.get('pos')
    hs2 = api.hand_contacts_side()
    q = api.joints()
    fing = {}
    for nm in q:
        if '_hand_' in nm:
            fing[nm] = q[nm]
    rise = obj2[2] - obj0[2]
    close_bil = (hs.get('left', 0) > 0 and hs.get('right', 0) > 0)
    lift_bil = (hs2.get('left', 0) > 0 and hs2.get('right', 0) > 0)
    proxy = (lift_bil and rise > 0.02)
    return {'standoff_aabb': da,
            'object_before': obj0, 'object_after': obj2,
            'object_rise': rise,
            'contacts_before': hs, 'contacts_after': hs2,
            'close_phase_bilateral': close_bil,
            'lift_phase_bilateral': lift_bil,
            'fingers': fing, 'proxy_grasped': proxy,
            'reach_grasp': {'palm_residual': r1.get('palm_residual'),
                            'joint_margin_min': r1.get('joint_margin_min'),
                            'ik_err': r1.get('ik_err')},
            'reach_lift': {'palm_residual': r2.get('palm_residual'),
                           'joint_margin_min': r2.get('joint_margin_min'),
                           'ik_err': r2.get('ik_err')},
            'close_contacts': c1.get('contacts'),
            'closed_present': closed is not None}
