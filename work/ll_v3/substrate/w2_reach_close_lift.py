def run(api, standoff=0.15, lift_dz=0.10, grasp_dx=-0.0838,
        grasp_dy=0.07, grasp_z=0.94, **kw):
    """W2 integration benchmark: 05 reach -> close -> lift.

    Gate: pad error <= 0.015 m/side, bilateral contact after the lift, object
    rise >= 0.05 m, no guard trip.  The grasp uses the scratch-screened drill
    target; the lift is a joint-space transport with the explicit
    closed-finger vector held, capped at 0.10 m for the joint margin (G10).
    """
    r1 = api.skill('base_standoff_servo', standoff=standoff)
    if not r1.get('ok'):
        return {'ok': False, 'stage': 'standoff',
                'binding_constraint': r1.get('binding_constraint'),
                'standoff': r1}
    st = api.observe()
    ob = st.get('object') or {}
    obj = ob.get('pos')
    grasp = {'left': [obj[0] + grasp_dx, obj[1] + grasp_dy, grasp_z],
             'right': [obj[0] + grasp_dx, obj[1] - grasp_dy, grasp_z]}
    r2 = api.skill('arm_pose_ik', targets=grasp, orientation='keep')
    if not r2.get('ok'):
        return {'ok': False, 'stage': 'ik',
                'binding_constraint': r2.get('binding_constraint'),
                'ik': r2}
    r3 = api.skill('hand_close_contact')
    if not r3.get('ok'):
        return {'ok': False, 'stage': 'close',
                'binding_constraint': r3.get('binding_constraint'),
                'close': r3}
    closed = r3.get('closed_upper')
    st0 = api.observe()
    obj0 = (st0.get('object') or {}).get('pos')
    tgt = r2.get('target') or {}
    tgt2 = {}
    for side in tgt:
        tgt2[side] = [tgt[side][0], tgt[side][1], tgt[side][2] + lift_dz]
    r4 = api.reach_box_qp(task_dim=12, targets=tgt2, orientation='keep',
                          hand_hold=closed, dur=0.35, iters=80)
    api.step_goal(upper=r4.get('upper_target'), dur=0.2)
    st1 = api.observe()
    obj1 = (st1.get('object') or {}).get('pos')
    rise = 0.0
    if obj0 is not None and obj1 is not None:
        rise = obj1[2] - obj0[2]
    contacts = api.hand_contacts_side()
    pe = r2.get('pad_error') or {}
    worst = None
    for side in pe:
        if worst is None or pe[side] > worst:
            worst = pe[side]
    g = api.skill('safety_guard')
    guard_ok = g.get('ok')
    bil = (contacts.get('left', 0) > 0 and contacts.get('right', 0) > 0)
    ok = (worst is not None and worst <= 0.015 and bil
          and rise >= 0.05 and guard_ok)
    binding = None
    if not ok:
        if worst is None or worst > 0.015:
            binding = 'pad_error'
        elif not bil:
            binding = 'bilateral_contact'
        elif rise < 0.05:
            binding = 'rise'
        elif not guard_ok:
            binding = 'guard'
    return {'ok': ok, 'stage': 'lift', 'pad_error': pe,
            'worst_pad_error': worst, 'contacts': contacts,
            'object_rise': rise, 'guard': g,
            'standoff': r1, 'ik': r2, 'close': r3, 'lift': r4,
            'binding_constraint': binding}
