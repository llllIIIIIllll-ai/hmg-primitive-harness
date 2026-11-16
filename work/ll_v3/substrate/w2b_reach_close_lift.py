def run(api, standoff=0.15, lift_dz=0.10, grasp_dx=-0.0838,
        grasp_dy=0.07, grasp_z=0.94, **kw):
    """W2b integration benchmark: 05 reach -> close -> lift.

    (a) per-control-step QP re-solve against the live base; (b) the standoff
    lateral pose follows the task-target palm midpoint.  The pass criteria are
    unchanged: pad error <= 0.015 m/side, bilateral contact after the lift,
    rise >= 0.05 m, no guard trip.
    """
    st = api.observe()
    ob = st.get('object') or {}
    obj = ob.get('pos')
    lateral = obj[1]
    r1 = api.skill('base_standoff_servo_v2', standoff=standoff,
                   lateral_target=lateral)
    if not r1.get('ok'):
        return {'ok': False, 'stage': 'standoff',
                'binding_constraint': r1.get('binding_constraint'),
                'standoff': r1}
    grasp = {'left': [obj[0] + grasp_dx, obj[1] + grasp_dy, grasp_z],
             'right': [obj[0] + grasp_dx, obj[1] - grasp_dy, grasp_z]}
    r2 = api.skill('arm_pose_ik_baselock', targets=grasp, orientation='keep')
    if not r2.get('ok'):
        return {'ok': False, 'stage': 'ik',
                'binding_constraint': r2.get('binding_constraint'),
                'ik': r2,
                'budget': {'base_drift': r2.get('base_drift'),
                           'solver_residual': r2.get('solver_residual'),
                           'target_frame_offset': r2.get(
                               'target_frame_offset'),
                           'wrist_margin': r2.get('wrist_margin'),
                           'pad_error': r2.get('pad_error')}}
    r3 = api.skill('hand_close_contact')
    if not r3.get('ok'):
        return {'ok': False, 'stage': 'close',
                'binding_constraint': r3.get('binding_constraint'),
                'close': r3}
    closed = r3.get('closed_upper')
    st0 = api.observe()
    obj0 = (st0.get('object') or {}).get('pos')
    tgt2 = {}
    for side in grasp:
        tgt2[side] = [grasp[side][0], grasp[side][1],
                      grasp[side][2] + lift_dz]
    r4 = api.skill('arm_pose_ik_baselock', targets=tgt2,
                   orientation='keep', hand_hold=closed)
    st1 = api.observe()
    obj1 = (st1.get('object') or {}).get('pos')
    rise = obj1[2] - obj0[2]
    contacts = api.hand_contacts_side()
    bil = (contacts.get('left', 0) > 0 and contacts.get('right', 0) > 0)
    g = api.skill('safety_guard')
    pe = r2.get('pad_error') or {}
    worst = None
    for side in pe:
        if worst is None or pe[side] > worst:
            worst = pe[side]
    guard_ok = g.get('ok')
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
    budget = {'base_drift': r2.get('base_drift'),
              'solver_residual': r2.get('solver_residual'),
              'target_frame_offset': r2.get('target_frame_offset'),
              'wrist_margin': r2.get('wrist_margin'),
              'pad_error': pe,
              'lift_base_drift': r4.get('base_drift')}
    return {'ok': ok, 'stage': 'lift', 'pad_error': pe,
            'worst_pad_error': worst, 'contacts': contacts,
            'object_rise': rise, 'guard': g, 'budget': budget,
            'standoff': r1, 'ik': r2, 'close': r3, 'lift': r4,
            'binding_constraint': binding}
