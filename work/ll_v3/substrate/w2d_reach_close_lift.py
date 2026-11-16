def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            + (a[2] - b[2]) ** 2) ** 0.5


def run(api, standoff=0.15, lift_dz=0.10, grasp_dy=0.07, grasp_z=0.94,
        lateral_offset=0.0, tuck=True, **kw):
    """W2d: 05 reach -> close -> lift with a LIVE object target and a
    phase-resolved displacement trace.  The grasp target and the grasp
    orientation are re-derived from the live object every baselock step; the
    pre-close object displacement must be <= 0.010 m.  Gate unchanged."""
    st0 = api.observe()
    ob0 = st0.get('object') or {}
    obj0 = ob0.get('pos')
    object_match = ob0.get('name') or 'bottle'
    tuck_r = None
    if tuck:
        tuck_r = api.skill('arm_tuck')
        if not tuck_r.get('ok'):
            return {'ok': False, 'stage': 'tuck',
                    'binding_constraint': tuck_r.get('binding_constraint'),
                    'tuck': tuck_r, 'phase': {'initial': obj0}}
    lateral = obj0[1] + lateral_offset
    r1 = api.skill('base_standoff_servo_v3', standoff=standoff,
                   lateral_target=lateral, object_match=object_match)
    if not r1.get('ok'):
        return {'ok': False, 'stage': 'standoff',
                'binding_constraint': r1.get('binding_constraint'),
                'standoff': r1,
                'phase': {'initial': obj0,
                          'after_standoff': (api.observe().get(
                              'object') or {}).get('pos')}}
    st = api.observe()
    o1 = (st.get('object') or {}).get('pos')
    d_standoff = _dist(o1, obj0)
    grasp = {'dx': -0.0838, 'dy': grasp_dy, 'z': grasp_z}
    r2 = api.skill('arm_pose_ik_live', grasp=grasp, orientation='keep')
    st = api.observe()
    o2 = (st.get('object') or {}).get('pos')
    d_reach = _dist(o2, o1)
    pre_close = _dist(o2, obj0)
    if not r2.get('ok'):
        return {'ok': False, 'stage': 'reach',
                'binding_constraint': r2.get('binding_constraint'),
                'reach': r2,
                'phase': {'initial': obj0, 'after_standoff': o1,
                          'after_reach': o2,
                          'displacements': {'standoff': d_standoff,
                                            'reach': d_reach,
                                            'pre_close': pre_close},
                          'reach_contacts': api.contact_detail(
                              limit=40, match=object_match)},
                'budget': {'base_drift': r2.get('base_drift'),
                           'solver_residual': r2.get('solver_residual'),
                           'target_frame_offset': r2.get(
                               'target_frame_offset'),
                           'wrist_margin': r2.get('wrist_margin'),
                           'pad_error': r2.get('pad_error'),
                           'normal_error_deg': r2.get(
                               'normal_error_deg')}}
    r3 = api.skill('hand_close_contact')
    st = api.observe()
    o3 = (st.get('object') or {}).get('pos')
    d_close = _dist(o3, o2)
    if not r3.get('ok'):
        return {'ok': False, 'stage': 'close',
                'binding_constraint': r3.get('binding_constraint'),
                'close': r3,
                'phase': {'initial': obj0, 'after_standoff': o1,
                          'after_reach': o2, 'after_close': o3,
                          'displacements': {'standoff': d_standoff,
                                            'reach': d_reach,
                                            'close': d_close,
                                            'pre_close': pre_close}},
                'reach': r2}
    closed = r3.get('closed_upper')
    grasp_l = {'dx': -0.0838, 'dy': grasp_dy, 'z': grasp_z + lift_dz}
    r4 = api.skill('arm_pose_ik_live', grasp=grasp_l, orientation='keep',
                   hand_hold=closed)
    st = api.observe()
    o4 = (st.get('object') or {}).get('pos')
    d_lift = _dist(o4, o3)
    rise = o4[2] - o3[2]
    contacts = api.hand_contacts_side()
    bil = True if (contacts.get('left', 0) > 0 and contacts.get('right', 0) > 0) else False
    sg = api.skill('safety_guard')
    pe = r2.get('pad_error') or {}
    worst = None
    for side in pe:
        if worst is None or pe[side] > worst:
            worst = pe[side]
    guard_ok = True if sg.get('ok') else False
    ok = True if (pre_close <= 0.010 and worst is not None and worst <= 0.015
                   and bil and rise >= 0.05 and guard_ok) else False
    binding = None
    if not ok:
        if pre_close > 0.010:
            binding = 'pre_close_displacement'
        elif worst is None or worst > 0.015:
            binding = 'pad_error'
        elif not bil:
            binding = 'bilateral_contact'
        elif rise < 0.05:
            binding = 'rise'
        elif not guard_ok:
            binding = 'guard'
    budget = {'object_displacement_pre_close': pre_close,
              'base_drift': r2.get('base_drift'),
              'solver_residual': r2.get('solver_residual'),
              'target_frame_offset': r2.get('target_frame_offset'),
              'wrist_margin': r2.get('wrist_margin'),
              'pad_error': pe,
              'normal_error_deg': r2.get('normal_error_deg')}
    return {'ok': ok, 'stage': 'lift', 'pad_error': pe,
            'worst_pad_error': worst,
            'normal_error_deg': r2.get('normal_error_deg'),
            'contacts': contacts, 'object_rise': rise, 'guard': sg,
            'phase': {'initial': obj0, 'after_standoff': o1,
                      'after_reach': o2, 'after_close': o3,
                      'after_lift': o4,
                      'displacements': {'standoff': d_standoff,
                                        'reach': d_reach, 'close': d_close,
                                        'lift': d_lift,
                                        'pre_close': pre_close},
                      'pre_close_contacts': api.contact_detail(
                          limit=40, match=object_match),
                      'after_lift_contacts': api.contact_detail(
                          limit=40, match=object_match)},
            'standoff_trace': r1.get('trace'),
            'standoff': r1, 'reach': r2, 'close': r3, 'lift_ik': r4,
            'budget': budget, 'binding_constraint': binding}
