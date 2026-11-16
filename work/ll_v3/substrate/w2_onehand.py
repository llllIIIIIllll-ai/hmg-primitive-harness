def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            + (a[2] - b[2]) ** 2) ** 0.5


def run(api, side='left', standoff=0.15, lateral_offset=0.0,
        pre_dx=-0.205, grasp_dx=-0.105, dy=0.0, z_grasp=0.954, z_pre=0.954,
        lift_dz=0.10, **kw):
    """W2 one-hand grasp: tuck -> standoff -> backoff -> free-space unfold to
    a palms-down pre-grasp for the GRASPING hand -> approach -> seeded
    descent -> close -> lift.  The non-grasping arm stays tucked clear."""
    st0 = api.observe()
    ob0 = st0.get('object') or {}
    obj0 = ob0.get('pos')
    om = ob0.get('name') or 'bottle'
    tuck = api.skill('arm_tuck')
    if not tuck.get('ok'):
        return {'ok': False, 'stage': 'tuck',
                'binding_constraint': tuck.get('binding_constraint')}
    st = api.observe()
    o_t = (st.get('object') or {}).get('pos')
    d_tuck = _dist(o_t, obj0)
    lateral = obj0[1] + lateral_offset
    r1 = api.skill('base_standoff_servo_v3', standoff=standoff,
                   lateral_target=lateral, object_match=om)
    if not r1.get('ok'):
        return {'ok': False, 'stage': 'standoff',
                'binding_constraint': r1.get('binding_constraint'),
                'phase': {'initial': obj0, 'after_tuck': o_t}}
    st = api.observe()
    o1 = (st.get('object') or {}).get('pos')
    d_standoff = _dist(o1, o_t)
    u1 = api.skill('arm_untuck', dx=-0.126, dy=0.07, z=1.16,
                   stop_err=0.12, max_steps=250, max_delta=0.04,
                   orient_mode='down', orientation='down', joint_mode=True,
                   deadband_comp=0.012, wrist_guard=True,
                   wrist_guard_dist=0.01, seed_mode='dls', sides=(side,),
                   base_gain=0.3, base_deadband=0.005, base_vmax=0.05,
                   base_yaw_deadband=0.003, base_yaw_gain=0.6,
                   base_wmax=0.1, trace_contacts=True, contact_match=om)
    st = api.observe()
    o_u1 = (st.get('object') or {}).get('pos')
    d_u1 = _dist(o_u1, o1)
    if d_u1 > 0.001:
        return {'ok': False, 'stage': 'unfold_down',
                'binding_constraint': 'object_displacement', 'untuck': u1,
                'phase': {'displacements': {'standoff': d_standoff,
                                            'untuck': d_u1}}}
    u2 = api.skill('arm_pose_ik_live', grasp={'dx': pre_dx, 'dy': dy,
                                              'z': z_pre},
                   orientation='keep', orient_mode='side',
                   sides=(side,), joint_mode=True, stop_err=0.12,
                   max_steps=300, max_delta=0.04, max_delta_near=0.02,
                   near_dist=0.03, deadband_comp=0.04,
                   wrist_guard=True, wrist_guard_dist=0.01, seed_mode='dls',
                   base_gain=0.3, base_deadband=0.005, base_vmax=0.05,
                   base_yaw_deadband=0.003, base_yaw_gain=0.6,
                   base_wmax=0.1, trace_contacts=True, contact_match=om)
    st = api.observe()
    o_u = (st.get('object') or {}).get('pos')
    d_untuck = _dist(o_u, o_u1)
    if d_untuck > 0.001:
        return {'ok': False, 'stage': 'unfold_front',
                'binding_constraint': 'object_displacement',
                'untuck': u2, 'unfold_down': u1,
                'phase': {'displacements': {'standoff': d_standoff,
                                            'untuck_down': d_u1,
                                            'untuck': d_untuck}}}
    if not u2.get('ok'):
        return {'ok': False, 'stage': 'unfold_front_unreached',
                'binding_constraint': u2.get('binding_constraint'),
                'untuck': u2, 'unfold_down': u1,
                'phase': {'displacements': {'standoff': d_standoff,
                                            'untuck_down': d_u1,
                                            'untuck': d_untuck}}}
    u_r = u2
    r1b = None
    o_u2 = o_u
    d_rest = 0.0
    lat_ok = True
    wp = {'dx': -0.15, 'dy': 0.0, 'z': 1.004}
    r_wp = api.skill('arm_pose_ik_live', grasp=wp, orientation='keep',
                     orient_mode='side', sides=(side,), joint_mode=True,
                     max_delta=0.04, max_steps=300, stop_err=0.05,
                     max_delta_near=0.02, near_dist=0.03,
                     deadband_comp=0.035, wrist_guard=True,
                     wrist_guard_dist=0.01, seed_mode='dls',
                     base_gain=0.3, base_deadband=0.005, base_vmax=0.05,
                     base_yaw_deadband=0.003, base_yaw_gain=0.6,
                     base_wmax=0.1, trace_contacts=True, contact_match=om)
    st = api.observe()
    o_wp = (st.get('object') or {}).get('pos')
    d_wp = _dist(o_wp, o_u2)
    if r_wp.get('wrist_abort'):
        return {'ok': False, 'stage': 'waypoint_wrist_guard',
                'binding_constraint': 'wrist_guard',
                'wrist_abort_step': r_wp.get('wrist_abort_step'),
                'wrist_dist_min': r_wp.get('wrist_dist_min'),
                'waypoint': r_wp, 'untuck': u_r,
                'phase': {'displacements': {'standoff': d_standoff,
                                            'untuck': d_untuck,
                                            'waypoint': d_wp}}}
    if d_wp > 0.001:
        return {'ok': False, 'stage': 'waypoint',
                'binding_constraint': 'object_displacement',
                'waypoint': r_wp, 'untuck': u_r,
                'phase': {'displacements': {'standoff': d_standoff,
                                            'untuck': d_untuck,
                                            'waypoint': d_wp}}}
    grasp = {'dx': grasp_dx, 'dy': dy, 'z': z_grasp}
    r2 = api.skill('arm_pose_ik_live', grasp=grasp, orientation='keep',
                   orient_mode='side', sides=(side,), max_delta=0.02,
                   max_steps=400, max_delta_near=0.01, freeze_frame=True,
                   reorient_each_step=True, base_deadband=0.005,
                   base_yaw_deadband=0.003, base_yaw_gain=0.6,
                   base_wmax=0.1, wrist_guard=True, wrist_guard_dist=0.01,
                   seed_mode='dls', seed_force=True, deadband_comp=0.035,
                   base_gain=0.3, base_vmax=0.05,
                   trace_contacts=True, contact_match=om)
    st = api.observe()
    o2 = (st.get('object') or {}).get('pos')
    d_reach = _dist(o2, o_u2)
    pre_close = _dist(o2, obj0)
    if r2.get('wrist_abort'):
        return {'ok': False, 'stage': 'descent_wrist_guard',
                'binding_constraint': 'wrist_guard',
                'wrist_abort_step': r2.get('wrist_abort_step'),
                'wrist_dist_min': r2.get('wrist_dist_min'),
                'wrist_dist_by_side': r2.get('wrist_dist_by_side'),
                'waypoint': r_wp, 'reach': r2, 'untuck': u_r,
                'phase': {'displacements': {'standoff': d_standoff,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reach': d_reach}}}
    if d_reach > 0.001:
        return {'ok': False, 'stage': 'descent',
                'binding_constraint': 'object_displacement',
                'waypoint': r_wp, 'reach': r2, 'untuck': u_r,
                'phase': {'displacements': {'standoff': d_standoff,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reach': d_reach}}}
    r3 = api.skill('hand_close_contact')
    st = api.observe()
    o3 = (st.get('object') or {}).get('pos')
    d_close = _dist(o3, o2)
    close_contacts = api.contact_detail(limit=60, match=om)
    closed = r3.get('closed_upper')
    grasp_l = {'dx': grasp_dx, 'dy': dy, 'z': z_grasp + lift_dz}
    r4 = api.skill('arm_pose_ik_live', grasp=grasp_l, orientation='keep',
                   orient_mode='side', sides=(side,), hand_hold=closed,
                   max_delta=0.005, max_delta_near=0.005, freeze_frame=True,
                   reorient_each_step=True, base_deadband=0.005,
                   freeze_object_pos=True)
    st = api.observe()
    o4 = (st.get('object') or {}).get('pos')
    d_lift = _dist(o4, o3)
    rise = o4[2] - o3[2]
    contacts = api.hand_contacts_side()
    side_contacts = contacts.get(side, 0)
    sg = api.skill('safety_guard')
    guard_ok = True if sg.get('ok') else False
    pe = r2.get('pad_error') or {}
    site = pe.get(side)
    ok = True if (pre_close <= 0.010 and site is not None
                  and site <= 0.015 and side_contacts > 0
                  and rise >= 0.05 and guard_ok
                  and r2.get('resets') == 0 and r2.get('nans') == 0) else False
    binding = None
    if pre_close > 0.010:
        binding = 'pre_close_displacement'
    elif r2.get('resets') > 0:
        binding = 'sim_reset'
    elif r2.get('nans') > 0:
        binding = 'nan_target'
    elif site is None or site > 0.015:
        binding = 'site'
    elif side_contacts <= 0:
        binding = 'grasp_contact'
    elif rise < 0.05:
        binding = 'rise'
    elif not guard_ok:
        binding = 'guard'
    phase = {'initial': obj0, 'after_tuck': o_t, 'after_standoff': o1,
             'after_untuck': o_u,
             'after_rest_standoff': o_u2, 'after_reach': o2,
             'after_close': o3, 'after_lift': o4,
             'displacements': {'tuck': d_tuck, 'standoff': d_standoff,
                               'untuck': d_untuck,
                               'rest_standoff': d_rest, 'reach': d_reach,
                               'close': d_close, 'lift': d_lift,
                               'pre_close': pre_close},
             'reach_trace': r2.get('trace'),
             'first_step': r2.get('first_step')}
    budget = {'object_displacement_pre_close': pre_close,
              'base_drift': r2.get('base_drift'),
              'solver_residual': r2.get('solver_residual'),
              'wrist_margin': r2.get('wrist_margin'),
              'joint_margin_min': r2.get('joint_margin_min'),
              'joint_margin_joint': r2.get('joint_margin_joint'),
              'pad_error': pe, 'resets': r2.get('resets'),
              'nans': r2.get('nans'),
              'wrist_dist_min': r2.get('wrist_dist_min')}
    return {'ok': ok, 'stage': 'lift', 'side': side, 'pad_error': pe,
            'site': site, 'contacts': contacts, 'close_contacts':
            close_contacts, 'object_rise': rise, 'guard': sg,
            'phase': phase, 'budget': budget,
            'standoff': r1, 'untuck': u_r,
            'rest_standoff': r1b, 'lateral_accepted': lat_ok,
            'waypoint': r_wp, 'reach': r2, 'close': r3,
            'lift_ik': r4, 'binding_constraint': binding}
