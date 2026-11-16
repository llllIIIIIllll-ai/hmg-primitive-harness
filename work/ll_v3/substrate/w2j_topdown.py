def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            + (a[2] - b[2]) ** 2) ** 0.5


def run(api, standoff=0.15, lateral_offset=0.0, z_pre=1.20, z_grasp=1.06,
        dy=0.14, lift_dz=0.10, pre_dx=-0.19, grasp_dx=-0.02, **kw):
    """W2j: top-down grasp.  tuck -> standoff -> backoff -> free-space
    unfold to a palms-down pre-grasp -> approach -> vertical descent ->
    close -> lift.  W2 criteria unchanged."""
    st0 = api.observe()
    ob0 = st0.get('object') or {}
    obj0 = ob0.get('pos')
    object_match = ob0.get('name') or 'bottle'
    tuck_r = api.skill('arm_tuck')
    if not tuck_r.get('ok'):
        return {'ok': False, 'stage': 'tuck',
                'binding_constraint': tuck_r.get('binding_constraint'),
                'phase': {'initial': obj0}}
    st = api.observe()
    o_t = (st.get('object') or {}).get('pos')
    d_tuck = _dist(o_t, obj0)
    lateral = obj0[1] + lateral_offset
    r1 = api.skill('base_standoff_servo_v3', standoff=standoff,
                   lateral_target=lateral, object_match=object_match)
    if not r1.get('ok'):
        return {'ok': False, 'stage': 'standoff',
                'binding_constraint': r1.get('binding_constraint'),
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'displacements': {'tuck': d_tuck}}}
    st = api.observe()
    o1 = (st.get('object') or {}).get('pos')
    d_standoff = _dist(o1, o_t)
    mv = api.move_base(x=-0.12, y=0.0, yaw=0.0, dur=4.5)
    st = api.observe()
    o_b = (st.get('object') or {}).get('pos')
    d_back = _dist(o_b, o1)
    if d_back > 0.001:
        return {'ok': False, 'stage': 'backoff',
                'binding_constraint': 'object_displacement',
                'backoff': mv,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_b,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back}}}
    u_r = api.skill('arm_untuck', dx=pre_dx, dy=dy, z=z_pre,
                    stop_err=0.12, max_steps=250, max_delta=0.04,
                    base_gain=0.3, base_deadband=0.005, base_vmax=0.05,
                    orient_mode='down', orientation='down',
                    deadband_comp=0.012, trace_contacts=True,
                    contact_match=object_match, joint_mode=True,
                    wrist_guard=True, wrist_guard_dist=0.01,
                    seed_mode='dls')
    st = api.observe()
    o_u = (st.get('object') or {}).get('pos')
    d_untuck = _dist(o_u, o_b)
    if u_r.get('resets') or u_r.get('nans'):
        return {'ok': False, 'stage': 'unfold',
                'binding_constraint': u_r.get('binding_constraint'),
                'untuck': u_r,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_b,
                          'after_untuck': o_u,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck}}}
    if d_untuck > 0.001:
        return {'ok': False, 'stage': 'unfold',
                'binding_constraint': 'object_displacement',
                'untuck': u_r,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_b,
                          'after_untuck': o_u,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck}}}
    r1b = api.skill('base_standoff_servo_v3', standoff=standoff,
                    lateral_target=lateral, object_match=object_match)
    st = api.observe()
    o_u2 = (st.get('object') or {}).get('pos')
    d_rest = _dist(o_u2, o_u)
    if not r1b.get('ok'):
        return {'ok': False, 'stage': 'standoff_after_untuck',
                'binding_constraint': r1b.get('binding_constraint'),
                'standoff_after_untuck': r1b, 'untuck': u_r,
                'backoff': mv,
                'phase': {'initial': obj0, 'after_standoff': o1,
                          'after_backoff': o_b, 'after_untuck': o_u,
                          'after_rest_standoff': o_u2,
                          'displacements': {'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest}}}
    if d_rest > 0.001:
        return {'ok': False, 'stage': 'approach',
                'binding_constraint': 'object_displacement',
                'standoff_after_untuck': r1b, 'untuck': u_r,
                'phase': {'initial': obj0, 'after_standoff': o1,
                          'after_backoff': o_b, 'after_untuck': o_u,
                          'after_rest_standoff': o_u2,
                          'displacements': {'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest}}}
    grasp = {'dx': grasp_dx, 'dy': dy, 'z': z_grasp}
    r2 = api.skill('arm_pose_ik_live', grasp=grasp, orientation='down',
                   orient_mode='down', max_delta=0.01, max_steps=400,
                   max_delta_near=0.005, freeze_frame=True,
                   reorient_each_step=True, base_deadband=0.005,
                   base_yaw_deadband=0.003, base_yaw_gain=0.6,
                   base_wmax=0.1, wrist_guard=True, wrist_guard_dist=0.01,
                   seed_mode='dls', seed_force=True, deadband_comp=0.012,
                   trace_contacts=True, contact_match=object_match)
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
                'reach': r2, 'untuck': u_r, 'unfold': u_r,
                'backoff': mv, 'standoff_after_untuck': r1b,
                'phase': {'initial': obj0, 'after_standoff': o1,
                          'after_backoff': o_b, 'after_untuck': o_u,
                          'after_rest_standoff': o_u2, 'after_reach': o2,
                          'displacements': {'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reach': d_reach}}}
    if d_reach > 0.001:
        return {'ok': False, 'stage': 'descent',
                'binding_constraint': 'object_displacement',
                'reach': r2, 'untuck': u_r, 'backoff': mv,
                'standoff_after_untuck': r1b,
                'phase': {'initial': obj0, 'after_standoff': o1,
                          'after_backoff': o_b, 'after_untuck': o_u,
                          'after_rest_standoff': o_u2, 'after_reach': o2,
                          'displacements': {'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reach': d_reach}}}
    r3 = api.skill('hand_close_contact')
    st = api.observe()
    o3 = (st.get('object') or {}).get('pos')
    d_close = _dist(o3, o2)
    close_contacts = api.contact_detail(limit=60, match=object_match)
    if not r3.get('ok'):
        return {'ok': False, 'stage': 'close',
                'binding_constraint': r3.get('binding_constraint'),
                'close': r3, 'reach': r2, 'untuck': u_r,
                'close_contacts': close_contacts,
                'phase': {'initial': obj0, 'after_standoff': o1,
                          'after_backoff': o_b, 'after_untuck': o_u,
                          'after_rest_standoff': o_u2, 'after_reach': o2,
                          'after_close': o3,
                          'displacements': {'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reach': d_reach,
                                            'close': d_close}}}
    closed = r3.get('closed_upper')
    grasp_l = {'dx': grasp_dx, 'dy': dy, 'z': z_grasp + lift_dz}
    r4 = api.skill('arm_pose_ik_live', grasp=grasp_l, orientation='down',
                   orient_mode='down', hand_hold=closed, max_delta=0.005,
                   max_delta_near=0.005, freeze_frame=True,
                   reorient_each_step=True, base_deadband=0.005,
                   freeze_object_pos=True)
    st = api.observe()
    o4 = (st.get('object') or {}).get('pos')
    d_lift = _dist(o4, o3)
    rise = o4[2] - o3[2]
    contacts = api.hand_contacts_side()
    bil = True if (contacts.get('left', 0) > 0
                   and contacts.get('right', 0) > 0) else False
    sg = api.skill('safety_guard')
    guard_ok = True if sg.get('ok') else False
    pe = r2.get('pad_error') or {}
    worst = None
    for side in pe:
        if worst is None or pe[side] > worst:
            worst = pe[side]
    ok = True if (pre_close <= 0.010 and worst is not None
                  and worst <= 0.015 and bil and rise >= 0.05
                  and guard_ok and r2.get('resets') == 0
                  and r2.get('nans') == 0) else False
    binding = None
    if pre_close > 0.010:
        binding = 'pre_close_displacement'
    elif r2.get('resets') > 0:
        binding = 'sim_reset'
    elif r2.get('nans') > 0:
        binding = 'nan_target'
    elif worst is None or worst > 0.015:
        binding = 'pad_error'
    elif not bil:
        binding = 'bilateral_contact'
    elif rise < 0.05:
        binding = 'rise'
    elif not guard_ok:
        binding = 'guard'
    phase = {'initial': obj0, 'after_tuck': o_t, 'after_standoff': o1,
             'after_backoff': o_b, 'after_untuck': o_u,
             'after_rest_standoff': o_u2, 'after_reach': o2,
             'after_close': o3, 'after_lift': o4,
             'pad_aabb_dist_min': r1b.get('pad_aabb_dist_min'),
             'wrist_aabb_dist_min': r1b.get('wrist_aabb_dist_min'),
             'displacements': {'tuck': d_tuck, 'standoff': d_standoff,
                               'backoff': d_back, 'untuck': d_untuck,
                               'rest_standoff': d_rest, 'reach': d_reach,
                               'close': d_close, 'lift': d_lift,
                               'pre_close': pre_close},
             'reach_trace': r2.get('trace'),
             'reach_sim_time': r2.get('sim_time'),
             'first_step': r2.get('first_step')}
    budget = {'object_displacement_pre_close': pre_close,
              'base_drift': r2.get('base_drift'),
              'solver_residual': r2.get('solver_residual'),
              'wrist_margin': r2.get('wrist_margin'),
              'joint_margin_min': r2.get('joint_margin_min'),
              'joint_margin_joint': r2.get('joint_margin_joint'),
              'joint_margins': r2.get('joint_margins'),
              'pad_error': pe,
              'normal_error_deg': r2.get('normal_error_deg'),
              'resets': r2.get('resets'), 'nans': r2.get('nans'),
              'wrist_dist_min': r2.get('wrist_dist_min')}
    return {'ok': ok, 'stage': 'lift', 'pad_error': pe,
            'worst_pad_error': worst,
            'normal_error_deg': r2.get('normal_error_deg'),
            'contacts': contacts, 'close_contacts': close_contacts,
            'object_rise': rise, 'guard': sg,
            'phase': phase, 'budget': budget,
            'standoff': r1, 'backoff': mv, 'untuck': u_r,
            'rest_standoff': r1b, 'reach': r2, 'close': r3,
            'lift_ik': r4, 'binding_constraint': binding}
