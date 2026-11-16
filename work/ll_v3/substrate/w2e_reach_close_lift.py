def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            + (a[2] - b[2]) ** 2) ** 0.5


def run(api, standoff=0.15, lift_dz=0.10, grasp_dy=0.07, grasp_z=0.94,
        lateral_offset=0.0, **kw):
    """W2e: tuck -> standoff -> untuck/pre-grasp -> true closed-loop live reach
    -> close -> lift.  Gate unchanged; pre-close displacement <= 0.010 m."""
    st0 = api.observe()
    ob0 = st0.get('object') or {}
    obj0 = ob0.get('pos')
    object_match = ob0.get('name') or 'bottle'
    tuck_r = api.skill('arm_tuck')
    if not tuck_r.get('ok'):
        return {'ok': False, 'stage': 'tuck',
                'binding_constraint': tuck_r.get('binding_constraint'),
                'tuck': tuck_r, 'phase': {'initial': obj0}}
    st = api.observe()
    o_t = (st.get('object') or {}).get('pos')
    d_tuck = _dist(o_t, obj0)
    lateral = obj0[1] + lateral_offset
    r1 = api.skill('base_standoff_servo_v3', standoff=standoff,
                   lateral_target=lateral, object_match=object_match)
    if not r1.get('ok'):
        return {'ok': False, 'stage': 'standoff',
                'binding_constraint': r1.get('binding_constraint'),
                'standoff': r1,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'displacements': {'tuck': d_tuck}}}
    st = api.observe()
    o1 = (st.get('object') or {}).get('pos')
    d_standoff = _dist(o1, o_t)
    # Back away with the arms still tucked: every unfold variant next to the
    # drill dragged it (0.247-0.544 m measured; the arm reconfiguration walks
    # the WBC base 0.4-0.5 m while the pads are beside the object).  Unfold in
    # free space, then re-approach with the arms in the pre-grasp.
    mv = api.move_base(x=-0.12, y=0.0, yaw=0.0, dur=4.5)
    st = api.observe()
    o_back = (st.get('object') or {}).get('pos')
    d_back = _dist(o_back, o1)
    if d_back > 0.001:
        return {'ok': False, 'stage': 'backoff',
                'binding_constraint': 'object_displacement',
                'backoff': mv,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back}}}
    # unfold UP and OUT in free space (drill AABB top 0.885): z=1.06 puts the
    # pad geoms ~0.07 below the site, i.e. >= 0.10 m above the AABB top, and
    # +/-0.20 lateral clears the volume.
    u_r = api.skill('arm_untuck', dx=-0.16, dy=0.22, z=0.94,
                    stop_err=0.02, max_steps=240, max_delta=0.015,
                    trace_contacts=True, contact_match=object_match)
    st = api.observe()
    o_u = (st.get('object') or {}).get('pos')
    d_untuck = _dist(o_u, o_back)
    upm = None
    upj = None
    for e in (u_r.get('trace') or []):
        m = e.get('margin')
        if m is None:
            continue
        if upm is None or m < upm:
            upm = m
            upj = e.get('margin_joint')
    unfold = {'ok': u_r.get('ok'),
              'path_margin_min': upm, 'path_margin_joint': upj,
              'worst_pad_error': u_r.get('worst_pad_error'),
              'pad_error': u_r.get('pad_error'),
              'steps': u_r.get('steps'),
              'resets': u_r.get('resets'), 'nans': u_r.get('nans'),
              'sim_time': u_r.get('sim_time'),
              'base_drift': u_r.get('base_drift'),
              'normal_error_deg': u_r.get('normal_error_deg'),
              'first_step': u_r.get('first_step'),
              'trace': u_r.get('trace'),
              'binding_constraint': u_r.get('binding_constraint')}
    if u_r.get('resets') or u_r.get('nans'):
        return {'ok': False, 'stage': 'untuck',
                'binding_constraint': u_r.get('binding_constraint'),
                'untuck': u_r,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1,
                          'untuck_resets': u_r.get('resets'),
                          'untuck_nans': u_r.get('nans'),
                          'untuck_sim_time': u_r.get('sim_time'),
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back}}}
    if d_untuck > 0.001:
        return {'ok': False, 'stage': 'unfold',
                'binding_constraint': 'object_displacement',
                'untuck': u_r, 'unfold': unfold,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck}}}
    # The unfold is an out-of-distribution arm motion for the WBC: measured
    # base walk ~0.14 m (backward + lateral) at max_delta 0.015.  The live
    # base->object axis rotates with it, so re-establish the standoff pose
    # with the proven servo (object displacement 0.001 m) before the reach.
    r1b = api.skill('base_standoff_servo_v3', standoff=standoff,
                    lateral_target=lateral, object_match=object_match)
    st = api.observe()
    o_u2 = (st.get('object') or {}).get('pos')
    d_rest = _dist(o_u2, o_u)
    if not r1b.get('ok'):
        return {'ok': False, 'stage': 'standoff_after_untuck',
                'binding_constraint': r1b.get('binding_constraint'),
                'standoff_after_untuck': r1b, 'untuck': u_r,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest}}}
    if d_rest > 0.001:
        return {'ok': False, 'stage': 'approach',
                'binding_constraint': 'object_displacement',
                'standoff_after_untuck': r1b, 'untuck': u_r,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest}}}
    clear_min = r1b.get('pad_clearance_min')
    dist_min = r1b.get('pad_aabb_dist_min')
    wrist_min = r1b.get('wrist_aabb_dist_min')
    wrist_side = r1b.get('wrist_aabb_dist')
    # The left wrist chain, not the pads, is what dragged the drill on the
    # descent (g27/g28_col at z 0.838).  Gate the descent on the wrist-chain
    # distance to the AABB, per side, mirrored.
    if wrist_min is None or wrist_min < 0.10:
        return {'ok': False, 'stage': 'pre_descent_wrist_clearance',
                'binding_constraint': 'wrist_clearance',
                'wrist_aabb_dist_min': wrist_min,
                'wrist_aabb_dist': wrist_side,
                'pad_aabb_dist_min': dist_min,
                'pad_clearance_min': clear_min,
                'backoff': mv, 'unfold': unfold,
                'standoff_after_untuck': r1b, 'untuck': u_r,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest}}}
    # The object is 0.226 m tall (AABB z 0.7995-1.0251), so "pads >= 0.10 m
    # above the top" is geometrically impossible at the grasp height (pads
    # ~1.00).  The safety quantity is the 3-D pad-to-AABB distance.
    if dist_min is not None and dist_min < 0.10:
        return {'ok': False, 'stage': 'approach',
                'binding_constraint': 'pad_aabb_distance',
                'pad_aabb_dist_min': dist_min,
                'pad_clearance_min': clear_min,
                'backoff': mv, 'unfold': unfold,
                'standoff_after_untuck': r1b, 'untuck': u_r,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest}}}
    # W2l: the approach walked the base forward ~0.18 m with the pads held,
    # carrying them past the pre-grasp.  Reposition to the object-relative
    # pre-grasp (the grid's clear branch) before the horizontal grasp move.
    pre = {'dx': -0.16, 'dy': 0.22, 'z': 0.94}
    r2a = api.skill('arm_pose_ik_live', grasp=pre, orientation='keep',
                    max_delta=0.015, max_steps=240, stop_err=0.05,
                    freeze_frame=True, reorient_each_step=True,
                    base_deadband=0.005,
                    base_yaw_deadband=0.003, base_yaw_gain=0.6,
                    base_wmax=0.1,
                    wrist_guard=True, wrist_guard_dist=0.01,
                    seed_mode='dls', joint_mode=True,
                    trace_contacts=True, contact_match=object_match)
    st = api.observe()
    o_p = (st.get('object') or {}).get('pos')
    d_pre = _dist(o_p, o_u2)
    if r2a.get('wrist_abort'):
        return {'ok': False, 'stage': 'reposition_wrist_guard',
                'binding_constraint': 'wrist_guard',
                'wrist_abort_step': r2a.get('wrist_abort_step'),
                'wrist_dist_min': r2a.get('wrist_dist_min'),
                'wrist_dist_by_side': r2a.get('wrist_dist_by_side'),
                'reposition': r2a, 'untuck': u_r, 'unfold': unfold,
                'backoff': mv, 'standoff_after_untuck': r1b,
                'pad_aabb_dist_min': dist_min,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2, 'after_reposition': o_p,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reposition': d_pre}}}
    if d_pre > 0.001:
        return {'ok': False, 'stage': 'reposition',
                'binding_constraint': 'object_displacement',
                'reposition': r2a, 'untuck': u_r, 'unfold': unfold,
                'backoff': mv, 'standoff_after_untuck': r1b,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2, 'after_reposition': o_p,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reposition': d_pre}}}
    grasp = {'dx': -0.0838, 'dy': grasp_dy, 'z': grasp_z}
    r2 = api.skill('arm_pose_ik_live', grasp=grasp, orientation='keep',
                   max_delta=0.015, max_steps=240,
                   freeze_frame=True, reorient_each_step=True,
                   base_deadband=0.005,
                   base_yaw_deadband=0.003, base_yaw_gain=0.6,
                   base_wmax=0.1,
                   wrist_guard=True, wrist_guard_dist=0.01,
                   seed_mode='dls',
                   trace_contacts=True, contact_match=object_match)
    st = api.observe()
    o2 = (st.get('object') or {}).get('pos')
    d_reach = _dist(o2, o_p)
    pre_close = _dist(o2, obj0)
    if r2.get('wrist_abort'):
        return {'ok': False, 'stage': 'descent_wrist_guard',
                'binding_constraint': 'wrist_guard',
                'wrist_abort_step': r2.get('wrist_abort_step'),
                'wrist_dist_min': r2.get('wrist_dist_min'),
                'wrist_dist_by_side': r2.get('wrist_dist_by_side'),
                'reach': r2, 'untuck': u_r, 'unfold': unfold,
                'backoff': mv, 'standoff_after_untuck': r1b,
                'pad_aabb_dist_min': dist_min,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2,
                          'after_reposition': o_p, 'after_reach': o2,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reposition': d_pre,
                                            'reach': d_reach}}}
    if d_reach > 0.001:
        return {'ok': False, 'stage': 'descent',
                'binding_constraint': 'object_displacement',
                'reach': r2, 'untuck': u_r, 'unfold': unfold,
                'backoff': mv, 'standoff_after_untuck': r1b,
                'pad_aabb_dist_min': dist_min,
                'phase': {'initial': obj0, 'after_tuck': o_t,
                          'after_standoff': o1, 'after_backoff': o_back,
                          'after_untuck': o_u,
                          'after_rest_standoff': o_u2, 'after_reach': o2,
                          'displacements': {'tuck': d_tuck,
                                            'standoff': d_standoff,
                                            'backoff': d_back,
                                            'untuck': d_untuck,
                                            'rest_standoff': d_rest,
                                            'reach': d_reach}}}
    phase = {'initial': obj0, 'after_tuck': o_t, 'after_standoff': o1,
             'after_backoff': o_back,
             'after_untuck': o_u, 'after_rest_standoff': o_u2,
             'after_reposition': o_p, 'after_reach': o2,
             'pad_clearance_min': clear_min,
             'pad_aabb_dist_min': dist_min,
             'wrist_aabb_dist_min': wrist_min,
             'wrist_dist_min': r2.get('wrist_dist_min'),
             'displacements': {'tuck': d_tuck, 'standoff': d_standoff,
                               'backoff': d_back, 'untuck': d_untuck,
                               'rest_standoff': d_rest, 'reach': d_reach,
                               'pre_close': pre_close},
             'reach_trace': r2.get('trace'),
             'reach_sim_time': r2.get('sim_time'),
             'resets': r2.get('resets'), 'nans': r2.get('nans'),
             'first_step': r2.get('first_step'),
             'untuck_resets': u_r.get('resets'),
             'untuck_nans': u_r.get('nans'),
             'untuck_sim_time': u_r.get('sim_time'),
             'untuck_steps': u_r.get('steps'),
             'untuck_worst_pad_error': u_r.get('worst_pad_error')}
    if not r2.get('ok'):
        return {'ok': False, 'stage': 'reach',
                'binding_constraint': r2.get('binding_constraint'),
                'reach': r2, 'phase': phase,
                'budget': {'object_displacement_pre_close': pre_close,
                           'base_drift': r2.get('base_drift'),
                           'solver_residual': r2.get('solver_residual'),
                           'target_frame_offset': r2.get(
                               'target_frame_offset'),
                           'wrist_margin': r2.get('wrist_margin'),
                           'pad_error': r2.get('pad_error'),
                           'normal_error_deg': r2.get(
                               'normal_error_deg'),
                           'resets': r2.get('resets'),
                           'nans': r2.get('nans')}}
    r3 = api.skill('hand_close_contact')
    st = api.observe()
    o3 = (st.get('object') or {}).get('pos')
    d_close = _dist(o3, o2)
    if not r3.get('ok'):
        return {'ok': False, 'stage': 'close',
                'binding_constraint': r3.get('binding_constraint'),
                'close': r3, 'phase': phase,
                'reach': r2}
    closed = r3.get('closed_upper')
    grasp_l = {'dx': -0.0838, 'dy': grasp_dy, 'z': grasp_z + lift_dz}
    r4 = api.skill('arm_pose_ik_live', grasp=grasp_l, orientation='keep',
                   hand_hold=closed, freeze_frame=True,
                   reorient_each_step=True, base_deadband=0.005,
                   max_delta=0.005, max_delta_near=0.005,
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
    phase['after_close'] = o3
    phase['after_lift'] = o4
    phase['displacements']['close'] = d_close
    phase['displacements']['lift'] = d_lift
    yd = None
    rtr = r2.get('trace') or []
    if rtr:
        yd = rtr[-1].get('yaw_after') - rtr[0].get('yaw_before')
    budget = {'object_displacement_pre_close': pre_close,
              'base_drift': r2.get('base_drift'),
              'yaw_drift_deg': yd,
              'wrist_aabb_dist_min': wrist_min,
              'wrist_dist_min': r2.get('wrist_dist_min'),
              'wrist_dist_by_side': r2.get('wrist_dist_by_side'),
              'solver_residual': r2.get('solver_residual'),
              'target_frame_offset': r2.get('target_frame_offset'),
              'wrist_margin': r2.get('wrist_margin'),
              'joint_margin_min': r2.get('joint_margin_min'),
              'joint_margin_joint': r2.get('joint_margin_joint'),
              'joint_margins': r2.get('joint_margins'),
              'pad_error': pe,
              'normal_error_deg': r2.get('normal_error_deg'),
              'resets': r2.get('resets'), 'nans': r2.get('nans'),
              'lift_base_drift': r4.get('base_drift'),
              'lift_resets': r4.get('resets'),
              'lift_nans': r4.get('nans')}
    return {'ok': ok, 'stage': 'lift', 'pad_error': pe,
            'worst_pad_error': worst,
            'normal_error_deg': r2.get('normal_error_deg'),
            'contacts': contacts, 'object_rise': rise, 'guard': sg,
            'phase': phase, 'budget': budget,
            'standoff': r1, 'backoff': mv, 'untuck': u_r,
            'unfold': unfold, 'rest_standoff': r1b,
            'pad_clearance_min': clear_min,
            'pad_aabb_dist_min': dist_min,
            'reposition': r2a, 'reach': r2, 'close': r3,
            'lift_ik': r4, 'binding_constraint': binding}
