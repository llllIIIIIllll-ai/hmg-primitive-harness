def run(api, standoff=0.15, lift_dz=0.10, grasp_dy=0.07, grasp_z=0.94,
        **kw):
    """W2c: 05 reach -> close -> lift with an explicit grasp orientation.

    The pad normals are derived from the live object/base frame (opposing
    normals toward the object; hand forward along the approach axis).  The
    reach uses the per-step baselock; if the site error is still > 0.015 m, a
    per-step target-correction retry is run.  Gate unchanged.
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
    b = api.base_state()
    dx = obj[0] - b['pos'][0]
    dy = obj[1] - b['pos'][1]
    d = (dx * dx + dy * dy) ** 0.5
    fx = dx / d
    fy = dy / d
    px = -fy
    py = fx
    f_des = [fx, fy, 0.0]
    n_left = [-px, -py, 0.0]
    n_right = [px, py, 0.0]
    grasp = {'left': [obj[0] - 0.0838 * fx + 0.07 * px,
                      obj[1] - 0.0838 * fy + 0.07 * py, grasp_z],
             'right': [obj[0] - 0.0838 * fx - 0.07 * px,
                       obj[1] - 0.0838 * fy - 0.07 * py, grasp_z]}
    g = api.geom()
    hg = api.hand_geoms()
    q = np.asarray(api.raw[1].qpos[3:7])
    Rb = api._quat_to_mat(q)
    r_body = {}
    for side in ('left', 'right'):
        site = g['sites'].get(side)
        if site is None:
            continue
        R_site = np.array(site['xmat'])
        pad_pos = None
        pad_n = None
        for body in hg:
            for gg in hg[body]['geoms']:
                if gg['name'] == 'gripper0_%s_g0_col' % side:
                    pad_pos = np.array(gg['world_pos'])
                    pad_n = np.array(gg['xmat'])[:, 0]
        if pad_pos is None:
            continue
        fwd_w = pad_pos - np.array(site['pos'])
        fwd_w = fwd_w / np.linalg.norm(fwd_w)
        e1 = R_site.T @ fwd_w
        e2 = R_site.T @ pad_n
        e2 = e2 - np.dot(e2, e1) * e1
        e2 = e2 / np.linalg.norm(e2)
        e3 = np.cross(e1, e2)
        B_local = np.column_stack([e1, e2, e3])
        f = np.array(f_des)
        f = f / np.linalg.norm(f)
        n = np.array(n_left if side == 'left' else n_right)
        n = n - np.dot(n, f) * f
        n = n / np.linalg.norm(n)
        c = np.cross(f, n)
        B_des = np.column_stack([f, n, c])
        R_des = B_des @ B_local.T
        U, S, Vt = np.linalg.svd(R_des)
        R_des = U @ Vt
        R_site0 = np.asarray(api.R_site0[side])
        R_body = Rb.T @ R_des @ R_site0.T
        U, S, Vt = np.linalg.svd(R_body)
        R_body = U @ Vt
        r_body[side] = R_body.tolist()
    normal_targets = {'left': n_left, 'right': n_right}
    r2 = api.skill('arm_pose_ik_baselock', targets=grasp, R_body=r_body,
                   normal_targets=normal_targets, orientation='keep')
    best = r2
    if not r2.get('ok'):
        r2c = api.skill('arm_pose_ik_baselock', targets=grasp,
                        R_body=r_body, normal_targets=normal_targets,
                        orientation='keep', correct=True)
        if r2c.get('ok') or (r2c.get('worst_pad_error') is not None
                             and (r2.get('worst_pad_error') is None
                                  or r2c['worst_pad_error']
                                  < r2['worst_pad_error'])):
            best = r2c
    if not best.get('ok'):
        return {'ok': False, 'stage': 'ik',
                'binding_constraint': best.get('binding_constraint'),
                'ik': r2, 'ik_corrected': r2c if best is not r2 else None,
                'best': best,
                'pad_error': best.get('pad_error'),
                'normal_error_deg': best.get('normal_error_deg'),
                'budget': {'base_drift': best.get('base_drift'),
                           'solver_residual': best.get('solver_residual'),
                           'target_frame_offset': best.get(
                               'target_frame_offset'),
                           'wrist_margin': best.get('wrist_margin')}}
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
    r4 = api.skill('arm_pose_ik_baselock', targets=tgt2, R_body=r_body,
                   normal_targets=normal_targets, orientation='keep',
                   hand_hold=closed)
    st1 = api.observe()
    obj1 = (st1.get('object') or {}).get('pos')
    rise = obj1[2] - obj0[2]
    contacts = api.hand_contacts_side()
    bil = (contacts.get('left', 0) > 0 and contacts.get('right', 0) > 0)
    sg = api.skill('safety_guard')
    pe = best.get('pad_error') or {}
    worst = None
    for side in pe:
        if worst is None or pe[side] > worst:
            worst = pe[side]
    guard_ok = sg.get('ok')
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
    budget = {'base_drift': best.get('base_drift'),
              'solver_residual': best.get('solver_residual'),
              'target_frame_offset': best.get('target_frame_offset'),
              'wrist_margin': best.get('wrist_margin'),
              'pad_error': pe,
              'normal_error_deg': best.get('normal_error_deg')}
    return {'ok': ok, 'stage': 'lift', 'pad_error': pe,
            'worst_pad_error': worst,
            'normal_error_deg': best.get('normal_error_deg'),
            'contacts': contacts, 'object_rise': rise, 'guard': sg,
            'budget': budget, 'standoff': r1, 'ik': r2,
            'ik_corrected': r2c if best is not r2 else None,
            'best_ik': best, 'close': r3, 'lift': r4,
            'binding_constraint': binding}
