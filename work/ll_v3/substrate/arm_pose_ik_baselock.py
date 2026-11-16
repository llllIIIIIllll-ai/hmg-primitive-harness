def run(api, targets=None, R_body=None, normal_targets=None,
        orientation='keep', max_steps=50, iters=80, hand_hold=None,
        stop_err=0.012, hold_s=0.1, correct=False, **kw):
    """W2b/W2c arm pose IK with a per-control-step QP re-solve against the
    live base.  Optional explicit R_body (grasp orientation), desired pad
    normals for the normal-error report, and a per-step target correction
    (site-error feedback) for W2c step (b)."""
    hg0 = api.hand_geoms()
    n0 = {}
    for body in hg0:
        for gg in hg0[body]['geoms']:
            for side in ('left', 'right'):
                if gg['name'] == 'gripper0_%s_g0_col' % side:
                    n0[side] = np.array(gg['xmat'])[:, 0]
    b0 = api.base_state()
    tgt_use = {}
    for side in (targets or {}):
        tgt_use[side] = [targets[side][0], targets[side][1], targets[side][2]]
    r = None
    worst = None
    k = 0
    while k < max_steps:
        r = api.reach_box_qp(task_dim=12, targets=tgt_use, R_body=R_body,
                             orientation=orientation, dur=0.02,
                             iters=iters, hand_hold=hand_hold)
        pr = r.get('palm_residual') or {}
        worst = None
        for side in pr:
            v = pr[side]
            e = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
            if worst is None or e > worst:
                worst = e
        if correct:
            gnow = api.geom()
            for side in (targets or {}):
                site = gnow['sites'].get(side)
                if site is None:
                    continue
                for i in (0, 1, 2):
                    tgt_use[side][i] = tgt_use[side][i] + (
                        targets[side][i] - site['pos'][i])
        if worst is not None and worst <= stop_err:
            break
        k = k + 1
    api.step_goal(upper=r.get('upper_target'), dur=hold_s)
    b1 = api.base_state()
    g = api.geom()
    pe = {}
    final = None
    for side in (targets or {}):
        site = g['sites'].get(side)
        if site is None:
            continue
        t = targets[side]
        v = [site['pos'][0] - t[0], site['pos'][1] - t[1],
             site['pos'][2] - t[2]]
        e = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
        pe[side] = e
        if final is None or e > final:
            final = e
    hg1 = api.hand_geoms()
    ne = {}
    for body in hg1:
        for gg in hg1[body]['geoms']:
            for side in ('left', 'right'):
                if gg['name'] == 'gripper0_%s_g0_col' % side:
                    n1 = np.array(gg['xmat'])[:, 0]
                    if normal_targets is not None and side in normal_targets:
                        tgt_n = np.array(normal_targets[side])
                        tgt_n = tgt_n / np.linalg.norm(tgt_n)
                    elif side in n0:
                        tgt_n = n0[side]
                    else:
                        continue
                    d = np.clip(np.dot(tgt_n, n1), -1.0, 1.0)
                    ne[side] = np.degrees(np.arccos(d))
    max_normal = None
    for side in ne:
        if max_normal is None or ne[side] > max_normal:
            max_normal = ne[side]
    target_frame_offset = 0.0
    if max_normal is not None:
        target_frame_offset = 0.0838 * np.sin(np.radians(max_normal))
    base_drift = ((b1['pos'][0] - b0['pos'][0]) ** 2
                  + (b1['pos'][1] - b0['pos'][1]) ** 2
                  + (b1['pos'][2] - b0['pos'][2]) ** 2) ** 0.5
    jr = api.joint_ranges()
    wrist_margin = None
    for nm in jr:
        if 'wrist' in nm:
            q = jr[nm]['q']
            lo = jr[nm]['lo']
            hi = jr[nm]['hi']
            m = q - lo
            if hi - q < m:
                m = hi - q
            if wrist_margin is None or m < wrist_margin:
                wrist_margin = m
    ik_err = (r or {}).get('ik_err') or {}
    solver_residual = None
    for side in ik_err:
        if solver_residual is None or ik_err[side] > solver_residual:
            solver_residual = ik_err[side]
    margin = (r or {}).get('joint_margin_min')
    solver = (r or {}).get('solver')
    ok = (final is not None and final <= 0.015
          and margin is not None and margin >= 0.05
          and solver == 'mink-qp')
    binding = None
    if not ok:
        if final is None:
            binding = 'no_solution'
        elif final > 0.015:
            binding = 'pad_error'
        elif margin is None or margin < 0.05:
            binding = 'joint_margin'
        else:
            binding = 'solver'
    return {'ok': ok, 'pad_error': pe, 'worst_pad_error': final,
            'stop_error': worst, 'steps': k + 1, 'base_drift': base_drift,
            'joint_margin_min': margin, 'solver': solver,
            'ik_err': ik_err, 'normal_error_deg': ne,
            'target_frame_offset': target_frame_offset,
            'wrist_margin': wrist_margin,
            'solver_residual': solver_residual,
            'targets': targets, 'targets_used': tgt_use,
            'r_body': R_body, 'binding_constraint': binding}
