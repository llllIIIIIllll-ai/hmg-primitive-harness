def run(api, targets=None, gap=0.155, z_off=0.45, x_off=0.0,
        orientation='keep', iters=80, ramp_s=0.35, hold_s=0.2, **kw):
    """Two palm poses (12-D) with the waist exogenous, mink QP default.

    Uses the measured ramp curve (0.35 s for a 1 rad delta) plus a short
    settling hold, then reports the executed palm/site error, pad normal error,
    joint margin and one binding constraint.  The 1.5 s fixed ramp is not used.
    """
    hg0 = api.hand_geoms()
    n0 = {}
    for body in hg0:
        for gg in hg0[body]['geoms']:
            for side in ('left', 'right'):
                if gg['name'] == 'gripper0_%s_g0_col' % side:
                    n0[side] = np.array(gg['xmat'])[:, 0]
    r = api.reach_box_qp(task_dim=12, gap=gap, z_off=z_off, x_off=x_off,
                         targets=targets, orientation=orientation,
                         dur=ramp_s, iters=iters)
    ramp_err = {}
    for side in (r.get('palm_residual') or {}):
        v = r['palm_residual'][side]
        ramp_err[side] = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
    api.step_goal(upper=r.get('upper_target'), dur=hold_s)
    g = api.geom()
    pe = {}
    worst = None
    for side in (r.get('target') or {}):
        site = g['sites'].get(side)
        if site is None:
            continue
        t = r['target'][side]
        v = [site['pos'][0] - t[0], site['pos'][1] - t[1],
             site['pos'][2] - t[2]]
        e = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
        pe[side] = e
        if worst is None or e > worst:
            worst = e
    ne = {}
    hg1 = api.hand_geoms()
    for body in hg1:
        for gg in hg1[body]['geoms']:
            for side in ('left', 'right'):
                if gg['name'] == 'gripper0_%s_g0_col' % side:
                    if side in n0:
                        n1 = np.array(gg['xmat'])[:, 0]
                        d = np.clip(np.dot(n0[side], n1), -1.0, 1.0)
                        ne[side] = np.degrees(np.arccos(d))
    m = r.get('joint_margin_min')
    owner = r.get('waist_owner')
    solver = r.get('solver')
    ok = (worst is not None and worst <= 0.015
          and m is not None and m >= 0.05
          and solver == 'mink-qp' and owner == 'homie')
    binding = None
    if not ok:
        if worst is None:
            binding = 'no_solution'
        elif worst > 0.015:
            binding = 'pad_error'
        elif m is None or m < 0.05:
            binding = 'joint_margin'
        elif owner != 'homie':
            binding = 'authority'
        else:
            binding = 'solver'
    return {'ok': ok, 'pad_error': pe, 'worst_pad_error': worst,
            'pad_error_at_ramp_end': ramp_err,
            'normal_error_deg': ne, 'joint_margin_min': m,
            'waist_owner': owner, 'waist_exogenous': r.get('waist_exogenous'),
            'solver': solver, 'ramp_s': ramp_s, 'hold_s': hold_s,
            'ik_err': r.get('ik_err'), 'ik_rot': r.get('ik_rot'),
            'pad': r.get('pad'), 'target': r.get('target'),
            'binding_constraint': binding}
