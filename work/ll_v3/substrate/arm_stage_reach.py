def run(api, grasp=None, target_site=None, side='left', n_stages=5, stop_err=0.02,
        max_delta=0.04, max_delta_near=0.02, near_dist=0.03,
        deadband_comp=0.04, orient_mode='side', orientation='keep',
        trace_contacts=True, contact_match='bottle', **kw):
    """Staged reach: interpolate the SITE target from the live pad position
    to the final grasp cell and drive each short goal with the true closed
    loop -- re-solving from the live state every step, so no stage commits
    to a distant branch.  Live collider guard on every stage."""
    dx0 = (grasp or {}).get('dx', -0.105)
    dy0 = (grasp or {}).get('dy', 0.0)
    z0 = (grasp or {}).get('z', 0.954)
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    b = api.base_state()
    dxv = obj[0] - b['pos'][0]
    dyv = obj[1] - b['pos'][1]
    dd = (dxv * dxv + dyv * dyv) ** 0.5
    fx = dxv / dd
    fy = dyv / dd
    px = -fy
    py = fx
    if target_site is not None:
        T = [target_site[0], target_site[1], target_site[2]]
    else:
        T = [obj[0] + dx0 * fx + dy0 * px, obj[1] + dx0 * fy + dy0 * py, z0]
    g = api.geom()
    site = g['sites'].get(side)
    P = [site['pos'][0], site['pos'][1], site['pos'][2]]
    rows = []
    r = None
    k = 1
    while k <= n_stages:
        s = k / (n_stages + 0.0)
        X = [P[0] + (T[0] - P[0]) * s, P[1] + (T[1] - P[1]) * s,
             P[2] + (T[2] - P[2]) * s]
        d = [X[0] - obj[0], X[1] - obj[1]]
        dxk = d[0] * fx + d[1] * fy
        dyk = d[0] * px + d[1] * py
        r = api.skill('arm_pose_ik_live',
                      grasp={'dx': dxk, 'dy': dyk, 'z': X[2]},
                      orientation=orientation, orient_mode=orient_mode,
                      sides=(side,), max_delta=max_delta,
                      max_delta_near=max_delta_near, near_dist=near_dist,
                      max_steps=240, stop_err=stop_err,
                      deadband_comp=deadband_comp, wrist_guard=True,
                      wrist_guard_dist=0.02, seed_mode='dls',
                      seed_force=True, freeze_frame=True,
                      reorient_each_step=True, base_gain=0.3,
                      base_deadband=0.005, base_vmax=0.05,
                      base_yaw_deadband=0.003, base_yaw_gain=0.6,
                      base_wmax=0.1, trace_contacts=trace_contacts,
                      contact_match=contact_match)
        pe = (r.get('pad_error') or {}).get(side)
        rows.append({'stage': k, 'frac': s,
                     'grasp': {'dx': dxk, 'dy': dyk, 'z': X[2]},
                     'ok': r.get('ok'), 'pad_error': pe,
                     'worst_pad_error': r.get('worst_pad_error'),
                     'steps': r.get('steps'),
                     'wrist_abort': r.get('wrist_abort'),
                     'wrist_dist_min': r.get('wrist_dist_min'),
                     'margin_min': r.get('joint_margin_min'),
                     'margin_joint': r.get('joint_margin_joint'),
                     'residual': r.get('solver_residual'),
                     'resets': r.get('resets'), 'nans': r.get('nans'),
                     'base_drift': r.get('base_drift')})
        if r.get('wrist_abort') or r.get('resets') or r.get('nans'):
            break
        if pe is not None and pe > stop_err * 4.0:
            break
        k = k + 1
    last = None
    for rr_ in rows:
        last = rr_
    return {'ok': True if (last is not None and last.get('pad_error')
                           is not None and last['pad_error'] <= stop_err
                           and not last.get('wrist_abort')) else False,
            'stages': rows, 'n_stages': n_stages,
            'final': last, 'target': T, 'start_site': P,
            'last_result': r}
