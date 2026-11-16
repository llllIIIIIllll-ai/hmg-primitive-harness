def run(api, joint='right_elbow_joint', delta=0.1, n_steps=200, **kw):
    q0 = api.upper()
    addr = api.addr[joint]
    idx = -1
    k = 0
    while k < 31:
        if api.qpos_addr[k] == addr:
            idx = k
            break
        k = k + 1
    if idx < 0:
        return {'error': 'joint not in contract'}
    target = api.upper()
    target[idx] = target[idx] + delta
    t0 = api.t
    traj = []
    t95 = None
    q95 = None
    k = 0
    while k < n_steps:
        api.step_goal(upper=target, dur=0.02)
        q = api.upper()
        frac = (q[idx] - q0[idx]) / delta
        if k % 5 == 0:
            traj.append([np.round(api.t - t0, 3), np.round(q[idx].item(), 5)])
        if t95 is None and frac >= 0.95:
            t95 = np.round(api.t - t0, 3)
            q95 = np.round(q[idx].item(), 5)
        k = k + 1
    qend = api.upper()[idx].item()
    return {'joint': joint, 'delta_cmd': delta, 'q0': q0[idx].item(),
            'qend': qend, 'achieved_frac': (qend - q0[idx].item()) / delta,
            't95_s': t95, 'q95': q95, 'traj': traj,
            't_end': np.round(api.t - t0, 3)}
