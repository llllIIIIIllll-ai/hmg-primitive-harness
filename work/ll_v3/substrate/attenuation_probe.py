def run(api, joint='left_shoulder_roll_joint', step=0.01, steps=5,
        sign=1.0, **kw):
    """Single-joint authority probe: command q[joint] += sign*step for
    `steps` control steps; report the live delta per step and the gain."""
    jj = api.addr[joint]
    idx = None
    k = 0
    while k < 31:
        if api.qpos_addr[k] == jj:
            idx = k
            break
        k = k + 1
    if idx is None:
        return {'ok': False, 'error': 'joint not in contract order'}
    tr = []
    n = 0
    while n < steps:
        q = api.upper()
        cmd = q.copy()
        cmd[idx] = q[idx] + sign * step
        api.step_goal(upper=cmd, nav=[0.0, 0.0, 0.0], dur=0.02)
        q1 = api.upper()
        tr.append({'k': n, 'delta': q1[idx] - q[idx], 'q': q1[idx]})
        n = n + 1
    gsum = 0.0
    gmin = None
    gmax = None
    for e in tr:
        g = e['delta'] / (sign * step)
        gsum = gsum + g
        if gmin is None or g < gmin:
            gmin = g
        if gmax is None or g > gmax:
            gmax = g
    return {'ok': True, 'joint': joint, 'idx': idx, 'step': sign * step,
            'deltas': tr, 'gain_mean': gsum / steps,
            'gain_min': gmin, 'gain_max': gmax,
            'q_start': tr[0]['q'] - tr[0]['delta'],
            'q_end': tr[-1]['q']}
