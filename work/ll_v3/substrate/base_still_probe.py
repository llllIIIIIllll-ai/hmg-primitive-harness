def run(api, steps=50, hold=False, base_gain=0.05, base_vmax=0.03,
        base_deadband=0.02, base_yaw_gain=0.3, base_yaw_deadband=0.01,
        base_wmax=0.05, **kw):
    """Hold the upper body exactly where it is.

    hold=False: send nav=0 -- measures the WBC's own drift/coasting with no
    arm motion (separates reaction from a stale nav command).
    hold=True:  send the corrected deadbanded base-hold law -- verifies the
    hold itself does not oscillate (the high-gain P loop wandered 0.7 m).
    """
    b0 = api.base_state()
    hold_xy = [b0['pos'][0], b0['pos'][1]]
    hold_yaw = b0['yaw_deg']
    tr = []
    k = 0
    while k < steps:
        nav = [0.0, 0.0, 0.0]
        if hold:
            b = api.base_state()
            dxh = hold_xy[0] - b['pos'][0]
            dyh = hold_xy[1] - b['pos'][1]
            yaw = np.radians(b['yaw_deg'])
            dxr = dxh * np.cos(yaw) + dyh * np.sin(yaw)
            dyr = -dxh * np.sin(yaw) + dyh * np.cos(yaw)
            ex = 0.0
            if dxr > base_deadband:
                ex = dxr - base_deadband
            elif dxr < -base_deadband:
                ex = dxr + base_deadband
            ey = 0.0
            if dyr > base_deadband:
                ey = dyr - base_deadband
            elif dyr < -base_deadband:
                ey = dyr + base_deadband
            ye = np.radians(hold_yaw - b['yaw_deg'])
            while ye > 3.14159:
                ye = ye - 6.28318
            while ye < -3.14159:
                ye = ye + 6.28318
            eyaw = 0.0
            if ye > base_yaw_deadband:
                eyaw = ye - base_yaw_deadband
            elif ye < -base_yaw_deadband:
                eyaw = ye + base_yaw_deadband
            wz = np.clip(base_yaw_gain * eyaw, -base_wmax, base_wmax)
            vy = np.clip(base_gain * ey, -base_vmax, base_vmax)
            vx = np.clip(base_gain * ex, -base_vmax, base_vmax)
            nav = [vx, vy, wz]
        api.step_goal(upper=api.upper(), nav=nav, dur=0.04)
        b = api.base_state()
        tr.append({'k': k, 't': api.t, 'pos': b['pos'],
                   'yaw_deg': b['yaw_deg'], 'nav': nav})
        k = k + 1
    b1 = api.base_state()
    dx = b1['pos'][0] - b0['pos'][0]
    dy = b1['pos'][1] - b0['pos'][1]
    drift = (dx * dx + dy * dy) ** 0.5
    return {'ok': True, 'drift': drift, 'b0': b0['pos'], 'b1': b1['pos'],
            'dyaw': b1['yaw_deg'] - b0['yaw_deg'],
            'sim_time': api.t, 'trace': tr}
