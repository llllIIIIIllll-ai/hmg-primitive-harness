def run(api, x=0.12, dur=5.0, settle=1.5, log_every=10, **kw):
    n_settle = 0
    while n_settle < 75:
        api.step_goal(upper=api.upper(), nav=[0.0, 0.0, 0.0], dur=0.02)
        n_settle = n_settle + 1
    b0 = api.base_state()
    m, d = api.raw
    tr = []
    n = 0
    nsteps = 250
    while n < nsteps:
        api.step_goal(upper=api.upper(), nav=[x, 0.0, 0.0], dur=0.02)
        if n % log_every == 0:
            b = api.base_state()
            vel = [d.qvel[0], d.qvel[1], d.qvel[2]]
            g = api.geom()
            feet = {}
            for nm in (g.get('robot_bodies') or {}):
                if 'ankle' in nm or 'shin' in nm or 'foot' in nm:
                    feet[nm] = g['robot_bodies'][nm]
            scene = []
            for c in (api.contact_detail(limit=200, match='') or []):
                s = c.get('b1', '') + c.get('b2', '')
                rob = ('robot' in s or 'gripper' in s)
                env = ('table' in s or 'floor' in s or 'shelf' in s
                       or 'wall' in s)
                if rob and env:
                    scene.append({'b1': c.get('b1'), 'b2': c.get('b2'),
                                  'dist': c.get('dist')})
            ns = 0
            for c in scene:
                ns = ns + 1
            tr.append({'k': n, 't': api.t, 'pos': b['pos'],
                       'tilt': b['tilt_deg'], 'yaw': b['yaw_deg'],
                       'vel': vel, 'feet': feet, 'scene': scene,
                       'nscene': ns})
        n = n + 1
    b1 = api.base_state()
    dx = b1['pos'][0] - b0['pos'][0]
    dy = b1['pos'][1] - b0['pos'][1]
    return {'ok': True, 'dist': (dx * dx + dy * dy) ** 0.5,
            'b0': b0, 'b1': b1, 'trace': tr}
