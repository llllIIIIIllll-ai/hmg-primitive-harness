def run(api, x=0.12, dur=2.0, **kw):
    b0 = api.base_state()
    r = api.move_base(x=x, y=0.0, yaw=0.0, dur=dur)
    b1 = api.base_state()
    dx = b1['pos'][0] - b0['pos'][0]
    dy = b1['pos'][1] - b0['pos'][1]
    dist = (dx * dx + dy * dy) ** 0.5
    rob = []
    for c in (api.contact_detail(limit=40, match='bottle') or []):
        s = (c.get('b1', '') + c.get('b2', ''))
        if 'robot' in s or 'gripper' in s:
            rob.append(c)
    return {'ok': True, 'moved': [dx, dy], 'dist': dist,
            'b0': b0, 'b1': b1, 'robot_contacts': rob}
