def run(api, x=-0.12, dur=4.5, **kw):
    r = api.move_base(x=x, y=0.0, yaw=0.0, dur=dur)
    return {'ok': True, 'moved_xy': r.get('moved_xy'), 't': r.get('t')}
