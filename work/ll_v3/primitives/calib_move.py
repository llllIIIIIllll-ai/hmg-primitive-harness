def run(api, x=0.0, y=0.0, yaw=0.0, dur=0.5, **kw):
    b0 = api.base_state()
    r = api.move_base(x=x, y=y, yaw=yaw, dur=dur)
    b1 = api.base_state()
    return {'b0': b0, 'b1': b1, 'move': r}
