def run(api, **kw):
    jr = api.joint_ranges()
    worst = None
    name = None
    rows = {}
    for nm in jr:
        q = jr[nm]['q']
        lo = jr[nm]['lo']
        hi = jr[nm]['hi']
        m = q - lo
        if hi - q < m:
            m = hi - q
        rows[nm] = m
        if worst is None or m < worst:
            worst = m
            name = nm
    return {'ok': True, 'worst': worst, 'joint': name, 'margins': rows}
