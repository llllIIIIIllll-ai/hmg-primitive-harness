def run(api, **kw):
    """Read-only sim state snapshot for the contract test."""
    m, d = api.raw
    q = np.array(d.qpos)
    v = np.array(d.qvel)
    return {'ok': True, 't': api.t, 'qpos': q.tolist(),
            'qvel': v.tolist()}
