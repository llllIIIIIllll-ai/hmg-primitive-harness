def run(api):
    r = api.joint_ranges()
    out = {}
    for nm in r:
        d = r[nm]
        lo, hi, q = d.get('lo'), d.get('hi'), d.get('q')
        m = None
        if lo is not None and hi is not None and q is not None:
            a = q - lo
            b = hi - q
            m = a if a < b else b
        out[nm] = {'q': q, 'lo': lo, 'hi': hi, 'margin': m}
    u = api.upper()
    return {'ok': True, 't': api.t, 'joints': out,
            'upper': u.tolist()}
