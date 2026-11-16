def run(api, **kw):
    a = api.object_aabb()
    lo = np.array(a['lo'])
    hi = np.array(a['hi'])
    ag = api.arm_geoms()
    out = {}
    for body in ag:
        for gg in ag[body]['geoms']:
            nm = gg['name']
            if nm in ('robot0_g27_col', 'robot0_g28_col',
                      'robot0_g34_col', 'robot0_g35_col'):
                p = np.array(gg['world_pos'])
                near = np.clip(p, lo, hi)
                out[nm] = {'pos': gg['world_pos'],
                           'dist': np.linalg.norm(p - near)}
    b = api.bodies(match='bottle')
    geoms = []
    for nm in b:
        for gg in b[nm].get('geoms', []):
            geoms.append({'body': nm, 'name': gg.get('name'),
                          'aabb': gg.get('aabb')})
    return {'ok': True, 'object_aabb': a, 'wrist': out, 'object_bodies': geoms}
