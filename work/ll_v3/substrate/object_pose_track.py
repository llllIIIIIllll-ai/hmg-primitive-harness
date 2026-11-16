def run(api, target=None, tol=0.01, **kw):
    """Live object pose/AABB plus a pose error (if a target is given) and a
    tracking gap (change since the previous call in this episode)."""
    st = api.observe()
    ob = st.get('object') or {}
    pos = ob.get('pos')
    if pos is None:
        return {'ok': False, 'binding_constraint': 'no_object'}
    a = api.object_aabb()
    last = None
    if '_object_track_last' in api.__dict__:
        last = api.__dict__['_object_track_last']
    api._object_track_last = pos
    gap = None
    if last is not None:
        gap = ((pos[0] - last[0]) ** 2 + (pos[1] - last[1]) ** 2
               + (pos[2] - last[2]) ** 2) ** 0.5
    err = None
    if target is not None:
        err = ((pos[0] - target[0]) ** 2 + (pos[1] - target[1]) ** 2
               + (pos[2] - target[2]) ** 2) ** 0.5
    ok = True
    binding = None
    if err is not None and err > tol:
        ok = False
        binding = 'pose_error'
    return {'ok': ok, 'object': ob, 'aabb': a,
            'pose_error': err, 'tracking_gap': gap, 'target': target,
            'binding_constraint': binding}
