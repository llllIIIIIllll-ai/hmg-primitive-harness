def run(api, **kw):
    oc = api.object_colliders()
    cols = oc.get('colliders') or []
    out = []
    for c in cols:
        out.append({'name': c.get('name'), 'type': c.get('type'),
                    'world_pos': c.get('world_pos'), 'size': c.get('size')})
    return {'ok': True, 'body': oc.get('body'), 'colliders': out}
