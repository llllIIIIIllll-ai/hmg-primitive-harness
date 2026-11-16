def run(api, targets=None, orientation='keep', grip_angle=0.6, **kw):
    out = []
    for t in targets:
        r = api.grasp_scratch({'left': t['left'], 'right': t['right']},
                              orientation=orientation, grip_angle=grip_angle)
        out.append({'target': t, 'contacts': r.get('contacts'),
                    'all_object_contacts': r.get('all_object_contacts'),
                    'sides': r.get('sides'), 'fingers': r.get('fingers')})
    return {'results': out}
