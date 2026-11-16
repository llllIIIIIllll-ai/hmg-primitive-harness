def run(api, fwd=0.35, dy=0.07, z=1.16, **kw):
    """Read-only: can the QP establish a palms-down pose in FREE space from
    the current (tucked) arm pose?"""
    b = api.base_state()
    tgt = {'left': [b['pos'][0] + fwd, b['pos'][1] + dy, z],
           'right': [b['pos'][0] + fwd, b['pos'][1] - dy, z]}
    r = api.reach_box_qp(task_dim=12, targets=tgt, orientation='down',
                         dur=0.02, apply=False)
    return {'ok': True, 't': api.t, 'targets': tgt,
            'ik_err': r.get('ik_err'), 'ik_rot': r.get('ik_rot'),
            'joint_margin_min': r.get('joint_margin_min'),
            'joint_margin_joint': r.get('joint_margin_joint')}
