def run(api, tilt_limit=8.0, joint_margin_min=0.05,
        penetration_tol=0.002, **kw):
    """Read-only safety guard: joint margin, base tilt, penetration.

    Never commands.  Returns abort=True and the first binding constraint when
    a limit is met or crossed.
    """
    b = api.base_state()
    jr = api.joint_ranges()
    worst = None
    worst_joint = None
    for nm in jr:
        if '_hand_' in nm:
            continue
        q = jr[nm]['q']
        lo = jr[nm]['lo']
        hi = jr[nm]['hi']
        m = q - lo
        if hi - q < m:
            m = hi - q
        if worst is None or m < worst:
            worst = m
            worst_joint = nm
    cd = api.contact_detail(limit=200)
    pen = 0.0
    pen_pairs = []
    for c in cd:
        if c['dist'] < -penetration_tol:
            if -c['dist'] > pen:
                pen = -c['dist']
            pen_pairs.append({'g1': c['g1'], 'g2': c['g2'],
                              'dist': c['dist']})
    violations = []
    if b['tilt_deg'] > tilt_limit:
        violations.append('base_tilt')
    if worst is not None and worst < joint_margin_min:
        violations.append('joint_margin')
    if pen > penetration_tol:
        violations.append('penetration')
    ok = True
    if violations:
        ok = False
    binding = None
    if violations:
        binding = violations[0]
    return {'ok': ok, 'abort': not ok, 'violations': violations,
            'base_tilt_deg': b['tilt_deg'],
            'joint_margin_min': worst,
            'joint_margin_joint': worst_joint,
            'penetration_max': pen,
            'penetration_pairs': pen_pairs,
            'binding_constraint': binding}
