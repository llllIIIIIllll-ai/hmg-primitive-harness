def run(api, ramp_s=0.15, hold_s=0.5, open_tol=0.15, **kw):
    """Open the fingers to the neutral (non-closing) target and report the
    contacts and object disturbance after the release."""
    c0 = api.hand_contacts_side()
    st0 = api.observe()
    ob0 = st0.get('object') or {}
    obj0 = ob0.get('pos')
    r = api.close_grip(angle=0.0, dur=ramp_s)
    open_target = r.get('upper_target')
    api.step_goal(upper=open_target, dur=hold_s)
    c1 = api.hand_contacts_side()
    st1 = api.observe()
    ob1 = st1.get('object') or {}
    obj1 = ob1.get('pos')
    dist = 0.0
    if obj0 is not None and obj1 is not None:
        dist = ((obj1[0] - obj0[0]) ** 2 + (obj1[1] - obj0[1]) ** 2
                + (obj1[2] - obj0[2]) ** 2) ** 0.5
    jr = api.joint_ranges()
    open_err = 0.0
    for nm in jr:
        if '_hand_' in nm:
            if np.abs(jr[nm]['q']) > open_err:
                open_err = np.abs(jr[nm]['q'])
    before = (c0.get('left', 0) > 0 and c0.get('right', 0) > 0)
    after = (c1.get('left', 0) > 0 or c1.get('right', 0) > 0)
    open_ok = True if open_err <= open_tol else False
    release_ok = True if ((not before) or (not after)) else False
    ok = True if (open_ok and release_ok) else False
    binding = None
    if not open_ok:
        binding = 'open_target'
    elif before and after:
        binding = 'still_held'
    return {'ok': ok, 'contacts_before': c0, 'contacts_after': c1,
            'object_disturbance': dist, 'open_joint_error': open_err,
            'open_target_present': open_target is not None,
            'release_test_applicable': before,
            'binding_constraint': binding,
            'setup_note': (None if before else 'no_grasp_to_open')}
