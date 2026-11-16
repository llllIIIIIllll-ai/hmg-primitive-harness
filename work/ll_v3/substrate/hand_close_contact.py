def run(api, max_angle=0.8, steps=8, ramp_s=0.15, hold_s=0.5,
        disturbance_tol=0.02, **kw):
    """Contact-triggered close: step the fingers until bilateral contact or
    the close target.

    The close-phase contact is reported but is not the pass condition: on the
    05 drill it is often 0 at the close and appears during the lift.  The
    substrate passes when the close reaches its target (or contact), the grip
    does not degrade during the hold, and the object is not disturbed; the lift
    L2 adds the lift-phase bilateral-contact check.
    """
    jr = api.joint_ranges()
    hand = []
    for nm in jr:
        if '_hand_' in nm:
            hand.append(nm)
    idx = {}
    for nm in hand:
        addr = api.addr[nm]
        k = 0
        while k < 31:
            if api.qpos_addr[k] == addr:
                idx[nm] = k
                break
            k = k + 1
    st0 = api.observe()
    ob0 = st0.get('object') or {}
    obj0 = ob0.get('pos')
    contacts = None
    ang = 0.0
    k = 0
    while k < steps:
        ang = max_angle * (k + 1) / steps
        q = api.upper()
        for nm in hand:
            lo = jr[nm]['lo']
            hi = jr[nm]['hi']
            sgn = 0.0
            if np.abs(hi) > np.abs(lo) + 1e-9:
                sgn = 1.0
            elif np.abs(lo) > np.abs(hi) + 1e-9:
                sgn = -1.0
            q[idx[nm]] = np.clip(sgn * ang, lo, hi)
        api.step_goal(upper=q, dur=ramp_s)
        contacts = api.hand_contacts_side()
        if contacts.get('left', 0) > 0 and contacts.get('right', 0) > 0:
            break
        k = k + 1
    qhold = api.upper()
    api.step_goal(upper=qhold, dur=hold_s)
    contacts_hold = api.hand_contacts_side()
    st1 = api.observe()
    ob1 = st1.get('object') or {}
    obj1 = ob1.get('pos')
    dist = 0.0
    if obj0 is not None and obj1 is not None:
        dist = ((obj1[0] - obj0[0]) ** 2 + (obj1[1] - obj0[1]) ** 2
                + (obj1[2] - obj0[2]) ** 2) ** 0.5
    bil = (contacts is not None and contacts.get('left', 0) > 0
           and contacts.get('right', 0) > 0)
    closed = (ang >= max_angle - 1e-6) or bil
    stable = (contacts_hold.get('left', 0) >= contacts.get('left', 0)
              and contacts_hold.get('right', 0) >= contacts.get('right', 0))
    ok = closed and stable and dist <= disturbance_tol
    binding = None
    if not closed:
        binding = 'close_incomplete'
    elif not stable:
        binding = 'slip'
    elif dist > disturbance_tol:
        binding = 'object_disturbance'
    return {'ok': ok, 'closed_angle': ang, 'steps': k + 1,
            'contacts': contacts, 'contacts_after_hold': contacts_hold,
            'bilateral_at_close': bil,
            'object_disturbance': dist,
            'object_before': obj0, 'object_after': obj1,
            'closed_upper': qhold.tolist(),
            'binding_constraint': binding}
