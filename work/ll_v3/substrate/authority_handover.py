def run(api, delta=-0.2, ramp_s=0.15, hold_s=1.0, tilt_limit=8.0,
        min_gain=0.8, **kw):
    """Authorized waist handover and re-handover with the achieved-gain check.

    The roll joint is the discriminator: its homie gain is below 0.8 and its
    ik gain is above 0.8.  The pitch/yaw upper-channel response is high under
    both owners, so it is recorded but not used as the pass criterion.
    """
    joints = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')
    events = []
    b0 = api.base_state()
    g0 = api.track_gain(joints=joints, delta=delta, dur=ramp_s, hold=hold_s)
    gains0 = {}
    for nm in joints:
        gains0[nm] = g0['joints'][nm]['gain']
    events.append({'event': 'before', 'owner': g0.get('waist_owner'),
                   'gains': gains0, 'tilt_deg': b0['tilt_deg']})

    a1 = api.authority(waist_owner='ik')
    b1 = api.base_state()
    g1 = api.track_gain(joints=joints, delta=delta, dur=ramp_s, hold=hold_s)
    gains1 = {}
    for nm in joints:
        gains1[nm] = g1['joints'][nm]['gain']
    events.append({'event': 'handoff', 'owner': a1.get('waist_owner'),
                   'gains': gains1, 'tilt_deg': b1['tilt_deg']})

    a2 = api.authority(waist_owner='homie')
    b2 = api.base_state()
    g2 = api.track_gain(joints=joints, delta=delta, dur=ramp_s, hold=hold_s)
    gains2 = {}
    for nm in joints:
        gains2[nm] = g2['joints'][nm]['gain']
    events.append({'event': 'restore', 'owner': a2.get('waist_owner'),
                   'gains': gains2, 'tilt_deg': b2['tilt_deg']})

    gb = gains0['waist_roll_joint']
    gi = gains1['waist_roll_joint']
    ga = gains2['waist_roll_joint']
    owner_ok = (a1.get('waist_owner') == 'ik'
                and a2.get('waist_owner') == 'homie')
    gain_ok = (np.abs(gb) < min_gain and np.abs(gi) >= min_gain
               and np.abs(ga) < min_gain)
    bal_ok = (b1['tilt_deg'] <= tilt_limit
              and b2['tilt_deg'] <= tilt_limit)
    ok = owner_ok and gain_ok and bal_ok
    binding = None
    if a2.get('waist_owner') != 'homie':
        binding = 'restore_failed'
    elif not gain_ok:
        if np.abs(gi) < min_gain:
            binding = 'ik_gain'
        elif np.abs(ga) >= min_gain:
            binding = 'restore_gain'
        else:
            binding = 'homie_gain_not_low'
    elif not bal_ok:
        binding = 'balance'
    return {'ok': ok, 'events': events,
            'roll_gains': {'before': gb, 'ik': gi, 'after': ga},
            'all_gains': {'before': gains0, 'ik': gains1, 'after': gains2},
            'binding_constraint': binding}
