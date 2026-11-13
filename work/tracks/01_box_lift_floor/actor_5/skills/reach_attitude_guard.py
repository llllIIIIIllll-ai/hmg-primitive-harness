def run(api, records=None, delta_ztol=0.10, level_ztol=0.12):
    """Decide, from records already returned by the free-space approach,
    whether the approach finished with a palm-pad attitude that is rolling
    out of the grasp and must be retried.

    Removes: a free-space approach that is accepted because it made no
    contact, yet leaves a pad pitched (its outward normal gaining a downward
    component) so that the following normal press slides that pad up the
    face and over the box top.  Measured: on the seed where the approach
    command ends with no hand-object contact, the right pad outward normal
    moves from [0.11, 0.99, 0.03] to [0.07, 0.98, -0.18] across the two
    commands of the y-before-face step, and the press then drives it to
    [0.0, 0.18, -0.98] with `ik_err` 0.27 while its height rises 0.538 ->
    0.585, above the box top (0.558).  On every seed whose approach does
    make contact at that step the safety abort already forces a retract and
    a re-approach, and the re-approached attitude is level (normal z in
    +0.01 .. +0.09) with `ik_err` <= 0.013.

    Contract: pre: `records` is the log of ONE free-space approach step (the
    last one before the press), each entry optionally carrying `pad_left` /
    `pad_right` dicts with a `normal` vector; a `None` or empty list is
    legal -> post: `{retry, reason, side, dz, z_first, z_last, samples}`.
    `retry` is True iff a side's normal-z falls by more than `delta_ztol`
    between its first and last sample (reason 'roll_away') or its last
    normal-z is below -`level_ztol` (reason 'down_tilt'); the reported `dz`
    and `z_*` belong to the worst side.  Reads no engine state and issues no
    command: it is a pure function of the passed records, so a caller that
    leaves `records=None` (or passes records with a level attitude) is
    bit-identical to not calling it.  `samples` is the number of normal
    samples used per side.

    Side effects: none.
    """
    recs = records or []
    n = np.shape(recs)[0]
    out = {'retry': False, 'reason': None, 'side': None, 'dz': None,
           'z_first': None, 'z_last': None, 'samples': 0}
    if n < 1:
        return out
    worst = None
    for s_ in ('left', 'right'):
        zs = []
        i = 0
        while i < n:
            pad_ = recs[i].get('pad_' + s_)
            if pad_ is not None:
                nv = pad_.get('normal')
                if nv is not None:
                    zs.append(nv[2])
            i += 1
        m = np.shape(zs)[0]
        if m < 1:
            continue
        z_first = zs[0]
        z_last = zs[m - 1]
        dz = z_last - z_first
        if m >= 2 and dz < -delta_ztol:
            cand = {'retry': True, 'reason': 'roll_away', 'side': s_,
                    'dz': np.round(dz, 4).item(),
                    'z_first': np.round(z_first, 4).item(),
                    'z_last': np.round(z_last, 4).item(), 'samples': m}
        elif z_last < -level_ztol:
            cand = {'retry': True, 'reason': 'down_tilt', 'side': s_,
                    'dz': np.round(dz, 4).item(),
                    'z_first': np.round(z_first, 4).item(),
                    'z_last': np.round(z_last, 4).item(), 'samples': m}
        else:
            cand = {'retry': False, 'reason': None, 'side': s_,
                    'dz': np.round(dz, 4).item(),
                    'z_first': np.round(z_first, 4).item(),
                    'z_last': np.round(z_last, 4).item(), 'samples': m}
        if cand['retry']:
            if worst is None or not worst['retry'] or cand['dz'] < worst['dz']:
                worst = cand
        elif worst is None:
            worst = cand
    if worst is None:
        return out
    out.update(worst)
    return out
