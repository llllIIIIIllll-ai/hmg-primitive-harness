def run(api, box_ref=None, press_steps=None, press_ncmd=4, press_out0=0.004,
        off=(0.076, -0.018, -0.030), x_in=0.03, z_frac=0.94,
        pad_half=0.0217, box_move_tol=0.02, box_move_hard=0.25,
        place_dur=0.40):
    """Seat both pads on the opposed box faces by a three-step normal press.

    Removes: a one-sided grip that closes with the right pad only.  A single
    or double correction command per press step leaves the left pad clear of
    its face on the hardest seed (close measured left 0 / right 3), and the
    later squeeze then makes contact only by dragging the box; four commands
    per step is the smallest count measured to seat BOTH pads before the
    fingers close (close 5 contacts, left 1 / right 4, box displacement
    0.109 m, under the 0.12 m plough limit).

    Contract: pre: pads already brought to the face pair by
    `place_pads_closed_loop` free-space waypoints, a settled `box_ref`, and
    the box still at rest -> post: for each entry of `press_steps` (default
    [0.012, 0.003, -0.006] m of inboard stand offset) the pads are commanded
    normal to the faces with `press_ncmd` measured corrections each; stops at
    the first step whose per-command displacement gate trips.  Returns
    `{ok, failed_at, records, contacts, sides, commands}`; `contacts`/`sides`
    are the LAST measured values, read by the last placement command itself.
    Commands arms only; the fingers are left untouched for the caller to
    close.

    Side effects: one `geom_detail` per press step (top of loop) and, per
    command, one `reach_box` + one `geom_detail` + two `observe` +
    `hand_contacts_side` via `place_pads_closed_loop`.  Adds NO trailing
    observation, so a caller that closes immediately keeps the same engine
    read sequence as closing directly after the press.
    """
    if box_ref is None:
        return {'ok': False, 'failed_at': 'no-box-ref', 'records': [],
                'contacts': None, 'sides': None, 'commands': 0}
    if press_steps is None:
        press_steps = [0.012, 0.003, -0.006]
    recs = []
    ok = True
    failed_at = None
    pi = 0
    npress = np.shape(press_steps)[0]
    while pi < npress:
        gg = api.geom_detail()
        lo_ = gg['world_lo']
        hi_ = gg['world_hi']
        rp = api.skill(
            'place_pads_closed_loop', tag='press%d' % pi,
            stand=press_steps[pi], lo=lo_, hi=hi_, box_ref=box_ref,
            ncmd=press_ncmd, free=False, out0=press_out0, dur=place_dur,
            off=off, x_in=x_in, z_frac=z_frac, pad_half=pad_half,
            box_move_tol=box_move_tol, box_move_hard=box_move_hard)
        recs.extend(rp.get('log') or [])
        okp = rp.get('ok')
        pi += 1
        if not okp:
            ok = False
            failed_at = 'press%d' % (pi - 1)
            break
    last = recs[-1] if recs else {}
    return {'ok': ok, 'failed_at': failed_at, 'records': recs,
            'log': recs, 'contacts': last.get('contacts'),
            'sides': last.get('sides'),
            'commands': last.get('commands')}
