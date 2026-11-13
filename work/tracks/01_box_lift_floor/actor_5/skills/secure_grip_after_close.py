def run(api, box_ref=None, hold=None, press_records=None, min_contacts=4,
        empty_min=2, seated_tol=None, seated_min=-0.02, stand_deep=-0.013,
        squeeze_ncmd=2, press_out0=0.004, place_out0=0.015, recover_out=0.03,
        recover_stand=0.035, recover_press=-0.004, recover_ncmd=2,
        off=(0.076, -0.018, -0.030), x_in=0.03, z_frac=0.94,
        pad_half=0.0217, box_move_tol=0.02, box_move_hard=0.25,
        place_dur=0.40, retract_dur=0.35):
    """Decide the post-close squeeze and the empty-grip recovery from the
    measured press record, and run only the fallback that the record allows.

    Removes: a deep inboard squeeze (`squeeze_deep`) commanded after a press
    that already left BOTH pads on the faces.  With both pads seated, the
    only thing an inboard normal offset can do is plough the box: the
    measured record on the failing seed is squeeze command 1 displacing the
    box 0.020 m and command 2 displacing it 0.128 m while hand contacts fall
    from 2 to 0, after which the lift raises the body over a box left behind
    (box z 0.148, AABB top 0.276, t 23.96 -- a shove, not a lift).  Skipping
    that squeeze keeps the contacts the close achieved and lets the lift
    carry the box.

    Contract: pre: fingers already commanded closed (`hold` is the commanded
    upper pose), `press_records` is the record list from the press step (each
    record carries `clear_m` per side), `box_ref` the settled reference ->
    post: one `observe`; if the last press record does NOT show both pads
    seated in `[seated_min, seated_tol]` and contacts < `min_contacts`, one
    `place_pads_closed_loop` squeeze at `stand_deep` m of inboard offset with
    `squeeze_ncmd` commands; one `observe`; if contacts < `empty_min`, one
    retract (`retract_dur`), one wide re-place at `recover_stand` and one
    press at `recover_press`, then one `observe` and one `hand_contacts_side`
    for the recovery record.  Returns `{log, squeezed, recovered,
    both_seated, contacts_end, box_end}`.

    Switch: `seated_tol=None` disables the squeeze guard (pinned behaviour);
    a numeric `seated_tol` enables it, and `seated_min` is the floor that
    rejects a pad jammed through its face (the measured jam state is
    -0.10 m).  When the guard does not fire, this skill issues exactly the
    pinned read/command order (observe, optional squeeze with a fresh
    `geom_detail`, observe, optional retract + `geom_detail` + two re-places,
    observe, side read).

    Side effects: the reads named above (`observe` x2, plus per squeeze one
    `geom_detail` per command and per retract one `geom` + one `reach_box` +
    one `observe`); the fallbacks command arms only, and the recovery is the
    only path that can move the box.  Never commands the base or the
    fingers.
    """
    log = []
    Rb = {'left': np.array([[1., 0, 0], [0, -1., 0], [0, 0, -1.]]),
          'right': np.array([[1., 0, 0], [0, 1., 0], [0, 0, 1.]])}
    b_ref = np.array(box_ref, dtype='float64')

    def _box_pos():
        return np.array(api.observe()['object']['pos'])

    def _ab():
        gg = api.geom_detail()
        return np.array(gg['world_lo']), np.array(gg['world_hi'])

    def _sides():
        sr = api.hand_contacts_side()
        return {'left': sr.get('left'), 'right': sr.get('right')}

    def _retract_out(dy):
        gg = api.geom()
        tgt = {}
        for s_ in ('left', 'right'):
            sg = 1.0 if s_ == 'left' else -1.0
            tgt[s_] = (np.array(gg['sites'][s_]['pos'])
                       + np.array([0.0, sg * dy, 0.0])).tolist()
        api.reach_box(targets=tgt, R_body=Rb, dur=retract_dur, iters=300)
        o = api.observe()
        log.append({'phase': 'retract_out', 'base': o['base'],
                    'box': o['object']['pos'],
                    'moved_l1_m': np.round(
                        np.sum(np.abs(_box_pos() - b_ref)), 4).item(),
                    'contacts': o['hand_object_contacts']})

    def _run_place(tag, stand, xb, zu, ncmd, free, hold_=None, out0=None,
                   lo_=None, hi_=None, fresh=False):
        if fresh:
            lo_, hi_ = _ab()
        if lo_ is None:
            lo_, hi_ = _ab()
        r = api.skill('place_pads_closed_loop', tag=tag, stand=stand,
                      xb=xb, zu=zu,
                      lo=lo_.tolist(), hi=hi_.tolist(), box_ref=box_ref,
                      ncmd=ncmd, free=free, hold=hold_, out0=out0,
                      dur=place_dur, off=off, x_in=x_in, z_frac=z_frac,
                      pad_half=pad_half, box_move_tol=box_move_tol,
                      box_move_hard=box_move_hard)
        log.extend(r.get('log') or [])
        return r

    both_seated = False
    if seated_tol is not None and press_records:
        clr = (press_records[-1] or {}).get('clear_m') or {}
        c_l = clr.get('left')
        c_r = clr.get('right')
        if c_l is not None and c_r is not None:
            both_seated = (seated_min <= c_l <= seated_tol
                           and seated_min <= c_r <= seated_tol)

    o = api.observe()
    squeezed = False
    if (not both_seated) and o['hand_object_contacts'] < min_contacts:
        _run_place('squeeze_deep', stand_deep, 0.0, 0.0, squeeze_ncmd, False,
                   hold_=hold, out0=press_out0, fresh=True)
        squeezed = True
    o = api.observe()
    recovered = False
    if o['hand_object_contacts'] < empty_min:
        _retract_out(recover_out)
        lo_, hi_ = _ab()
        _run_place('recover_wide', recover_stand, 0.0, 0.0, recover_ncmd,
                   False, out0=place_out0, lo_=lo_, hi_=hi_)
        _run_place('recover_press', recover_press, 0.0, 0.0, recover_ncmd,
                   False, hold_=hold, out0=press_out0, lo_=lo_, hi_=hi_)
        o = api.observe()
        log.append({'phase': 'recover', 'base': o['base'],
                    'box': o['object']['pos'],
                    'contacts': o['hand_object_contacts'],
                    'sides': _sides()})
        recovered = True
    return {'log': log, 'squeezed': squeezed, 'recovered': recovered,
            'both_seated': both_seated,
            'contacts_end': o['hand_object_contacts'],
            'box_end': o['object']['pos']}
