def run(api, tag='place', stand=0.0, xb=0.0, zu=0.0, ncmd=2, free=True,
        hold=None, out0=None, dur=0.40, lo=None, hi=None, box_ref=None,
        x_in=0.03, z_frac=0.94, pad_half=0.0217,
        off=(0.076, -0.018, -0.030), box_move_tol=0.02,
        box_move_hard=0.25, jump_ratio=0.0, jump_abs=0.05,
        jump_eps=0.004, restore_mode='prev', restore_dy=0.08,
        restore_dur=0.35,
        branch='off', branch_iters=300, branch_ratio=3.0,
        branch_clear_tol=-0.04, branch_travel=0.25, branch_err_tol=0.0,
        branch_dq_tol=0.0, branch_switch_corr=1,
        branch_min_cmd=0, branch_alphas=(0.5, 0.25, 0.0),
        branch_fallback='refuse'):
    """Command both palm sites to a box face pair and correct from the
    ACHIEVED pad pose, exactly `place_pads_jumpguard` / the pinned closed
    loop, plus a PRE-COMMAND IK-branch screen.

    Removes: a closed-loop correction that is solved onto a different IK
    branch than the one it starts from, so the pad swings several times
    further than the correction it was asked for and is driven through a
    box face, shoving the box out from under the press.  The post-command
    guard in `place_pads_jumpguard` can only measure that AFTER the arm has
    moved and the box has been pushed; this skill asks the read-only
    `api.ik_preview(targets=...)` -- same multi-start seeds, same seed-0
    early break, same `iters` as the `reach_box` call it precedes -- where
    each pad will land and how far it will swing, and decides before the
    command is sent.

    Screen (per side, from the preview rows, no engine step, live state
    untouched).  A side is flagged when ANY of:
      * this command is a closed-loop correction (`ci > 0`; the first
        command of a step is `ci == 0`) and the winning seed is not 0,
        unless `branch_switch_corr=0`.  A correction target is
        `prev_cmd + (dpad - pad)`: a few centimetres from the pose the arm
        already holds, so a solution on another multi-start branch is a
        branch JUMP, not a local correction, and the servo's PATH (not its
        endpoint) is what sweeps the pad through the face.  At `ci == 0`
        the step is a whole waypoint move and a non-zero seed is the
        solver's normal choice.  Measured on the 22-seed `report` sweep:
        this fires on exactly ONE row of the whole sweep, the failing
        `reach_r0_ybef` command 2 (left pad, seed 2, dq 4.416 rad), while
        every other branch-1/2 row in the sweep is a command 1; that row's
        pad_travel (0.0487 m), ratio (1.345) and predicted clearance
        (+0.0491 m) all look benign.  This is the criterion the shipped
        candidate uses, with `branch_min_cmd=1` so `ci == 0` rows are not
        screened at all;
      * `travel_ratio` = predicted pad displacement from the LIVE pad
        position divided by the correction the previous command asked for
        (its pad error) exceeds `branch_ratio` (default 3.0, the same
        threshold the post-command guard uses; every benign multi-command
        row of the 22-seed baseline is at most 1.99, the failing one 9.82);
      * the predicted pad centre is through the live box face by more than
        `branch_clear_tol` metres (negative = inboard of the face; the
        predicted clearance uses the preview's box-centre-relative pad
        position plus the caller's AABB, i.e. the same formula the
        post-command `clear_m` uses);
      * the winning seed is not 0 AND the predicted pad travel exceeds
        `branch_travel` metres (a catastrophic swing, independent of the
        ask);
      * `branch_dq_tol > 0` and the predicted joint travel (`dq`, max abs
        over the 7 arm joints, radians) exceeds it (off by default).  This
        is the solver's PATH, not its endpoint.  It does NOT separate at
        the 22-seed level and must not be given a magic threshold: benign
        command-1 branch-2 rows reach 3.9446 rad against 4.4160 rad for the
        failing row (12% apart), because a first-command waypoint may
        legitimately need a big local swing.  Kept only as an experiment
        lever; `branch_switch_corr` is the criterion the candidate uses;
      * `branch_err_tol > 0` and the predicted site residual exceeds it
        (off by default: residuals above 0.05 m occur on passing seeds).

    Modes.  `branch='off'` (default) takes no preview at all and is
    bit-identical to `place_pads_jumpguard`.  `'report'` screens and then
    commands the original target anyway, recording the prediction in
    `record['branch']` for every command (the control run).  `'select'`
    re-aims: candidates are `prev_target + alpha*(undamped_target -
    prev_target)` for `alpha` in `branch_alphas`, where `undamped_target`
    is the closed-loop target that would cancel the achieved pad error
    exactly; the largest alpha whose preview is clean on both sides is
    commanded, otherwise `branch_fallback='mintravel'` picks the smallest
    predicted swing and `'refuse'` (default) commands nothing.  `'refuse'`
    never commands a flagged target: the step ends with `ok=False`,
    `record['branch_refuse']` set and `branch_refused=True`, so the box is
    not shoved and the caller can retry from a pose it can reach.

    Contract: pre: same as `place_pads_closed_loop` -> post: same, plus the
    per-command screen records; on a refusal no command is sent and the
    pads stay where the previous command left them.  Side effects: as
    `place_pads_jumpguard`; in `'report'`/`'select'`/`'refuse'` one
    `ik_preview` per commanded target (plus one per re-aim candidate) --
    IK solves on a scratch copy, no sim step.  Returns
    `{ok, record, log, contacts, sides, commands, jump_guard, jump_guards,
    branch_refused, branch_refuse}`.
    """
    OFF = np.array([off[0], off[1], off[2]])
    Rb = {'left': np.array([[1., 0, 0], [0, -1., 0], [0, 0, -1.]]),
          'right': np.array([[1., 0, 0], [0, 1., 0], [0, 0, 1.]])}
    b_ref = np.array(box_ref, dtype='float64')
    if out0 is None:
        out0 = 0.015

    def _box_pos():
        return np.array(api.observe()['object']['pos'])

    def _ab():
        gg = api.geom_detail()
        return np.array(gg['world_lo']), np.array(gg['world_hi'])

    def _sides():
        sr = api.hand_contacts_side()
        return {'left': sr.get('left'), 'right': sr.get('right')}

    def _clear(lo_, hi_, pad):
        out = {}
        pp = np.array(pad['left']['pos'])
        nn = np.array(pad['left']['normal'])
        out['left'] = np.round((pp[1] + nn[1] * pad_half) - hi_[1],
                               4).item()
        pp = np.array(pad['right']['pos'])
        nn = np.array(pad['right']['normal'])
        out['right'] = np.round(lo_[1] - (pp[1] + nn[1] * pad_half),
                                4).item()
        return out

    # desired pad centres from the measured box AABB
    pz = lo[2] + z_frac * (hi[2] - lo[2]) + zu
    px = lo[0] + x_in - xb
    dpad = {'left': np.array([px, hi[1] + pad_half + stand, pz]),
            'right': np.array([px, lo[1] - pad_half - stand, pz])}

    site = {}
    for s_ in dpad:
        site[s_] = np.array(dpad[s_], dtype='float64') - OFF
    site['left'] = site['left'] + np.array([0.0, out0, 0.0])
    site['right'] = site['right'] + np.array([0.0, -out0, 0.0])

    lo_ab = np.array(lo, dtype='float64')
    hi_ab = np.array(hi, dtype='float64')

    # ---- PRE-COMMAND branch screen (read-only) --------------------------
    def _screen(tg):
        pv = api.ik_preview(
            targets={s_: np.array(tg[s_]).tolist() for s_ in tg},
            R_body=Rb, iters=branch_iters, prefer_current=True,
            sides=('left', 'right'))
        cbx = np.array(pv.get('box') or [0.0, 0.0, 0.0])
        info = {}
        bad = {}
        for row in (pv.get('rows') or []):
            s_ = row.get('side')
            pw = None
            pc = None
            if row.get('pad') is not None:
                pw = cbx + np.array(row['pad']['pos'])
                nn = np.array(row['pad']['normal'])
                if s_ == 'left':
                    pc = (pw[1] + nn[1] * pad_half) - hi_ab[1]
                else:
                    pc = lo_ab[1] - (pw[1] + nn[1] * pad_half)
            why = []
            tv = row.get('pad_travel')
            er = row.get('err')
            dv = row.get('dq')
            e0 = None
            if err_prev is not None:
                e0 = err_prev.get(s_)
            cri = {}
            if (tv is not None and e0 is not None and e0 > jump_eps):
                cri['ratio'] = np.round(tv / e0, 3).item()
                if cri['ratio'] > branch_ratio:
                    why.append('ratio')
            # A closed-loop correction (ci > 0) that re-solves onto a
            # branch other than the current pose is a branch JUMP, not a
            # local correction: the target is prev_cmd + (dpad - pad).
            if branch_switch_corr and ci > 0 and row.get('seed') != 0:
                why.append('switch')
            if pc is not None and pc < branch_clear_tol:
                why.append('clear')
            if (row.get('seed') != 0 and tv is not None
                    and tv > branch_travel):
                why.append('travel')
            if branch_dq_tol > 0 and dv is not None and dv > branch_dq_tol:
                why.append('dq')
            if branch_err_tol > 0 and er is not None and er > branch_err_tol:
                why.append('unreached')
            info[s_] = {
                'seed': row.get('seed'), 'dq': row.get('dq'),
                'pad_travel': tv, 'err': er, 'rot': row.get('rot'),
                'ratio': cri.get('ratio'),
                'clear_pred_m': (None if pc is None
                                 else np.round(pc, 4).item()),
                'why': why}
            if why:
                bad[s_] = why
        return info, bad

    ok = True
    refused = False
    bref_rec = None
    ci = 0
    recs = []
    rec = {'phase': tag, 'commands': ci}
    mv0 = np.sum(np.abs(_box_pos() - b_ref))
    rec['box_from_ref_m'] = np.round(mv0, 4).item()
    site_prev = None
    pad_prev = None
    err_prev = None
    prev_cmd = None
    undamp = None
    q_pre = None
    jg = None
    guards = []
    while ci < ncmd:
        site_prev = {s_: np.array(site[s_]) for s_ in site}
        if jump_ratio > 0:
            q_pre = np.array(api.upper(), dtype='float64')
        binfo = None
        cmd_site = site
        if branch != 'off' and ci >= branch_min_cmd:
            inf0, bad0 = _screen(site)
            binfo = {'cmd': ci + 1, 'orig': inf0, 'action': 'command',
                     'chosen': 1.0, 'tried': []}
            if bad0:
                binfo['orig_bad'] = bad0
                if branch == 'report':
                    binfo['action'] = 'command_despite'
                elif branch == 'select':
                    clean = None
                    fall = None
                    fall_tr = None
                    for a in branch_alphas:
                        cand = {}
                        for s_ in site:
                            if prev_cmd is not None and undamp is not None:
                                cand[s_] = (prev_cmd[s_] + a
                                            * (undamp[s_] - prev_cmd[s_]))
                            else:
                                cand[s_] = np.array(site[s_])
                        infa, bada = _screen(cand)
                        mtr = 0.0
                        for s_ in infa:
                            tva = infa[s_].get('pad_travel')
                            if tva is not None:
                                mtr = np.maximum(mtr, tva)
                        binfo['tried'].append(
                            {'alpha': a, 'bad': bada,
                             'max_travel': np.round(mtr, 4).item()})
                        if not bada:
                            clean = (a, cand)
                            break
                        if fall_tr is None or mtr < fall_tr:
                            fall_tr = mtr
                            fall = (a, cand)
                    if clean is not None:
                        binfo['action'] = 'reaim'
                        binfo['chosen'] = clean[0]
                        cmd_site = clean[1]
                    elif branch_fallback == 'mintravel' and fall is not None:
                        binfo['action'] = 'reaim_mintravel'
                        binfo['chosen'] = fall[0]
                        cmd_site = fall[1]
                    else:
                        binfo['action'] = 'refuse'
                else:
                    binfo['action'] = 'refuse'
            if binfo['action'] == 'refuse':
                refused = True
                bref_rec = binfo
                rec = {'phase': tag, 'commands': ci,
                       'branch_refuse': binfo,
                       'box_from_ref_m': rec.get('box_from_ref_m')}
                recs.append(rec)
                ok = False
                break
            site_prev = cmd_site
        rr = api.reach_box(
            targets={s_: cmd_site[s_].tolist() for s_ in cmd_site},
            R_body=Rb, dur=dur, iters=300, hand_hold=hold)
        lo_, hi_ = _ab()
        pad = rr.get('pad') or {}
        oo = api.observe()
        mv = np.sum(np.abs(_box_pos() - b_ref))
        errn = {}
        for s_ in dpad:
            if pad.get(s_) is None:
                errn[s_] = None
            else:
                errn[s_] = np.round(np.sum(np.abs(
                    dpad[s_] - np.array(pad[s_]['pos']))), 4).item()
        rec = {'phase': tag, 'commands': ci + 1,
               'ik_err': rr.get('ik_err'), 'ik_rot': rr.get('ik_rot'),
               'ik_seed': rr.get('ik_seed'),
               'pad_err_l1_m': errn,
               'clear_m': (None if not pad else _clear(lo_, hi_, pad)),
               'base': oo['base'], 'box': oo['object']['pos'],
               'moved_l1_m': np.round(mv, 4).item(),
               'contacts': oo['hand_object_contacts'],
               'sides': _sides()}
        if binfo is not None:
            rec['branch'] = binfo
        if pad:
            for s_ in pad:
                if pad[s_] is not None:
                    rec['pad_' + s_] = {
                        'pos': pad[s_]['pos'],
                        'normal': pad[s_]['normal']}
        ci += 1
        recs.append(rec)

        # ---- pad-excursion guard: did this command move a pad far more
        # ---- than the previous command asked for?
        if (jump_ratio > 0 and site_prev is not None and pad
                and pad_prev is not None and err_prev is not None):
            jump = {}
            for s_ in dpad:
                p0 = pad_prev.get(s_)
                p1 = pad.get(s_)
                e0 = err_prev.get(s_)
                if p0 is None or p1 is None or e0 is None:
                    continue
                moved = np.sum(np.abs(np.array(p1['pos'])
                                      - np.array(p0['pos'])))
                if moved > jump_abs and e0 > jump_eps:
                    ratio = moved / e0
                    if ratio > jump_ratio:
                        jump[s_] = {
                            'pad_moved_l1_m': np.round(moved, 4).item(),
                            'pad_err_prev_m': np.round(e0, 4).item(),
                            'ratio': np.round(ratio, 3).item(),
                            'ik_seed': rr.get('ik_seed')}
            if jump:
                if restore_mode == 'qpos' and q_pre is not None:
                    api._ramp(q_pre, restore_dur)
                    qa = np.array(api.upper(), dtype='float64')
                    jg = {'jump': jump, 'restore_mode': 'qpos',
                          'restore_dur': restore_dur,
                          'restore_q_err': np.round(
                              np.max(np.abs(qa - q_pre)), 4).item()}
                else:
                    if restore_mode == 'out' and pad_prev is not None:
                        tgt = {}
                        for s_ in dpad:
                            sg = 1.0 if s_ == 'left' else -1.0
                            base = pad.get(s_)
                            if base is None:
                                base = pad_prev.get(s_)
                            if base is None:
                                continue
                            tgt[s_] = (np.array(base['pos']) - OFF
                                       + np.array([0.0, sg * restore_dy,
                                                   0.0])).tolist()
                    else:
                        tgt = {s_: site_prev[s_].tolist() for s_ in site_prev}
                    rr2 = api.reach_box(targets=tgt, R_body=Rb, dur=dur,
                                        iters=300, hand_hold=hold)
                    jg = {'jump': jump, 'restore_mode': restore_mode,
                          'restore_ik_err': rr2.get('ik_err'),
                          'restore_ik_seed': rr2.get('ik_seed')}
                rec['jump_guard'] = jg
                guards.append(jg)
                ok = False
                break

        if free and oo['hand_object_contacts'] > 0:
            ok = False
            break
        dmv = mv - mv0
        rec['dmv_step_m'] = np.round(dmv, 4).item()
        if dmv > box_move_tol or mv > box_move_hard:
            ok = False
            break
        if not pad:
            ok = False
            break
        pad_prev = pad
        err_prev = errn
        # the closed loop is anchored on the target that was COMMANDED
        prev_cmd = {}
        for s_ in site:
            prev_cmd[s_] = np.array(cmd_site[s_])
            site[s_] = np.array(cmd_site[s_])
            if pad.get(s_) is not None:
                site[s_] = site[s_] + (np.array(dpad[s_])
                                       - np.array(pad[s_]['pos']))
        undamp = {s_: np.array(site[s_]) for s_ in site}
    return {'ok': ok, 'record': rec, 'log': recs,
            'contacts': rec.get('contacts'),
            'sides': rec.get('sides'),
            'commands': rec.get('commands'),
            'jump_guard': jg, 'jump_guards': guards,
            'branch_refused': refused, 'branch_refuse': bref_rec}
