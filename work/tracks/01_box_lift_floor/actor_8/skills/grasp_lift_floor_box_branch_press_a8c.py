def run(api, **kw):
    """Whole-body skill: grasp a box on the floor between the opposed palm
    pads, close the fingers around it and stand up holding it.  Branch-aware
    variant of `grasp_lift_floor_box_jumpguard`: same composition, same
    read/command order, with the free-space placement moved into
    `place_pads_branch` so the IK branch of every commanded palm target is
    screened BEFORE the command is sent, and with ONE extra approach attempt
    granted only to a seed whose normal retry was refused by that screen.

    Composition (one env, one episode, depth 3):
      reach:  place_pads_branch waypoints x-before-y (free space;
              pad-excursion guard on, `jump_ratio` default 3.0, pre-command
              branch screen in `branch` mode)
              reach_attitude_guard on the y-before-face records
      press:  seat_pads_branch_a8 on the pinned placement (guard inert;
              branch screen on the press stream only when press_branch is
              'refuse'/'report')
      close:  close_fingers_flip_left
      grip:   secure_grip_after_close       (squeeze / recovery decision)
      lift:   stand_lift_box
    plus the base approach / high guard / deep crouch / settle, which are
    whole-body and stay in this skill.

    Why the refusal must be pre-command: on the failing seed the FIRST
    approach pass ends in a usable pad pose and is aborted only by the
    free-space contact check, which makes the retry the standard path for
    every seed.  The retry then re-solves a closed-loop target 0.0362 m from
    the pad, lands on IK seed 2 for the left arm and swings the pad 0.3554 m
    through the box face; the post-command guard can see that only after the
    box has been shoved.  Refusing the flagged command leaves the pads at the
    pose the previous command achieved and the box unshoved, so the extra
    attempt starts from a good box.

    Contract: pre: box at rest on the floor within reach of a standing
    stance at approach_x -> post: pads seated, fingers closed, body standing
    to lift_h with the box held; returns the full `log` (piecewise records),
    `box_ref`, `box_end`, `box_end_z_hi`, `contacts_close`, `sides_at_close`,
    `sides_end`, `reach_ok`, `reach_failed_at`, `retries`, `repairs`,
    `reach_attitude`, `attitude_retries`, `branch_mode`, `branch_refused`,
    `branch_refusals`, and the guard decisions `reach_stop`, `both_seated`,
    `squeezed`, `recovered`, `jump_guards`, `jump_restores`.
    Press screen (opt-in): `press_branch='refuse'` runs the SAME pre-command
    branch screen on the press command stream (via `seat_pads_branch_a8`), so
    a press closed-loop correction that re-solves onto another IK branch is
    refused before it is sent.  `press_branch='off'` (default) leaves the
    press path bit-identical to `grasp_lift_floor_box_branch`; a refusal ends
    the press step early (`press_failed_at`) and the composition continues to
    close/grip/lift from the pads' last commanded pose.  Press-screen
    thresholds default to the free-space `branch_*` values; `press_ratio`,
    `press_clear_tol`, `press_travel`, `press_err_tol`, `press_dq_tol`,
    `press_switch_corr`, `press_min_cmd`, `press_alphas` and
    `press_fallback` narrow the press screen without moving the reach screen.

    `branch='off'`, `repair_n=0`, and `att_retry_reasons` containing both
    guard reasons (all defaults or explicit) is bit-identical to
    `grasp_lift_floor_box_jumpguard`; `branch='report'` screens and commands
    the original targets, so it adds no engine step and no command and its
    trajectory equals the guarded baseline while recording the prediction for
    every command.  A refusal adds no engine read of its own and a repair
    attempt runs only for a seed that refused.  Never reads the engine
    success predicate and never changes physics or model limits.

    Guard drop gate (this variant): `reach_attitude_guard` is called only on
    a COMPLETED six-waypoint approach.  It returns `roll_away` when a pad's
    normal-z DROPS by more than `delta_ztol` during the approach -- the pose
    that lets the press slide up the face and over -- and `down_tilt` when the
    last normal-z merely sits below -`level_ztol`, a static level condition.
    Discarding the approach re-runs the whole reach from wide_hi, and that
    extra re-solve is what shoves the box (seeds 66, 74) -- but it is also
    what saves seed 60, whose pad normal dropped 0.088 over the approach.
    The gate therefore keys on the DROP, not on the label: a reason listed in
    `att_retry_reasons` always retries, and any other reason retries when
    -`dz` >= `att_tilt_dz` (default 0.05).  The four down_tilt fires measured
    over seeds 0-80 split as dz -0.010/-0.012/-0.030 (skip: s74/s33/s66) and
    -0.088 (retry: s60).  `att_retry_reasons` containing both reasons
    reproduces the pin exactly; `att_guard=0` disables the guard entirely.
    The returned log keys and the rest of the composition are unchanged; only
    the re-approach is skipped.
    """
    PAD_HALF = 0.0217
    OFF = (kw.get('off_x', 0.076), kw.get('off_y', -0.018),
           kw.get('off_z', -0.030))
    Rb = {'left': np.array([[1., 0, 0], [0, -1., 0], [0, 0, -1.]]),
          'right': np.array([[1., 0, 0], [0, 1., 0], [0, 0, 1.]])}
    log = []
    h = kw.get('h', 0.46)

    def _box_pos():
        return np.array(api.observe()['object']['pos'])

    def _ab():
        gg = api.geom_detail()
        return np.array(gg['world_lo']), np.array(gg['world_hi'])

    def _sides():
        sr = api.hand_contacts_side()
        return {'left': sr.get('left'), 'right': sr.get('right')}

    # ---- approach: walk to a standing stance short of the box ----------
    tx = kw.get('approach_x', 0.67)
    i = 0
    nmax = kw.get('approach_iters', 18)
    while i < nmax:
        st = api.observe()
        b = api.base_state()
        ex = tx - b['pos'][0]
        ey = st['object']['pos'][1] - b['pos'][1]
        eyaw = 0.0 - b['yaw_deg']
        if np.abs(ex) < 0.03 and np.abs(ey) < 0.03 and np.abs(eyaw) < 3.0:
            break
        api.move_base(x=np.clip(1.6 * ex, -0.16, 0.16),
                      y=np.clip(2.0 * ey, -0.16, 0.16),
                      yaw=np.clip(1.0 * eyaw * 0.0174533, -0.25, 0.25),
                      dur=0.35)
        i += 1
    o = api.observe()
    log.append({'phase': 'approach', 'base': o['base'],
                'box': o['object']['pos'],
                'contacts': o['hand_object_contacts']})

    # ---- high guard: hands up in front of the chest, clear of the box ---
    b0 = o['base']
    g = {'left': [b0[0] + 0.15, b0[1] + 0.14, b0[2] + 0.35],
         'right': [b0[0] + 0.15, b0[1] - 0.14, b0[2] + 0.35]}
    gr = api.reach_box(targets=g, orientation='none', dur=0.8, iters=200)
    o = api.observe()
    log.append({'phase': 'guard', 'ik_err': gr.get('ik_err'),
                'base': o['base'], 'box': o['object']['pos'],
                'contacts': o['hand_object_contacts']})

    # ---- deep crouch with base-x servo ----------------------------------
    q = api.commanded()
    if q is None:
        q = api.upper()
    h0 = api.observe()['base'][2]
    nsteps = np.maximum(1.0, np.ceil(kw.get('crouch_dur', 3.0) / 0.02))
    k = 0
    while k < nsteps:
        aa = (k + 1.0) / nsteps
        aa = aa * aa * (3.0 - 2.0 * aa)
        hc = h0 + (h - h0) * aa
        bx = api.observe()['base'][0]
        vx = np.clip(1.5 * (kw.get('crouch_xt', 0.665) - bx), -0.35, 0.35)
        api.step_goal(upper=q, nav=[vx, 0.0, 0.0], height=hc, dur=0.02)
        k += 1
    o = api.observe()
    log.append({'phase': 'crouch', 'h': h, 'base': o['base'],
                'tilt': o['base_tilt_deg'], 'box': o['object']['pos'],
                'contacts': o['hand_object_contacts']})

    # ---- settle, then take the box reference (never while it is airborne)
    b_ref = _box_pos()
    jj = 0
    nset = kw.get('settle_n', 10)
    while jj < nset:
        api.step_goal(upper=api.commanded(), height=h,
                      dur=kw.get('settle_dur', 0.05))
        b2 = _box_pos()
        dd = np.sum(np.abs(b2 - b_ref))
        if dd < kw.get('settle_tol', 0.002):
            b_ref = b2
            break
        b_ref = b2
        jj += 1
    lo0, hi0 = _ab()
    log.append({'phase': 'settle', 'n': jj, 'box': b_ref.tolist(),
                'lo': lo0.tolist(), 'hi': hi0.tolist(),
                'base': api.observe()['base']})

    # ---- geometry parameters (forwarded to the sub-skills) --------------
    x_in = kw.get('x_in', 0.03)
    z_frac = kw.get('z_frac', 0.94)
    y_wide = kw.get('y_wide', 0.10)
    x_back = kw.get('x_back', 0.05)
    z_up = kw.get('z_up', 0.06)
    stand_before = kw.get('stand_before', 0.030)
    tol = kw.get('box_move_tol', 0.02)
    place_dur = kw.get('place_dur', 0.40)
    move_hard = kw.get('box_move_hard', 0.25)
    p_out0 = kw.get('place_out0', 0.015)
    jump_ratio = kw.get('jump_ratio', 3.0)
    jump_abs = kw.get('jump_abs', 0.05)
    restore_mode = kw.get('restore_mode', 'prev')
    restore_dy = kw.get('restore_dy', 0.08)
    jguards = []
    brefs = []
    pr_out0 = kw.get('press_out0', 0.004)
    bref = b_ref.tolist()
    bro = kw.get('branch', 'off')
    biters = kw.get('branch_iters', 300)
    bratio = kw.get('branch_ratio', 3.0)
    bclear = kw.get('branch_clear_tol', -0.04)
    btravel = kw.get('branch_travel', 0.25)
    berr = kw.get('branch_err_tol', 0.0)
    bdq = kw.get('branch_dq_tol', 0.0)
    bsw = kw.get('branch_switch_corr', 1)
    bmc = kw.get('branch_min_cmd', 0)
    balpha = kw.get('branch_alphas', (0.5, 0.25, 0.0))
    bfall = kw.get('branch_fallback', 'refuse')
    # Press-screen overrides: each defaults to the free-space value, so the
    # press screen is identical to the reach screen unless a caller narrows
    # it.  A criterion can be disabled through its own parameter (ratio:
    # huge branch_ratio; switch: branch_switch_corr=0; travel: huge
    # branch_travel; dq/unreached: tol <= 0) without moving the reach screen.
    pratio = kw.get('press_ratio', bratio)
    pclear = kw.get('press_clear_tol', bclear)
    ptravel = kw.get('press_travel', btravel)
    perr = kw.get('press_err_tol', berr)
    pdq = kw.get('press_dq_tol', bdq)
    psw = kw.get('press_switch_corr', bsw)
    pmc = kw.get('press_min_cmd', bmc)
    palpha = kw.get('press_alphas', balpha)
    pfall = kw.get('press_fallback', bfall)

    def _retract_out(dy):
        gg = api.geom()
        tgt = {}
        for s_ in ('left', 'right'):
            sg = 1.0 if s_ == 'left' else -1.0
            tgt[s_] = (np.array(gg['sites'][s_]['pos'])
                       + np.array([0.0, sg * dy, 0.0])).tolist()
        api.reach_box(targets=tgt, R_body=Rb, dur=kw.get('retract_dur', 0.35),
                      iters=300)
        o = api.observe()
        log.append({'phase': 'retract_out', 'base': o['base'],
                    'box': o['object']['pos'],
                    'moved_l1_m': np.round(
                        np.sum(np.abs(_box_pos() - b_ref)), 4).item(),
                    'contacts': o['hand_object_contacts']})

    # ---- free-space approach: x first, then y ---------------------------
    def _run_place(tag, stand, xb, zu, ncmd, free, hold=None, out0=None,
                   lo_=None, hi_=None, fresh=False):
        if fresh:
            lo_, hi_ = _ab()
        if lo_ is None:
            lo_, hi_ = _ab()
        # Free-space steps run the branch-screened placement; the press path
        # stays on the pinned placement (seat_pads_reach_aware), so a press
        # command stream is never altered by the screen.
        r = api.skill('place_pads_branch', tag=tag, stand=stand,
                      xb=xb, zu=zu,
                      lo=lo_.tolist(), hi=hi_.tolist(), box_ref=bref,
                      ncmd=ncmd, free=free, hold=hold, out0=out0,
                      dur=place_dur, off=OFF, x_in=x_in, z_frac=z_frac,
                      pad_half=PAD_HALF, box_move_tol=tol,
                      box_move_hard=move_hard,
                      jump_ratio=(jump_ratio if free else 0.0),
                      jump_abs=jump_abs,
                      restore_mode=restore_mode,
                      restore_dy=restore_dy,
                      branch=(bro if free else 'off'),
                      branch_iters=biters, branch_ratio=bratio,
                      branch_clear_tol=bclear, branch_travel=btravel,
                      branch_err_tol=berr, branch_dq_tol=bdq,
                      branch_switch_corr=bsw, branch_min_cmd=bmc,
                      branch_alphas=balpha, branch_fallback=bfall)
        log.extend(r.get('log') or [])
        for gg in (r.get('jump_guards') or []):
            jguards.append(gg)
        for L in (r.get('log') or []):
            if L.get('branch_refuse') is not None:
                brefs.append({'tag': tag, 'cmd': L.get('commands'),
                              'why': L['branch_refuse'].get('orig_bad')})
        return r

    def _free_run(tag, wide, xb, zu, sb):
        lo_ = lo0
        hi_ = hi0
        r0 = _run_place(tag + '_wide_hi', kw.get('wide_hi', 0.19),
                        xb=xb, zu=zu + 0.04, ncmd=1, free=True,
                        out0=p_out0, lo_=lo_, hi_=hi_)
        if not r0.get('ok'):
            return False, 'wide_hi', (r0.get('log') or []), r0.get(
                'branch_refused')
        r1 = _run_place(tag + '_pre', wide, xb=xb, zu=zu, ncmd=1, free=True,
                        out0=p_out0, lo_=lo_, hi_=hi_)
        if not r1.get('ok'):
            return False, 'pre', (r1.get('log') or []), r1.get(
                'branch_refused')
        r2 = _run_place(tag + '_zdown', wide, xb=xb, zu=0.0, ncmd=1,
                        free=True, out0=p_out0, lo_=lo_, hi_=hi_)
        if not r2.get('ok'):
            return False, 'zdown', (r2.get('log') or []), r2.get(
                'branch_refused')
        r3 = _run_place(tag + '_xfwd1', wide, xb=0.5 * xb, zu=0.0, ncmd=1,
                        free=True, out0=p_out0, lo_=lo_, hi_=hi_)
        if not r3.get('ok'):
            return False, 'xfwd1', (r3.get('log') or []), r3.get(
                'branch_refused')
        r4 = _run_place(tag + '_xfwd2', wide, xb=0.0, zu=0.0, ncmd=1,
                        free=True, out0=p_out0, lo_=lo_, hi_=hi_)
        if not r4.get('ok'):
            return False, 'xfwd2', (r4.get('log') or []), r4.get(
                'branch_refused')
        r5 = _run_place(tag + '_ybef', sb, xb=0.0, zu=0.0, ncmd=2,
                        free=True, out0=p_out0, lo_=lo_, hi_=hi_)
        # Hand the last step's records back so the attitude guard can judge
        # the palm pose the press is about to build on.
        if not r5.get('ok'):
            return False, 'ybef', (r5.get('log') or []), r5.get(
                'branch_refused')
        return True, 'ok', (r5.get('log') or []), False

    ok, where, recs, refused = _free_run('reach', y_wide, x_back, z_up,
                                         stand_before)
    ra = None
    na = 0

    def _att_retry(r):
        # Discard a COMPLETED approach and re-run it from wide_hi only when the
        # guard saw a pad normal-z DROP over the approach: the motion that lets
        # the press slide up the face and over.  `roll_away` always drops by
        # more than delta_ztol; a `down_tilt` label is kept only when its own
        # dz shows a comparable drop (seed 60, -0.088), while the static-level
        # fires (seeds 33/66/74, -0.010..-0.030) are skipped.  A reason listed
        # in `att_retry_reasons` always retries, so listing both reasons
        # reproduces the pin; `att_guard=0` disables the guard entirely.
        if not kw.get('att_guard', 1):
            return False
        if not r or not r.get('retry'):
            return False
        if r.get('reason') in kw.get('att_retry_reasons', ['roll_away']):
            return True
        td = kw.get('att_tilt_dz', 0.05)
        if td is None:
            return False
        dz = r.get('dz')
        return dz is not None and (-dz) >= td

    if ok:
        # Guard: a free-space approach that made no contact is not aborted by
        # the step's own safety check, yet it can finish with a pad pitched
        # down.  That pose is what lets the press slide the pad up the face
        # and over the top.  Judge it from the records already in hand.
        ra = api.skill('reach_attitude_guard', records=recs,
                       delta_ztol=kw.get('att_delta_ztol', 0.10),
                       level_ztol=kw.get('att_level_ztol', 0.12))
        if _att_retry(ra):
            log.append({'phase': 'reach_attitude', 'attempt': 0,
                        'action': 'retry', 'why': ra})
            ok = False
            where = 'attitude'
            refused = False
            na += 1
    nr = 0
    while not ok and nr < kw.get('retry_n', 1):
        _retract_out(kw.get('retract_out_dy', 0.05))
        lo0, hi0 = _ab()
        y_wide = y_wide + kw.get('retry_wide_add', 0.03)
        x_back = x_back + kw.get('retry_back_add', 0.02)
        ok, where, recs, refused = _free_run('reach_r%d' % nr, y_wide,
                                             x_back, z_up, stand_before)
        nr += 1
        ra = None
        if ok:
            ra = api.skill('reach_attitude_guard', records=recs,
                           delta_ztol=kw.get('att_delta_ztol', 0.10),
                           level_ztol=kw.get('att_level_ztol', 0.12))
            if _att_retry(ra):
                log.append({'phase': 'reach_attitude', 'attempt': nr,
                            'action': 'retry', 'why': ra})
                ok = False
                where = 'attitude'
                refused = False
                na += 1
    # ---- repair: only a seed whose retry was REFUSED gets another attempt;
    # ---- every other seed keeps the guarded baseline geometry.
    npair = 0
    while refused and not ok and npair < kw.get('repair_n', 0):
        _retract_out(kw.get('retract_out_dy', 0.05))
        lo0, hi0 = _ab()
        y_wide = y_wide + kw.get('repair_wide_add', 0.0)
        x_back = x_back + kw.get('repair_back_add', 0.02)
        ok, where, recs, refused = _free_run('reach_p%d' % npair, y_wide,
                                             x_back, z_up, stand_before)
        npair += 1
        ra = None
        if ok:
            ra = api.skill('reach_attitude_guard', records=recs,
                           delta_ztol=kw.get('att_delta_ztol', 0.10),
                           level_ztol=kw.get('att_level_ztol', 0.12))
            if _att_retry(ra):
                log.append({'phase': 'reach_attitude', 'attempt': nr + npair,
                            'action': 'retry', 'why': ra})
                ok = False
                where = 'attitude'
                refused = False
                na += 1
    log.append({'phase': 'reach_done', 'ok': ok, 'failed_at': where,
                'retries': nr, 'repairs': npair, 'y_wide': y_wide,
                'x_back': x_back, 'attitude': ra, 'attitude_retries': na,
                'branch_mode': bro, 'branch_refused': refused,
                'branch_refusals': brefs})

    # ---- press: seat both pads (sub-skill, default 4 commands/step) -----
    # The reach guard is inert at its default 0 thresholds; the run that
    # enables it passes ik_reach_tol / clear_unreached_tol / other_seat_tol.
    # The press command stream can carry the SAME pre-command branch screen
    # as the free-space placement.  It is OPT-IN: `press_branch` defaults to
    # 'off', so at the shipped kwargs this composition sends exactly the
    # pinned press commands (bit for bit with `grasp_lift_floor_box_branch`).
    # Pass `press_branch='refuse'` to refuse a press command whose
    # closed-loop correction re-solves onto another IK branch before it is
    # sent; `press_branch='report'` predicts without refusing.
    pbro = kw.get('press_branch', 'off')
    rs = api.skill('seat_pads_branch_a8', box_ref=bref,
                   press_steps=kw.get('press_steps', [0.012, 0.003, -0.006]),
                   press_ncmd=kw.get('press_ncmd', 4), press_out0=pr_out0,
                   off=OFF, x_in=x_in, z_frac=z_frac, pad_half=PAD_HALF,
                   box_move_tol=tol, box_move_hard=move_hard,
                   place_dur=place_dur,
                   ik_reach_tol=kw.get('ik_reach_tol', 0.0),
                   clear_unreached_tol=kw.get('clear_unreached_tol', 0.0),
                   other_seat_tol=kw.get('other_seat_tol', 0.0),
                   branch=pbro, branch_iters=biters, branch_ratio=pratio,
                   branch_clear_tol=pclear, branch_travel=ptravel,
                   branch_err_tol=perr, branch_dq_tol=pdq,
                   branch_switch_corr=psw, branch_min_cmd=pmc,
                   branch_alphas=palpha, branch_fallback=pfall)
    log.extend(rs.get('records') or [])
    for gg in (rs.get('branch_refusals') or []):
        brefs.append(gg)

    # ---- close the fingers (left index/middle flipped) -----------------
    ca = kw.get('close_angle', 0.9)
    qc_h = None
    cc = None
    cs = None
    if ca:
        rc = api.skill('close_fingers_flip_left', angle=ca,
                       dur=kw.get('close_dur', 0.8),
                       left_finger_flip=kw.get('left_finger_flip', 1),
                       rthumb_min=kw.get('rthumb_min', 0.30),
                       box_ref=bref)
        qc_h = np.array(rc.get('target'), dtype='float64')
        cc = rc.get('contacts')
        cs = rc.get('sides')
        log.append(rc.get('record'))

    # ---- deepen the squeeze only if the grip is thin AND the press did ---
    # ---- not already leave both pads seated; one recovery if empty ------
    rg = api.skill('secure_grip_after_close', box_ref=bref, hold=qc_h,
                   press_records=rs.get('records') or [],
                   min_contacts=kw.get('min_contacts', 4),
                   empty_min=kw.get('empty_min', 2),
                   seated_tol=kw.get('seated_tol', None),
                   seated_min=kw.get('seated_min', -0.02),
                   stand_deep=kw.get('stand_deep', -0.013),
                   squeeze_ncmd=kw.get('squeeze_ncmd', 2),
                   press_out0=pr_out0, place_out0=p_out0,
                   recover_out=kw.get('recover_out', 0.03),
                   recover_stand=kw.get('recover_stand', 0.035),
                   recover_press=kw.get('recover_press', -0.004), off=OFF,
                   x_in=x_in, z_frac=z_frac, pad_half=PAD_HALF,
                   box_move_tol=tol, box_move_hard=move_hard,
                   place_dur=place_dur,
                   retract_dur=kw.get('retract_dur', 0.35))
    log.extend(rg.get('log') or [])

    # ---- stand lift holding the last commanded pose ---------------------
    if kw.get('stand_lift', 1):
        rl = api.skill('stand_lift_box', lift_h=kw.get('lift_h', 0.68),
                       lift_dur=kw.get('lift_dur', 2.5))
        log.extend(rl.get('samples') or [])

    o = api.observe()
    g3 = api.geom_detail()
    return {'log': log, 'final': o, 'base': api.base_state(),
            'reach_ok': ok, 'reach_failed_at': where, 'retries': nr,
            'repairs': npair, 'branch_mode': bro,
            'branch_refused': refused, 'branch_refusals': brefs,
            'box_ref': b_ref.tolist(),
            'box_end': o['object']['pos'], 'box_end_z_hi': g3['world_hi'][2],
            'contacts_close': cc, 'sides_at_close': cs,
            'sides_end': _sides(),
            'reach_stop': rs.get('reach_stop'),
            'press_failed_at': rs.get('failed_at'),
            'reach_attitude': ra, 'attitude_retries': na,
            'jump_guards': jguards,
            'jump_restores': np.shape(jguards)[0],
            'both_seated': rg.get('both_seated'),
            'squeezed': rg.get('squeezed'),
            'recovered': rg.get('recovered')}
