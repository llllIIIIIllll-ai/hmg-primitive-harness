# pp31 (2026-11-10, actor_5): pp27 at sha 75758bfe... plus ONE knob pair that
# makes the pre-turn retreat's lateral command a ONE-SHOT FORK on the base y
# measured at the retreat start:
#   pre_lat_y (m, default -8.0 = OFF)  deadband on base y at the first retreat
#                                      iteration; base_y > pre_lat_y selects
#                                      pre_lat_v for EVERY remaining retreat row
#   pre_lat_v (m/s, default 0.0)       lateral (world -y at yaw 0) command
#                                      used when the deadband is crossed
# The decision is latched from the loop's EXISTING per-iteration api.state()
# read, so the read count and order are byte-identical to pp27.  At the
# defaults (pre_lat_y=-8.0) the command stream is byte-identical to pp27.
#
# pp25 (2026-11-10, actor_5): pp21_a3 at sha 6df4bae8... plus ONE knob pair
# that parameterises the body descent into the lowers transition:
#   drop_ramp (int, default 1)  number of set_height targets used to travel
#                               from walk_h down to low_h
#   drop_dur  (s,  default 1.1) total commanded descent time
# At the defaults (drop_ramp=1, drop_dur=1.1) the branch below is
# byte-for-byte the pinned call api.set_height(low_h, dur=1.1), so every
# other phase (nudges, lowers, release) is untouched and a run at the
# defaults must reproduce the pin bit-identically.
def run(api, h=0.55, grip=0.9, yL=0.19, yR=-0.19, z=0.765, wz=-1.0,
        carry_h=0.66, low_h=0.55, walk_h=0.66, drop='set',
        drop_ramp=1, drop_dur=1.1,
        fm_stop=0.010, fm_gate=0.004,
        vx=0.20, vy_walk=0.0, contact_abort=1500.0, kmax=16, stall_n=2,
        pre_x=0.50, pre_vy=0.0, pre_lat_y=-8.0, pre_lat_v=0.0,
        n_ret=16, post_x=0.70, n_str=5, str_v=0.12,
        str_vx=0.0, str_yaw_tol=0.0, tuck=0.0, tuck_x=0.0, tuck_h=0.66,
        con_log=0,
        low_vx=0.10, n_low=10, lows_n=14,
        nudge_n=4, nudge_v=0.20, nudge_dur=0.35,
        push_n=0, push_dy=-0.005, push_z=0.0, push_fm_stop=0.020,
        push_dur=0.4, push_settle=0.25,
        drop_fm_stop=-9.0, lows_gap=0.008,
        low_dy=-0.004, low_dy_fm=0.008, low_dz=None, lows_fm_stop=9.0,
        turn_yaw=-90.0, yaw_guard=2.5,
        turn_pred=0, turn_pred_margin=3.0,
        low_dy_rows=None, low_shr=0.0, low_shr_rows=None,
        low_spread=0.0, low_spread_rows=None, low_yaw_hold=0,
        low_dx=0.0, low_dx_rows=None,
        low_yaw_hold_rows=None, lean_rows=None,
        yaw_n=0, yaw_tgt=90.0, yaw_tol=1.0, yaw_k=0.0006, yaw_shr_max=0.012,
        yaw_spr=-0.004, yaw_dz=0.0,
        # rel_mode default switched 'hold' -> 'settle' after the settle arm
        # (resp_settle_s0/_s4) produced a complete board placement on seed 0
        # (gap 0.0002, up 1.0, hb 0) where 'hold' left the box pinched 2.1 cm
        # aloft.  'hold' stays selectable for A/B;
        # see the release-skill block below.
        rel_mode='settle', rel_open=0.45, rel_dur=0.4, rel_hold=0.15,
        rel_open_x=0.02, rel_dx=0.0, rel_gain=1.0, rel_n=2,
        rel_gap_tol=0.003, rel_grasp_min=0.5, rel_ret=0.05,
        rel_pre=0.0, **kw):
    # pp21 (2026-11-10, actor_3): pp20 + closed-loop yaw regulation at low
    # height.  pp20's identification run (resp_id1_s0.json, seed 0) measured,
    # with the grasp held and ZERO palm command, a box yaw drift of +0.9 to
    # +1.55 deg/row (104.24 -> 111.66 while the palms were only held) and a
    # matching shear of the palm sites themselves (left +y 5-11 mm/row, right
    # -y 4-6 mm/row): hy = 0.0975|R10|+0.1405|R11| therefore grew 0.1290 ->
    # 0.1438 and ate the slide's y gain.  Counter-shear works: shr=-0.004 per
    # row gave -0.11..-0.20 deg/row (achieved shear ~0), shr=+0.004 gave
    # +0.92..+1.22 (same as baseline, i.e. +side saturates), and a 4 mm pinch
    # spr=-0.004 (cell D) damped the drift to +0.07..+0.30 deg/row.  So yaw
    # is controllable and the 22-deg overshoot is worth up to 4.5 cm of
    # front_margin (0.1438 -> 0.0975 hy).
    #  yaw_n/yaw_tgt/yaw_tol: closed-loop phase at low height, zero
    #    translation, runs until yaw <= yaw_tgt+yaw_tol or yaw_n rows.
    #  yaw_k: shr = -yaw_k*(yaw-yaw_tgt) metres per row, clamped to
    #    +-yaw_shr_max.  yaw_spr: constant pinch in the same rows.
    #  yaw_dz: vertical trim in the same rows (default 0: pp20 showed dz=0
    #    keeps the board gap at 0.001-0.006, i.e. no climb).
    #  low_yaw_hold: reuse the same closed loop inside the lows slide loop;
    #    when 1 it OVERRIDES low_shr/low_shr_rows with
    #    shr = -clip(yaw_k*(yaw-yaw_tgt), +-yaw_shr_max), so the forward slide
    #    runs with yaw held near yaw_tgt instead of self-drift.
    # MEASURED (resp_yaw1_s0.json, seed 0, drop='set', nudge_n=4, yaw_n=16,
    # yaw_tgt=90, yaw_k=0.0010): yaw 102.77 -> 93.04 monotone (dyaw -1.00 ->
    # -0.05/row), hy 0.1261 -> 0.1048, fm -0.0959 -> -0.0596 with ZERO
    # translation; gap only 0.0062 -> 0.0142; box held (hb 7-8, up 0.9977);
    # after release yaw 89.66, hy 0.0983, fm -0.0421, gap 0.0002, xm 0.0669
    # vs the pp18 s0 baseline fm -0.0797 (net +3.76 cm, pure yaw).
    # All default off (yaw_n=0, low_yaw_hold=0), so defaults reproduce pp20.
    # pp20 (2026-11-10, actor_3): pp19 + identification channels.  pp19's
    # 16-row slide (resp_slide16_s0123.json) showed the box translating 2.9-4.7
    # cm in -y from the palms at zero base motion, but hy = 0.0975|R10| +
    # 0.1405|R11| grew 12-32 mm over the same rows, cancelling all of the gain
    # on s2/s3 and half on s0/s1.  So the slide has an uncontrolled yaw term.
    # This template adds two orthogonal palm channels and a row schedule so one
    # episode can identify them:
    #  low_dy_rows: per-row common-mode dy (list; last value extends).  dy=0
    #    rows isolate the rotational channels from the translation.
    #  low_shr / low_shr_rows: differential y (left +shr, right -shr) = grasp
    #    line rotation about z; positive shr should increase yaw
    #    (atan2(R10,R00), deg).
    #  low_spread / low_spread_rows: differential x (left +spread, right
    #    -spread) = palm separation; positive = wider = less squeeze (the box
    #    climbed 3-5 cm while sliding, so squeeze may be the climb source).
    #  low_dx / low_dx_rows (pp21, added 2026-11-10): COMMON-MODE x on both
    #    palms (left x + dx, right x + dx).  The robot faces -y, so its left
    #    hand is on world +x and its right hand on world -x; a NEGATIVE dx
    #    therefore pushes both palms toward world -x, which is the direction
    #    that raises the board x_margin (the box has to move ~-0.023 m in x
    #    for seed 2 because hx ~0.148 at yaw 85-90).  spread/shear are
    #    differential and cannot translate the box; this is the only common-
    #    mode x channel.  Scope assumed: the pinch friction carries the box
    #    with the palms in x roughly 1:1 while the box is held; measured
    #    carry ratio is the deciding number (see out['lowers'] cmd_dx/dx).
    # All default off, so defaults reproduce pp19 row for row.
    # pp17 (2026-11-09, actor_2): pp16 + the pp16 batch's measured mechanisms.
    # pp18 (2026-11-09, actor_2): pp17 + four knobs so one template spans a
    # grid.  pp16/pp17 evidence says the walk is capped by leg->cabinet contact
    # at base y ~-0.355 while the box is still 1-8 cm short of the board, and
    # the body drop then costs another 7-10 cm of front_margin.  The remaining
    # lever is the ARM: the box is 10 g and hangs 0.34 m ahead of the base, so
    # translating the palms forward at carry height drags it over the edge.
    #  push_n/push_dy/push_z/push_fm_stop: reach_box push at walk_h before any
    #    body motion (logged as out['pushes'], with the box's measured dy/dx).
    #  low_dy/low_dy_fm: the same push inside the descent servo.
    #  drop_fm_stop: 'walk' descent stops early once fm reaches it (sentinel
    #    -9.0 = off); lows_gap: descent terminates on this board gap.
    #  turn_yaw/yaw_guard: approach yaw (default -90).
    #  Evidence out (16 episodes, seeds 0-3, all on the engine summary):
    #  - v1/v2/v3 (fm_stop .010/.030/.050) were BYTE-IDENTICAL: the walk always
    #    ended on 'contact' or 'stalled' first, so fm_stop never binds.
    #  - the walk saturates at base y -0.355 with leg->cabinet contact
    #    (s0 shelf_cabinet_main vs robot0_left_hip_yaw_link/robot0_g3_col
    #    449.49 N at base [0.545,-0.357]; s3 vs robot0_g12_col 794.05 N at
    #    [0.601,-0.355]) and s2 stalls contact-free at base y -0.315.
    #  - THE DROP IS THE LOSS: body 0.66->0.55 moves the pelvis back 0.096 m
    #    and the held box back 0.077 m (s0 fm -0.010 -> -0.0876; s3
    #    -0.0227 -> -0.1049).  The 3 low pushes then move the base y by
    #    -0.001/0.000/-0.003 m per 0.25 s burst = ~6% velocity tracking, i.e.
    #    the squat cannot recover the shift.  Low walking is dead.
    #  - release slide is real: pre_release -> final_b fm +0.026 (s0),
    #    +0.063 (s2), +0.068 (s3); the box rotates 100->87-90 deg and slides
    #    deeper.  Never gate the release on fm (0/16 passed the gate).
    #  - s1: retreat tracks 1 cm/burst backwards (0.677->0.586 in 10), the
    #    turn then adds +0.19 m x, yawfix +0.027, so base x ends 0.759-0.796
    #    and the pelvis edge (~+0.058) reaches the table slab at 0.8487
    #    (table_main vs robot0_pelvis/robot0_g0_col 61.53 N).  Fix = retreat
    #    target pre_x with n_ret bursts + post-turn lateral trim to post_x.
    # New mechanisms this candidate:
    #  drop='none': body never lowers; the arm 'lowers' servo does the ~0.09 m
    #    box descent at carry height (reach_box dz, hand_hold=carry).
    #  drop='walk': lower-walk - descend carry_h->low_h over n_low step_goal
    #    quanta while commanding body +x nav, so the same motion that lowers
    #    the box carries it over the board edge (stepping still works while
    #    the squat is shallow; the static drop's 0.096 m back-shift is
    #    cancelled instead of paid).
    #  drop='set': pp16's static drop (control).
    #  box<->board contacts are no longer walk-abort triggers (only robot
    #    link contacts are), they are logged as boxenv.
    # ---------------------------------------------------------- release skill
    #  NAME: box_place_settle  (parameterised dexterous place / release)
    #  Addresses the failure signature: the box is held in a two-palm pinch
    #  with its bottom 0.5-3 cm above the board (board_gap > 0.003), a pure
    #  finger-open release drops and spins it (see below), so the box either
    #  lands on a corner or never reaches the board.
    #  MECHANISM (measured): with the pinch still closed, command each palm
    #  DOWN by the box's measured board_gap (dzc = -rel_gain*gap, per row).
    #  The pinch friction carries the box down ~1:1 with the palms (seed 0:
    #  cmd_dz -0.0202 -> achieved box dz -0.020) until board_gap <= rel_gap_tol;
    #  then pull both palms +rel_ret in y to clear the box.
    #  Signature (named params): rel_mode='settle', rel_open=0.45,
    #    rel_dur=0.4, rel_hold=0.15, rel_open_x=0.02, rel_dx=0.0,
    #    rel_gain=1.0, rel_n=2, rel_gap_tol=0.003, rel_ret=0.05.
    #    rel_dx (added 2026-11-10) is a COMMON-MODE x offset on both palm
    #    targets applied during the open step, the seat rows and the retract;
    #    it sets the palm-associated x at which the box settles (default 0.0
    #    keeps the proven settle arm unchanged).  MEASURED response in
    #    rel_dx (seeds 0-3; the pre-release poses are bit-identical at every
    #    value, so the whole effect is inside the release): the placed box
    #    centre moves -6/-11/-6/-20 mm in x at -0.010 and the release yaw
    #    kick is pulled toward 90, so x_margin gains +11..+35 mm through both
    #    terms (hx shrinks with the yaw and the centre shifts -x).  Beyond
    #    -0.010 the gain keeps rising with diminishing returns, and yaw sits
    #    closest to 90 at -0.020: xm 0.1433/0.015/0.0443/0.119 (-0.010),
    #    0.1585/0.0252/0.0601/0.1285 (-0.020), 0.1621/0.0333/0.066/0.138
    #    (-0.030), every value 4/4 on the board with gap 0.0002, up 1.0,
    #    hb 0; |final yaw - 90| at -0.020 is 0.57/0.17/0.8/1.2 deg.  No
    #    optimum seen up to -0.030; -0.020 is the balanced choice (best min
    #    fm and yaw), -0.030 buys ~+8 mm on the tightest seed at a small bm
    #    cost.
    #  Preconditions: pinch grasp held (hb > 0), box roughly flat (up >= 0.99),
    #    box y-extent inside the board (front_margin > 0 for a clean place).
    #  Postconditions: board_gap <= 0.005, up >= 0.99, hb 0, both palms clear.
    #  Side effects: the release drags the box +x as contact relaxes (+23..28 mm
    #    during the open step, +6..18 mm during the retract, measured seeds 0-3
    #    at rel_dx 0) and its yaw kick is seed-dependent in sign: at rel_dx 0
    #    measured from the pre-release yaw (last_lower last_yaw) to final_b,
    #    +7.0 (seed 0), -7.8 (seed 1), -11.5 (seed 2), +8.6 (seed 3) deg --
    #    not a fixed -8 deg cost.  rel_dx=-0.010 makes every kick negative
    #    (-2.1/-5.0/-7.7/-3.6 deg) and cuts |final yaw - 90| to 0.3-3.8 deg;
    #    at -0.020 every seed lands within 1.2 deg of 90.  At -0.030 seed 1
    #    overshoots to 93.8 (and needs one seat row), the first sign of a
    #    cost to larger trims.
    #    dtype/units: metres, deg.
    #  Scope believed covered: box mass <= 0.01 kg, board_gap 0..0.03 m,
    #    low-height two-palm pinch, seeds 0-3 of 07 (all four place flat and
    #    inside the board +x edge over rel_dx 0..-0.030: gap 0.0002, up 1.0,
    #    hb 0 at every value tested; at the recommended rel_dx=-0.020
    #    xm +0.1585/+0.0252/+0.0601/+0.1285, fm +0.1013/+0.0803/+0.0347/
    #    +0.0977, bm +0.1353/+0.1583/+0.2007/+0.1357, yaw 90.57/90.17/
    #    89.2/91.2, t 56.0-59.3 s; 0-1 seat rows, i.e. the 0.45 s open alone
    #    seats a box <=1.5 cm above the board and the seat rows only matter
    #    above that; 0 seat rows on all seeds at -0.010 and -0.020).
    #    NOT covered: any other seed/task; box mass > 0.01 kg; rel_dx beyond
    #    -0.030 (untested); board_gap > 0.03 m; a non-flat box at release.
    #  MEASURED (resp_settle_s0.json / resp_settle_s4.json):
    #    seed 0: 1 seat row cmd_dz -0.0202 -> dz -0.020, gap 0.0002, up 1.0,
    #      hb 0; final xm +0.0969.  Seed 0 repeated identically in settle_s4.
    #    seed 2: 0 rows needed (fingers-open drop seated it), gap 0.0002,
    #      up 1.0, hb 0, xm -0.0246.  seed 3: gap 0.0002, up 1.0, hb 0,
    #      xm -0.0070.
    #    rel_open=0.0 (open inside seat row 0): seeds 2/3 xm -0.0070/+0.0122
    #      but seed 0 degraded (gap 0.0072, hb 3, fm -0.0121, yaw +18 deg):
    #      do not merge the open into a 2.7 cm descent.
    #    rel_dx=-0.010, full 07 arm (resp_reldx10_s0123.json, seeds 0-3):
    #      0 seat rows; final obj [0.707,-0.802,0.741] / [0.836,-0.791,0.741] /
    #      [0.801,-0.742,0.741] / [0.731,-0.802,0.741].  The pre-release poses
    #      are bit-identical to the rel_dx 0 arm (pre fm 0.085/0.0294/0.0024/
    #      0.0782, gap 0.0142/0.0122/0.0122/0.0092, xm 0.1627/0.0249/0.0665/
    #      0.1408), so the whole x gain is produced inside the release; at
    #      rel_dx 0 those same pre-release poses end xm 0.111/-0.0006/0.0328/
    #      0.0841 (resp_walkvy06_dx9_s0123.json, 3/4).  Relative to a rel_dx 0
    #      arm the only dynamics edits are the five rel_dx offsets: the
    #      open-step reach_box targets, both seat-row x targets and both
    #      retract x targets.
    #    rel_dx=-0.020 / -0.030 (resp_reldx20_s0123.json /
    #      resp_reldx30_s0123.json, seeds 0-3): 4/4 PASS at both; final obj x
    #      0.692/0.826/0.79/0.721 and 0.688/0.812/0.782/0.712; fm
    #      0.1013/0.0803/0.0347/0.0977 and 0.1037/0.0676/0.0306/0.0976; bm
    #      0.1353/0.1583/0.2007/0.1357 and 0.1317/0.1536/0.1986/0.1376.  At
    #      -0.030 seed 1 takes one seat row (open gap 0.0042 > tol 0.003)
    #      and ends yaw 93.78; nothing else differs from 0 seat rows.
    I = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    RB = {'left': I, 'right': I}
    out = {}
    out['knobs'] = {'walk_h': walk_h, 'drop': drop, 'vx': vx,
                    'vy_walk': vy_walk, 'contact_abort': contact_abort,
                    'kmax': kmax, 'stall_n': stall_n, 'pre_x': pre_x,
                    'pre_vy': pre_vy,
                    'pre_lat_y': pre_lat_y, 'pre_lat_v': pre_lat_v,
                    'n_ret': n_ret, 'post_x': post_x, 'n_str': n_str,
                    'str_v': str_v, 'str_vx': str_vx, 'tuck': tuck,
                    'tuck_x': tuck_x, 'str_yaw_tol': str_yaw_tol,
                    'con_log': con_log,
                    'low_vx': low_vx, 'n_low': n_low, 'lows_n': lows_n,
                    'push_n': push_n, 'push_dy': push_dy, 'push_z': push_z,
                    'push_fm_stop': push_fm_stop, 'drop_fm_stop': drop_fm_stop,
                    'low_dy': low_dy, 'low_dy_fm': low_dy_fm,
                    'low_dz': low_dz, 'lows_fm_stop': lows_fm_stop,
                    'turn_yaw': turn_yaw, 'low_h': low_h,
                    'low_shr': low_shr, 'low_spread': low_spread,
                    'low_dx': low_dx, 'low_dx_rows': low_dx_rows,
                    'yaw_n': yaw_n, 'yaw_tgt': yaw_tgt, 'yaw_k': yaw_k,
                    'yaw_shr_max': yaw_shr_max, 'yaw_spr': yaw_spr,
                    'yaw_dz': yaw_dz, 'low_yaw_hold': low_yaw_hold,
                    'low_yaw_hold_rows': low_yaw_hold_rows,
                    'lean_rows': lean_rows,
                    'rel_mode': rel_mode, 'rel_open': rel_open,
                    'rel_dur': rel_dur, 'rel_hold': rel_hold,
                    'rel_open_x': rel_open_x, 'rel_dx': rel_dx,
                    'rel_gain': rel_gain,
                    'rel_n': rel_n, 'rel_gap_tol': rel_gap_tol,
                    'rel_ret': rel_ret, 'rel_grasp_min': rel_grasp_min}
    body_end = low_h
    if drop == 'none':
        body_end = walk_h

    # ---------------------------------------------------------------- grasp
    api.reach_box(targets={'left': [0.80, 0.24, 0.98],
                           'right': [0.80, -0.30, 0.98]},
                  sides=('left', 'right'), orientation='keep', dur=1.1)
    api.set_height(h, dur=2.0)
    st = api.state()
    out['crouch'] = {'base': st['base'], 'obj': st['object'],
                     'tilt': st['base_tilt_deg']}

    def pads():
        hg = api.hand_geoms()
        res = {}
        for kk in hg:
            if kk.endswith('_palm_link'):
                side = 'left' if '_left_' in kk else 'right'
                for g in hg[kk]['geoms']:
                    if g['name'].endswith('g0_col'):
                        res[side] = g['world_pos']
        return res

    def handbox():
        n = 0
        fsum = 0.0
        for c in api.contact_detail(limit=100, match='box'):
            if c['g1'].startswith('gripper0') or c['g2'].startswith('gripper0'):
                n = n + 1
                f = c['force']
                if f:
                    f0 = f[0]
                    if f0 < 0:
                        f0 = -f0
                    fsum = fsum + f0
        return {'n': n, 'f': fsum}

    def envc():
        res = []
        for c in api.contact_detail(limit=100):
            s = c['b1'] + ' ' + c['b2'] + ' ' + c['g1'] + ' ' + c['g2']
            hit = 0
            if 'table' in s:
                hit = 1
            if 'shelf' in s:
                hit = 1
            if 'cabinet' in s:
                hit = 1
            if hit:
                f = 0.0
                if c['force']:
                    f = c['force'][0]
                    if f < 0:
                        f = -f
                if f > 0.5:
                    res.append({'b1': c['b1'], 'b2': c['b2'],
                                'g2': c['g2'], 'f': np.round(f, 2).item()})
        return res

    def snap():
        st = api.state()
        g = api.geom()['object']
        R = np.asarray(g['xmat'])
        r10 = R[1][0]
        if r10 < 0:
            r10 = -r10
        r11 = R[1][1]
        if r11 < 0:
            r11 = -r11
        hy = 0.0975 * r10 + 0.1405 * r11
        hx = 0.0975 * (R[0][0] if R[0][0] >= 0 else -R[0][0]) \
            + 0.1405 * (R[0][1] if R[0][1] >= 0 else -R[0][1])
        by = st['object']['pos'][1]
        bx = st['object']['pos'][0]
        xm1 = 0.992 - (bx + hx)
        xm2 = (bx - hx) - 0.108
        if xm2 < xm1:
            xm1 = xm2
        return {'obj': st['object']['pos'], 'up': np.round(g['z_up'], 4).item(),
                'half_y': np.round(hy, 4).item(),
                'yaw': np.round(np.arctan2(R[1][0], R[0][0]) * 57.2958,
                                2).item(),
                'front_margin': np.round(-0.6128 - (by + hy), 4).item(),
                'back_margin': np.round((by - hy) + 1.0472, 4).item(),
                'x_margin': np.round(xm1, 4).item(),
                'board_gap': np.round(st['object']['pos'][2] - 0.7408, 4).item(),
                'base': st['base'], 't': st['t'],
                'sites': {'left': api.geom()['sites']['left']['pos'],
                          'right': api.geom()['sites']['right']['pos']},
                'hb': handbox()}

    api.reach_box(targets={'left': [0.93, 0.32, 0.86],
                           'right': [0.96, -0.32, 0.86]},
                  sides=('left', 'right'), R_body=RB, dur=0.9)
    api.step_goal(upper=api.commanded(), dur=0.5)
    out['hover'] = {'pads': pads(), 'base': api.state()['base']}

    def servo(gy, gz, n, maxstep, dur, settle, tag):
        rows = []
        k = 0
        while k < n:
            s0 = api.geom()['sites']
            r = api.reach_box(
                targets={'left': [s0['left']['pos'][0],
                                  s0['left']['pos'][1],
                                  s0['left']['pos'][2]],
                         'right': [s0['right']['pos'][0],
                                   s0['right']['pos'][1],
                                   s0['right']['pos'][2]]},
                sides=('left', 'right'), R_body=RB, dur=dur)
            api.step_goal(upper=r.get('upper_target'), dur=settle)
            pd = pads()
            st = api.state()
            eL = [0.0, gy[0] - pd['left'][1], gz - pd['left'][2]]
            eR = [0.0, gy[1] - pd['right'][1], gz - pd['right'][2]]
            rows.append({'tag': tag, 'k': k, 'eL': eL, 'eR': eR,
                         'obj': st['object']['pos'], 'ik': r.get('ik_err'),
                         'hb': handbox()['n']})
            done = 1
            for e in (eL, eR):
                if e[1] > 0.02 or e[1] < -0.02:
                    done = 0
                if e[2] > 0.02 or e[2] < -0.02:
                    done = 0
            if done:
                break
            s1 = api.geom()['sites']
            dl = eL[1]
            if dl > maxstep:
                dl = maxstep
            if dl < -maxstep:
                dl = -maxstep
            dzl = eL[2]
            if dzl > maxstep:
                dzl = maxstep
            if dzl < -maxstep:
                dzl = -maxstep
            dr = eR[1]
            if dr > maxstep:
                dr = maxstep
            if dr < -maxstep:
                dr = -maxstep
            dzr = eR[2]
            if dzr > maxstep:
                dzr = maxstep
            if dzr < -maxstep:
                dzr = -maxstep
            r = api.reach_box(
                targets={'left': [s1['left']['pos'][0],
                                  s1['left']['pos'][1] + dl,
                                  s1['left']['pos'][2] + dzl],
                         'right': [s1['right']['pos'][0],
                                   s1['right']['pos'][1] + dr,
                                   s1['right']['pos'][2] + dzr]},
                sides=('left', 'right'), R_body=RB, dur=dur)
            api.step_goal(upper=r.get('upper_target'), dur=settle)
            k = k + 1
        return rows

    out['descend'] = servo((0.30, -0.30), z, 4, 0.055, 0.7, 0.5, 'descend')
    out['inward'] = servo((yL, yR), z, 3, 0.04, 0.6, 0.4, 'inward')
    st = api.state()
    out['pre'] = {'obj': st['object']['pos'], 'pads': pads(),
                  'hb': handbox()}
    api.close_grip(angle=grip, dur=1.0)
    carry = api.commanded()
    out['close'] = {'obj': api.state()['object']['pos'], 'hb': handbox()}

    lift = []
    k = 0
    while k < 5:
        s0 = api.geom()['sites']
        rl = api.reach_box(
            targets={'left': [s0['left']['pos'][0], s0['left']['pos'][1],
                              s0['left']['pos'][2] + 0.016],
                     'right': [s0['right']['pos'][0], s0['right']['pos'][1],
                               s0['right']['pos'][2] + 0.016]},
            sides=('left', 'right'), R_body=RB, dur=0.4, hand_hold=carry)
        if rl.get('upper_target'):
            carry = rl.get('upper_target')
        api.step_goal(upper=carry, dur=0.25)
        st = api.state()
        lift.append({'k': k, 'obj': st['object']['pos'], 'base': st['base'],
                     'ik': rl.get('ik_err'), 'hb': handbox()['n']})
        k = k + 1
    out['lift'] = lift

    stand = []
    k = 0
    while k < 10:
        api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0],
                      height=h + (walk_h - h) * (k + 1) / 10.0, dur=0.12)
        st = api.state()
        stand.append({'k': k, 'base': st['base'],
                      'obj': st['object']['pos'], 'hb': handbox()['n']})
        k = k + 1
    out['stand'] = stand

    def walk_hold(vx2, vy2, wz2, dur, upper, hh=0.66):
        n = 0
        while n < 20:
            if n * 0.05 >= dur:
                break
            api.step_goal(upper=upper, nav=[vx2, vy2, wz2], height=hh,
                          dur=0.05)
            n = n + 1

    def row(tag, k):
        st = api.state()
        bs = api.base_state()
        r = {'tag': tag, 'k': k, 'base': st['base'],
             'obj': st['object']['pos'], 'yaw': bs['yaw_deg'],
             'hb': handbox()['n']}
        if con_log:
            r['env'] = envc()
        return r

    def yawfix(ref, maxk, wmax, hh=0.66):
        rows = []
        k = 0
        while k < maxk:
            y = api.base_state()['yaw_deg']
            e = ref - y
            if e < 1.5 and e > -1.5:
                break
            d = 0.08
            if e > 12.0 or e < -12.0:
                d = 0.25
            elif e > 5.0 or e < -5.0:
                d = 0.15
            if e > 0.0:
                walk_hold(0.0, 0.0, wmax, d, carry, hh)
            else:
                walk_hold(0.0, 0.0, -wmax, d, carry, hh)
            rows.append({'tag': 'yawfix', 'k': k,
                         'yaw': api.base_state()['yaw_deg'],
                         'base': api.state()['base'], 'e0': e})
            k = k + 1
        return rows

    # ---- pre-turn retreat: the turn adds +0.10..+0.19 m x (pp16 measured),
    # so pre_x must absorb that or the pelvis wedges on the table slab.
    ret = [row('retreat', 0)]
    k = 1
    latch = -1.0
    while k < n_ret:
        st0 = api.state()          # the SAME single read pp27 performs here
        if st0['base'][0] <= pre_x:
            break
        if latch < 0.0:
            # ONE-SHOT fork, decided from the loop's existing first read.
            # Re-deciding every row would be a LIVE deadband, and the pin's
            # own base y drifts across it mid-retreat even on a passer
            # (seed 0: -0.0710 -> -0.0080), which would break bit-identity.
            latch = 0.0
            if pre_lat_y > -1.0 and st0['base'][1] > pre_lat_y:
                latch = 1.0
        vy0 = pre_lat_v if latch > 0.5 else pre_vy
        walk_hold(-0.18, vy0, 0.0, 0.4, carry, walk_h)
        b = row('retreat', k)
        ret.append(b)
        if b['hb'] == 0:
            break
        k = k + 1
    out['retreat'] = ret

    # ------------------------------------------------------- turn to face -y
    turns = []
    k = 0
    yp = None
    while k < 26:
        yaw = api.base_state()['yaw_deg']
        if yaw <= turn_yaw + 6.0:
            break
        # ---- delta-aware stop (turn_pred=1): the fixed 6 deg margin is
        # calibrated for ~6-8 deg/row; at wz<=-1.3 a row carries 12-15 deg, so
        # the rule above overshoots by up to a full row and the yawfix has to
        # walk it back.  Break BEFORE a row whose PREDICTED end yaw (last
        # measured delta, extrapolated) would pass turn_yaw - margin.
        # turn_pred=0 executes the identical read/command stream (yp unused).
        if turn_pred > 0 and yp is not None:
            if yaw + (yaw - yp) <= turn_yaw - turn_pred_margin:
                break
        walk_hold(0.0, 0.0, wz, 0.25, carry, walk_h)
        turns.append(row('turn', k))
        if turns[k]['hb'] == 0:
            break
        yp = yaw
        k = k + 1
    out['turns'] = turns
    out['turn_fix'] = yawfix(turn_yaw, 8, 0.8, walk_h)
    api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0], height=walk_h, dur=0.4)
    out['S0'] = row('S0', 0)

    # ---- post-turn lateral trim: body -y is world -x at yaw -90, which is
    # the direction that pulls the pelvis off the table slab.
    strafes = []
    k = 0
    while k < n_str:
        if api.state()['base'][0] <= post_x:
            break
        walk_hold(str_vx, -str_v, 0.0, 0.3, carry, walk_h)
        strafes.append(row('strafe', k))
        k = k + 1
        # ---- a lateral command at a yaw offset is absorbed as foot yaw
        # rather than base translation (seed-1 diag: 5 rows moved x 0.000
        # while yaw drifted -88.5 -> -85.2).  Re-level the base yaw between
        # lateral bursts when str_yaw_tol > 0.
        dyw = api.base_state()['yaw_deg'] - turn_yaw
        if str_yaw_tol > 0.0 and (dyw > str_yaw_tol or dyw < -str_yaw_tol):
            for rr in yawfix(turn_yaw, 2, 0.7, walk_h):
                rr['tag'] = 'strfix'
                strafes.append(rr)
    out['strafes'] = strafes
    out['S1'] = row('S1', 0)

    # ---- carry tuck: pull both palms (and so the pinched box) +y toward
    # the body by `tuck` before the walk, then hold that pose.  Measured
    # motivation: seed 1 carries the box 0.424 m forward of its base vs
    # 0.320/0.382/0.358 for seeds 0/2/3 and 0.028 m higher, and its walk
    # pins at base y -0.098 under a persistent table_main/robot0_pelvis
    # contact.  A +y palm shift moves the box toward the body (the release
    # retract uses the same sign and moves the box 0).
    if tuck != 0.0 or tuck_x != 0.0:
        s1p = api.geom()['sites']
        rt = api.reach_box(
            targets={'left': [s1p['left']['pos'][0] + tuck_x,
                              s1p['left']['pos'][1] + tuck,
                              s1p['left']['pos'][2]],
                     'right': [s1p['right']['pos'][0] + tuck_x,
                               s1p['right']['pos'][1] + tuck,
                               s1p['right']['pos'][2]]},
            sides=('left', 'right'), R_body=RB, dur=0.4, hand_hold=carry)
        if rt.get('upper_target'):
            carry = rt.get('upper_target')
        api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0], height=tuck_h,
                      dur=0.25)
        out['tuck'] = {'tuck': tuck, 'tuck_x': tuck_x,
                       'ik': rt.get('ik_err'),
                       'base': api.state()['base'],
                       'obj': api.state()['object']['pos'],
                       'siteL': api.geom()['sites']['left']['pos'],
                       'siteR': api.geom()['sites']['right']['pos'],
                       'hb': handbox()}

    # --------- walk until the LIVE front_margin says the box is over the
    # board (or environment contact / lost box / stall).
    walks = []
    why = 'max'
    k = 0
    stall = 0
    kf = 0
    boxenv = []
    while k < kmax:
        sn = snap()
        if sn['front_margin'] >= fm_stop:
            why = 'over_board'
            break
        yaw = api.base_state()['yaw_deg']
        if yaw > turn_yaw + yaw_guard or yaw < turn_yaw - yaw_guard:
            walks.append({'tag': 'pre_yawfix', 'k': k, 'yaw': yaw,
                          'base': sn['base'], 'fm': sn['front_margin']})
            yfx = yawfix(turn_yaw, 2, 0.7, walk_h)
            for x_ in yfx:
                x_['tag'] = 'walkfix'
                walks.append(x_)
            kf = kf + 1
            if kf > 10:
                break
        b0 = api.state()['base']
        walk_hold(vx, vy_walk, 0.0, 0.4, carry, walk_h)
        sn = snap()
        b = sn['base']
        env = envc()
        w = {'tag': 'walk', 'k': k, 'base': b, 'obj': sn['obj'],
             'yaw': api.base_state()['yaw_deg'], 'hb': sn['hb']['n'],
             'fm': sn['front_margin'], 'hy': sn['half_y'],
             'xm': sn['x_margin'], 'boxyaw': sn['yaw'],
             'gap': sn['board_gap'], 'env': env}
        walks.append(w)
        big = 0.0
        for c in env:
            s_ = c['b1'] + ' ' + c['b2']
            if 'box' in s_ or 'object' in s_:
                boxenv.append({'k': k, 'c': c, 'fm': sn['front_margin']})
                continue
            if 'shelf' in s_ or 'cabinet' in s_ or 'table' in s_:
                if c['f'] > big:
                    big = c['f']
        if big > contact_abort:
            why = 'contact'
            break
        if w['hb'] == 0:
            why = 'lost_box'
            break
        if b0[1] - b[1] < 0.008:
            stall = stall + 1
        else:
            stall = 0
        if stall >= stall_n:
            why = 'stalled'
            break
        k = k + 1
    out['walks'] = walks
    out['walk_why'] = why
    out['boxenv'] = boxenv
    api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0], height=walk_h, dur=0.5)
    out['settle'] = snap()

    # --------------- arm push: at carry height the 10 g box hangs ~0.34 m
    # ahead of the base, so translating the palms in world -y drags it over the
    # board edge with no leg contact.  pp16's carry-height advance levered the
    # box up because it was a multi-step 0.0139 m advance with unmeasured
    # vertical drift; each push here is one small dy with explicit push_z, the
    # palms stay at walk_h, and the box's dy/dx/dz are logged, not assumed.
    pushes = []
    k = 0
    while k < push_n:
        sn = snap()
        fm0 = sn['front_margin']
        if fm0 >= push_fm_stop:
            break
        s0 = api.geom()['sites']
        rl = api.reach_box(
            targets={'left': [s0['left']['pos'][0],
                              s0['left']['pos'][1] + push_dy,
                              s0['left']['pos'][2] + push_z],
                     'right': [s0['right']['pos'][0],
                               s0['right']['pos'][1] + push_dy,
                               s0['right']['pos'][2] + push_z]},
            sides=('left', 'right'), orientation='keep', dur=push_dur,
            hand_hold=carry)
        if rl.get('upper_target'):
            carry = rl.get('upper_target')
        api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0], height=walk_h,
                      dur=push_settle)
        sn2 = snap()
        pushes.append({'tag': 'push', 'k': k, 'base': sn2['base'],
                       'obj': sn2['obj'], 'ik': rl.get('ik_err'),
                       'fm': sn2['front_margin'], 'fm0': fm0,
                       'hy': sn2['half_y'], 'xm': sn2['x_margin'],
                       'gap': sn2['board_gap'], 'hb': sn2['hb']['n'],
                       'dy': np.round(sn2['obj'][1] - sn['obj'][1], 4).item(),
                       'dx': np.round(sn2['obj'][0] - sn['obj'][0], 4).item(),
                       'dz': np.round(sn2['obj'][2] - sn['obj'][2], 4).item()})
        if sn2['hb']['n'] == 0:
            break
        k = k + 1
    out['pushes'] = pushes

    # ------------------------------------------------- body to board height
    drops = []
    if drop == 'set':
        # pp25: drop_ramp=1, drop_dur=1.1 reproduces the pin exactly.
        # drop_ramp>1 splits the same 11 cm (walk_h -> low_h) into drop_ramp
        # sequential set_height targets of drop_dur/drop_ramp seconds each,
        # with NO base command, so only the descent rate changes.
        if drop_ramp > 1:
            kr = 0
            while kr < drop_ramp:
                hh = walk_h + (low_h - walk_h) * (kr + 1) / drop_ramp
                api.set_height(hh, dur=drop_dur / drop_ramp)
                kr = kr + 1
        else:
            api.set_height(low_h, dur=drop_dur)
        api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0], height=low_h, dur=0.5)
    elif drop == 'walk':
        k = 0
        while k < n_low:
            sn0 = snap()
            if drop_fm_stop > -8.0 and sn0['front_margin'] >= drop_fm_stop:
                break
            hh = carry_h + (low_h - carry_h) * (k + 1) / n_low
            api.step_goal(upper=carry, nav=[low_vx, 0.0, 0.0], height=hh,
                          dur=0.3)
            sn = snap()
            drops.append({'tag': 'lw', 'k': k, 'hh': np.round(hh, 4).item(),
                          'base': sn['base'], 'obj': sn['obj'],
                          'fm': sn['front_margin'], 'gap': sn['board_gap'],
                          'hy': sn['half_y'], 'hb': sn['hb']['n'],
                          'hbx': sn['hb']['f'],
                          'dy': np.round(sn['base'][1] - sn0['base'][1],
                                         4).item()})
            k = k + 1
    out['drops'] = drops
    out['body_low'] = snap()

    # ---------- low-height top-up push (only meaningful in 'set' mode: pp16
    # measured ~6% velocity tracking at height 0.55, so keep it small)
    nudges = []
    if drop == 'set':
        k = 0
        while k < nudge_n:
            sn = snap()
            fm0 = sn['front_margin']
            if fm0 >= fm_gate:
                break
            b0 = sn['base']
            walk_hold(nudge_v, 0.0, 0.0, nudge_dur, carry, low_h)
            sn = snap()
            nudges.append({'tag': 'nudge', 'k': k, 'base': sn['base'],
                           'fm': sn['front_margin'], 'fm0': fm0,
                           'hy': sn['half_y'],
                           'xm': sn['x_margin'], 'gap': sn['board_gap'],
                           'hb': sn['hb']['n'],
                           'dy': np.round(sn['base'][1] - b0[1], 4).item()})
            if sn['front_margin'] < fm0 - 0.020:
                break
            k = k + 1
    out['nudges'] = nudges

    # ---------------------------------- yaw regulation at low height (pp21)
    # Zero translation, zero dz.  A negative shr rotates the grasp line so the
    # box yaw DECREASES (pp20 identification: shr +0.004 -> yaw up, shr -0.004
    # -> yaw down), so shr = -yaw_k*(yaw-yaw_tgt).  This buys hy down towards
    # its yaw-90 minimum (0.0975) instead of carrying the 0.1438 the baseline
    # grasp drifts to.
    yaws = []
    k = 0
    while k < yaw_n:
        sn = snap()
        if sn['yaw'] <= yaw_tgt + yaw_tol:
            break
        e = sn['yaw'] - yaw_tgt
        shr = -yaw_k * e
        if shr > yaw_shr_max:
            shr = yaw_shr_max
        if shr < -yaw_shr_max:
            shr = -yaw_shr_max
        s0 = api.geom()['sites']
        rl = api.reach_box(
            targets={'left': [s0['left']['pos'][0] + yaw_spr,
                              s0['left']['pos'][1] + shr,
                              s0['left']['pos'][2] + yaw_dz],
                     'right': [s0['right']['pos'][0] - yaw_spr,
                               s0['right']['pos'][1] - shr,
                               s0['right']['pos'][2] + yaw_dz]},
            sides=('left', 'right'), orientation='keep', dur=0.4,
            hand_hold=carry)
        if rl.get('upper_target'):
            carry = rl.get('upper_target')
        api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0], height=low_h,
                      dur=0.25)
        sn2 = snap()
        yaws.append({'k': k, 'obj': sn2['obj'], 'base': sn2['base'],
                     'ik': rl.get('ik_err'), 'cmd_shr': shr,
                     'cmd_spr': yaw_spr, 'err0': np.round(e, 3).item(),
                     'dyaw': np.round(sn2['yaw'] - sn['yaw'], 3).item(),
                     'yaw': sn2['yaw'], 'hy': sn2['half_y'],
                     'fm': sn2['front_margin'], 'gap': sn2['board_gap'],
                     'xm': sn2['x_margin'], 'up': sn2['up'],
                     'dx': np.round(sn2['obj'][0] - sn['obj'][0], 4).item(),
                     'dy': np.round(sn2['obj'][1] - sn['obj'][1], 4).item(),
                     'dz': np.round(sn2['obj'][2] - sn['obj'][2], 4).item(),
                     'hb': sn2['hb']['n'],
                     'st0': {'l': s0['left']['pos'], 'r': s0['right']['pos']},
                     'st1': {'l': sn2['sites']['left'],
                             'r': sn2['sites']['right']}})
        if sn2['hb']['n'] == 0:
            break
        k = k + 1
    out['yaws'] = yaws

    # --------------------- arm descent: the box's last 0.09 m in 'none' mode,
    # a small trim otherwise.  reach_box dz with hand_hold=carry (pp15's
    # levering came from horizontal advance, not from vertical descent).
    lows = []
    k = 0
    while k < lows_n:
        sn = snap()
        if sn['board_gap'] <= lows_gap:
            break
        if sn['front_margin'] >= lows_fm_stop:
            break
        er = sn['board_gap']
        dz = -0.008
        if er > 0.04:
            dz = -0.012
        if low_dz is not None:
            dz = low_dz
        dy = 0.0
        if sn['front_margin'] < low_dy_fm:
            dy = low_dy
        shr = low_shr
        spr = low_spread
        dx = low_dx
        if low_dy_rows is not None:
            ar = np.asarray(low_dy_rows, dtype='float64')
            if ar.shape[0] > 0:
                if k < ar.shape[0]:
                    dy = ar[k].item()
                else:
                    dy = ar[ar.shape[0] - 1].item()
        if low_shr_rows is not None:
            ar = np.asarray(low_shr_rows, dtype='float64')
            if ar.shape[0] > 0:
                if k < ar.shape[0]:
                    shr = ar[k].item()
                else:
                    shr = ar[ar.shape[0] - 1].item()
        if low_spread_rows is not None:
            ar = np.asarray(low_spread_rows, dtype='float64')
            if ar.shape[0] > 0:
                if k < ar.shape[0]:
                    spr = ar[k].item()
                else:
                    spr = ar[ar.shape[0] - 1].item()
        if low_dx_rows is not None:
            ar = np.asarray(low_dx_rows, dtype='float64')
            if ar.shape[0] > 0:
                if k < ar.shape[0]:
                    dx = ar[k].item()
                else:
                    dx = ar[ar.shape[0] - 1].item()
        yh = low_yaw_hold
        if low_yaw_hold_rows is not None:
            ar = np.asarray(low_yaw_hold_rows, dtype='float64')
            if ar.shape[0] > 0:
                if k < ar.shape[0]:
                    yh = ar[k].item()
                else:
                    yh = ar[ar.shape[0] - 1].item()
        if yh:
            e = sn['yaw'] - yaw_tgt
            shr = -yaw_k * e
            if shr > yaw_shr_max:
                shr = yaw_shr_max
            if shr < -yaw_shr_max:
                shr = -yaw_shr_max
        s0 = api.geom()['sites']
        rl = api.reach_box(
            targets={'left': [s0['left']['pos'][0] + spr + dx,
                              s0['left']['pos'][1] + dy + shr,
                              s0['left']['pos'][2] + dz],
                     'right': [s0['right']['pos'][0] - spr + dx,
                               s0['right']['pos'][1] + dy - shr,
                               s0['right']['pos'][2] + dz]},
            sides=('left', 'right'), orientation='keep', dur=0.4,
            hand_hold=carry)
        if rl.get('upper_target'):
            carry = rl.get('upper_target')
        api.step_goal(upper=carry, nav=[0.0, 0.0, 0.0], height=body_end,
                      dur=0.25)
        sn2 = snap()
        lows.append({'k': k, 'obj': sn2['obj'], 'base': sn2['base'],
                     'ik': rl.get('ik_err'), 'dz': np.round(
                         sn2['obj'][2] - sn['obj'][2], 4).item(),
                     'dy': np.round(sn2['obj'][1] - sn['obj'][1],
                                    4).item(),
                     'dx': np.round(sn2['obj'][0] - sn['obj'][0],
                                    4).item(),
                     'cmd_dz': dz, 'cmd_dy': dy, 'cmd_shr': shr,
                     'cmd_spr': spr, 'cmd_dx': dx,
                     'bdy': np.round(sn2['base'][1] - sn['base'][1],
                                     4).item(), 'fm': sn2['front_margin'],
                     'hy': sn2['half_y'], 'yaw': sn2['yaw'],
                     'up': sn2['up'],
                     'xm': sn2['x_margin'], 'gap': sn2['board_gap'],
                     'hb': sn2['hb']['n'],
                     'st0': {'l': s0['left']['pos'], 'r': s0['right']['pos']},
                     'st1': {'l': sn2['sites']['left'],
                             'r': sn2['sites']['right']}})
        k = k + 1
    out['lowers'] = lows

    # --------------------- torso lean (pp21b): waist pitch as a reach lever.
    # The 31-joint contract order is waist(3) + left arm(7) + right arm(7) +
    # left hand(7) + right hand(7), so index 2 is waist_pitch_joint.  The lean
    # is ABSOLUTE (radians) on each row and the arms keep their carried
    # joint targets, so the hands move with the torso.  This is the untested
    # way to buy the last few cm of box -y once the arms are at reach limit
    # and the base is blocked by the shelf front.  lean_rows=null disables it.
    leans = []
    if lean_rows is not None:
        ar = np.asarray(lean_rows, dtype='float64')
        nl = ar.shape[0]
        k = 0
        while k < nl:
            sn = snap()
            v = np.asarray(carry, dtype='float64').tolist()
            v[2] = ar[k].item()
            api.step_goal(upper=v, nav=[0.0, 0.0, 0.0], height=body_end,
                          dur=0.5)
            carry = v
            sn2 = snap()
            leans.append({'k': k, 'lean': ar[k].item(), 'obj': sn2['obj'],
                          'base': sn2['base'], 'fm': sn2['front_margin'],
                          'hy': sn2['half_y'], 'yaw': sn2['yaw'],
                          'gap': sn2['board_gap'], 'xm': sn2['x_margin'],
                          'up': sn2['up'], 'hb': sn2['hb']['n'],
                          'dy': np.round(sn2['obj'][1] - sn['obj'][1],
                                         4).item(),
                          'dz': np.round(sn2['obj'][2] - sn['obj'][2],
                                         4).item(),
                          'bdy': np.round(sn2['base'][1] - sn['base'][1],
                                          4).item(),
                          'sl': sn2['sites']['left'],
                          'sr': sn2['sites']['right']})
            k = k + 1
    out['leans'] = leans
    out['pre_release'] = snap()

    # ------------------------------------------------------------- release
    js = api.joints()
    names = []
    for kk in js:
        names.append(kk)
    v = np.asarray(carry).tolist()
    i = 0
    while i < 31:
        if '_hand_' in names[i]:
            v[i] = 0.0
        i = i + 1
    pr = snap()
    gate = 0
    if pr['front_margin'] >= fm_gate:
        if pr['board_gap'] <= 0.05:
            gate = 1
    out['release_gate'] = {'front_margin': pr['front_margin'],
                           'board_gap': pr['board_gap'],
                           'x_margin': pr['x_margin'],
                           'pass': gate}
    if rel_mode == 'hold':
        api.step_goal(upper=v, nav=[0.0, 0.0, 0.0], height=body_end, dur=0.9)
        out['released'] = snap()
        api.step_goal(upper=v, nav=[0.0, 0.0, 0.0], height=body_end, dur=0.6)
        out['released_b'] = snap()

        out['hold'] = snap()
        api.step_goal(upper=v, nav=[0.0, 0.0, 0.0], height=body_end, dur=0.8)
        out['hold_b'] = snap()

        hb = handbox()
        if hb['n'] == 0:
            walk_hold(-0.10, 0.0, 0.0, 0.5, v, body_end)
            api.step_goal(upper=v, nav=[0.0, 0.0, 0.0], height=body_end, dur=0.8)
            out['withdraw'] = snap()
        else:
            out['withdraw'] = {'skipped': 1, 'hb': hb['n']}
    else:
        # settle: fingers open (v has the _hand_ joints zeroed), then command
        # each palm down by the box's measured board_gap so the box follows
        # onto the board, then pull both palms +y clear of the box.
        # rel_open <= 0 skips the standalone open step: the fingers then ramp
        # open inside row 0's reach_box (hand_hold=v) while the palms descend,
        # so the box is supported instead of free-falling.
        if rel_pre > 0.0:
            # Descend with the fingers still CLOSED (hand_hold=carry holds the
            # pinch) until the box is at the board, then let the open step run
            # with the box already supported.  Rationale: the open step alone
            # drops the box the remaining board_gap while the pads are still
            # touching it, and the pads then drag/spin the box (seed 7 at
            # grip 1.05: -78 mm y and -23.5 deg during a 0.45 s open from
            # gap 0.0202).  Default 0.0 keeps the pinned open-first path.
            rows_p = []
            k = 0
            while k < rel_n:
                sn = snap()
                if sn['board_gap'] <= rel_gap_tol:
                    break
                dzc = -rel_gain * sn['board_gap']
                sl = sn['sites']['left']
                sr = sn['sites']['right']
                api.reach_box(targets={'left': [sl[0], sl[1], sl[2] + dzc],
                                       'right': [sr[0], sr[1], sr[2] + dzc]},
                              sides=('left', 'right'), orientation='keep',
                              dur=rel_dur, hand_hold=carry)
                api.step_goal(upper=api.commanded(), nav=[0.0, 0.0, 0.0],
                              height=body_end, dur=rel_hold)
                s2 = snap()
                rows_p.append({'k': k, 'cmd_dz': dzc,
                               'gap': s2['board_gap'], 'up': s2['up'],
                               'fm': s2['front_margin'],
                               'bm': s2['back_margin'],
                               'yaw': s2['yaw'], 'hy': s2['half_y'],
                               'hb': s2['hb']['n'],
                               'dy': np.round(s2['obj'][1] - sn['obj'][1],
                                              4).item(),
                               'dz': np.round(s2['obj'][2] - sn['obj'][2],
                                              4).item()})
                k = k + 1
            out['predescend'] = rows_p
        if rel_open > 0.0:
            if rel_dx != 0.0:
                sn0 = snap()
                sl0 = sn0['sites']['left']
                sr0 = sn0['sites']['right']
                api.reach_box(
                    targets={'left': [sl0[0] + rel_dx, sl0[1], sl0[2]],
                             'right': [sr0[0] + rel_dx, sr0[1], sr0[2]]},
                    sides=('left', 'right'), orientation='keep',
                    dur=rel_open, hand_hold=v)
            else:
                api.step_goal(upper=v, nav=[0.0, 0.0, 0.0], height=body_end,
                              dur=rel_open)
        out['released'] = snap()
        shep = out['released']['hb']['f'] >= rel_grasp_min
        rows = []
        k = 0
        while k < rel_n:
            sn = snap()
            if sn['board_gap'] <= rel_gap_tol:
                break
            dzc = -rel_gain * sn['board_gap']
            dxc = 0.0
            if k == 0:
                dxc = rel_open_x
            # pp33 (2026-11-10): the settle row's lateral pad motion is a
            # shepherd only while the grasp released with the box still in
            # the pads.  Latch that once from the released snap recorded
            # just above (a dict lookup, no extra read): if the open left
            # the box with no measurable pinch, the same lateral command
            # ejects it off the board instead of steering it (seed 9:
            # released hand-box force 0.026 N, lateral row crossed
            # x_margin +0.0173 -> -0.0377 m).  With no pinch, descend
            # straight down and let the box settle.  Latched once, so a
            # seed whose release retained the box keeps the pinned
            # arithmetic on every row.  rel_grasp_min=0.5 sits between the
            # largest contact-free release (0.026 N) and the smallest
            # retained pinch (9.09 N) measured on seeds 0-13.
            lat_dx = rel_dx
            if not shep:
                dxc = 0.0
                lat_dx = 0.0
            sl = sn['sites']['left']
            sr = sn['sites']['right']
            api.reach_box(targets={'left': [sl[0] + dxc + lat_dx, sl[1],
                                         sl[2] + dzc],
                               'right': [sr[0] - dxc + lat_dx, sr[1],
                                         sr[2] + dzc]},
                      sides=('left', 'right'), orientation='keep',
                      dur=rel_dur, hand_hold=v)
            api.step_goal(upper=api.commanded(), nav=[0.0, 0.0, 0.0],
                          height=body_end, dur=rel_hold)
            s2 = snap()
            rows.append({'k': k, 'cmd_dz': dzc, 'cmd_dx': dxc,
                         'gap': s2['board_gap'], 'up': s2['up'],
                         'fm': s2['front_margin'], 'yaw': s2['yaw'],
                         'hy': s2['half_y'], 'hb': s2['hb']['n'],
                         'dy': np.round(s2['obj'][1] - sn['obj'][1],
                                        4).item(),
                         'dz': np.round(s2['obj'][2] - sn['obj'][2],
                                        4).item(),
                         'sl': s2['sites']['left'],
                         'sr': s2['sites']['right']})
            k = k + 1
        out['seat_rows'] = rows
        out['seated'] = snap()
        out['released_b'] = out['seated']
        sq = out['seated']
        sl = sq['sites']['left']
        sr = sq['sites']['right']
        api.reach_box(targets={'left': [sl[0] + rel_dx, sl[1] + rel_ret,
                                         sl[2]],
                               'right': [sr[0] + rel_dx, sr[1] + rel_ret,
                                         sr[2]]},
                      sides=('left', 'right'), orientation='keep',
                  dur=rel_dur, hand_hold=v)
        api.step_goal(upper=api.commanded(), nav=[0.0, 0.0, 0.0],
                      height=body_end, dur=rel_hold)
        out['hold'] = snap()
        out['withdraw'] = out['hold']
        out['hold_b'] = snap()
    st = api.state()
    out['final'] = {'obj': st['object']['pos'],
                    'obj_geom': api.geom()['object'], 'base': st['base'],
                    't': st['t'], 'hb': handbox()['n']}
    cf = v
    if rel_mode != 'hold':
        cf = api.commanded()
    api.step_goal(upper=cf, nav=[0.0, 0.0, 0.0], height=body_end, dur=0.6)
    fb = snap()
    out['final_b'] = fb
    out['verdict'] = {'obj': fb['obj'], 'up': fb['up'],
                      'front_margin': fb['front_margin'],
                      'back_margin': fb['back_margin'],
                      'x_margin': fb['x_margin'],
                      'board_gap': fb['board_gap'],
                      'yaw': fb['yaw'], 'hb': fb['hb']['n']}
    return out
