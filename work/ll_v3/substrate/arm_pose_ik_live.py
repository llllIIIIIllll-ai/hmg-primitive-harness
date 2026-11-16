def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            + (a[2] - b[2]) ** 2) ** 0.5


def run(api, grasp=None, orientation='keep', max_steps=50, max_delta=0.08,
        iters=80, hand_hold=None, stop_err=0.012, hold_base=True,
        base_vmax=0.03, base_wmax=0.05, base_gain=0.05,
        base_deadband=0.02, base_yaw_gain=0.3, base_yaw_deadband=0.01,
        trace_contacts=False, contact_match='drill', freeze_frame=False,
        reorient_each_step=False, freeze_object_pos=False,
        wrist_guard=False, wrist_guard_dist=0.010, max_delta_near=0.005,
        near_dist=0.15, orient_mode='side', seed_mode='none',
        seed_force=False, joint_mode=False, live_guard_dist=0.02,
        budget=True, **kw):
    """True closed-loop reach: solve from the live state with apply=False,
    clamp the per-step joint delta, apply one control step (sim advances),
    re-observe.  Logs sim time, live site, base, NaN and reset per iteration."""
    dx0 = (grasp or {}).get('dx', -0.0838)
    dy0 = (grasp or {}).get('dy', 0.07)
    z0 = (grasp or {}).get('z', 0.94)
    b0 = api.base_state()
    b_h = api.base_state()
    hold_xy = [b_h['pos'][0], b_h['pos'][1]]
    hold_yaw = b_h['yaw_deg']
    r_body = None
    r_des_side = {}
    f_px = None
    obj_fix = None
    normals = None
    trace = []
    resets = 0
    nans = 0
    wrist_abort = False
    wd_min = None
    wd_side = None
    wd_step = None
    seed_q = None
    seed_q_use = None
    seed_vec = None
    radii = {}
    if wrist_guard:
        agg0 = api.arm_geoms()
        for body in agg0:
            for gg in agg0[body]['geoms']:
                nm0 = gg['name']
                if (nm0 != 'robot0_g27_col' and nm0 != 'robot0_g28_col'
                        and nm0 != 'robot0_g34_col'
                        and nm0 != 'robot0_g35_col'):
                    continue
                sz0 = gg.get('local_size') or [0.0]
                tp0 = gg.get('type') or ''
                rad0 = sz0[0]
                if tp0 == 'box' or tp0 == 'cylinder':
                    rad0 = (sz0[0] * sz0[0] + sz0[1] * sz0[1]
                            + sz0[2] * sz0[2]) ** 0.5
                radii[nm0] = rad0
    first_step = None
    r = None
    k = 0
    while k < max_steps:
        st = api.observe()
        ob = st.get('object') or {}
        obj = ob.get('pos')
        # Lift: freeze the command frame at the grasp.  Re-deriving it from
        # the live object would raise the target WITH the object the hand is
        # carrying, making the relative lift zero.  The live object is read
        # outside this primitive only to verify the rise/contacts.
        if freeze_object_pos:
            if obj_fix is None:
                obj_fix = [obj[0], obj[1], obj[2]]
            else:
                obj = obj_fix
        b = api.base_state()
        t_b = api.t
        # base hold: the WBC walks backward under arm reaction when only an
        # upper-body target is sent (measured 0.44 m / 1.6 s with nav=0).
        # Send the same body-frame velocity law as the standoff servo so the
        # base stays at the pose the arm targets were measured from.
        dxh = hold_xy[0] - b['pos'][0]
        dyh = hold_xy[1] - b['pos'][1]
        yawh = np.radians(b['yaw_deg'])
        dxr = dxh * np.cos(yawh) + dyh * np.sin(yawh)
        dyr = -dxh * np.sin(yawh) + dyh * np.cos(yawh)
        # Gentle, deadbanded hold.  A high-gain P loop against the
        # velocity-controlled walker (which has lag) oscillated: nav
        # saturated at +/-0.2 and the base wandered 0.7 m in circles
        # (14:19 run).  Measured WBC reaction at max_delta 0.015 is
        # ~0.006 m/s, so a small cap suffices and the re-standoff cleans
        # up any residual.  wz is a yaw-setpoint error, never atan2 of
        # the position error (that is sign noise near zero).
        ex = 0.0
        if dxr > base_deadband:
            ex = dxr - base_deadband
        elif dxr < -base_deadband:
            ex = dxr + base_deadband
        ey = 0.0
        if dyr > base_deadband:
            ey = dyr - base_deadband
        elif dyr < -base_deadband:
            ey = dyr + base_deadband
        ye = np.radians(hold_yaw - b['yaw_deg'])
        while ye > 3.14159:
            ye = ye - 6.28318
        while ye < -3.14159:
            ye = ye + 6.28318
        eyaw = 0.0
        if ye > base_yaw_deadband:
            eyaw = ye - base_yaw_deadband
        elif ye < -base_yaw_deadband:
            eyaw = ye + base_yaw_deadband
        wz = np.clip(base_yaw_gain * eyaw, -base_wmax, base_wmax)
        vy = np.clip(base_gain * ey, -base_vmax, base_vmax)
        vx = np.clip(base_gain * ex, -base_vmax, base_vmax)
        nav = [0.0, 0.0, 0.0]
        if hold_base:
            nav = [vx, vy, wz]
        dxv = obj[0] - b['pos'][0]
        dyv = obj[1] - b['pos'][1]
        dd = (dxv * dxv + dyv * dyv) ** 0.5
        fx = dxv / dd
        fy = dyv / dd
        px = -fy
        py = fx
        # Freeze the approach/lateral axes at the first step.  The object is
        # static; the LIVE base->object axis rotates with any base drift and
        # pulled the left target across the midline during the descent
        # (y +0.100 -> -0.011 with 0.079 m of base drift, left wrist into
        # the drill).  Only the object POSITION is re-read.
        if f_px is None:
            f_px = [fx, fy, px, py]
        elif freeze_frame:
            fx = f_px[0]
            fy = f_px[1]
            px = f_px[2]
            py = f_px[3]
        targets = {'left': [obj[0] + dx0 * fx + dy0 * px,
                            obj[1] + dx0 * fy + dy0 * py, z0],
                   'right': [obj[0] + dx0 * fx - dy0 * px,
                             obj[1] + dx0 * fy - dy0 * py, z0]}
        if r_body is None:
            f_des = [fx, fy, 0.0]
            n_left = [-px, -py, 0.0]
            n_right = [px, py, 0.0]
            normals = {'left': n_left, 'right': n_right}
            if orient_mode != 'side':
                normals = None
            g = api.geom()
            hg = api.hand_geoms()
            q0 = np.asarray(api.raw[1].qpos[3:7])
            Rb = api._quat_to_mat(q0)
            r_body = {}
            for side in ('left', 'right'):
                if orient_mode != 'side':
                    continue
                site = g['sites'].get(side)
                if site is None:
                    continue
                R_site = np.array(site['xmat'])
                pad_pos = None
                pad_n = None
                for body in hg:
                    for gg in hg[body]['geoms']:
                        if gg['name'] == 'gripper0_%s_g0_col' % side:
                            pad_pos = np.array(gg['world_pos'])
                            pad_n = np.array(gg['xmat'])[:, 0]
                if pad_pos is None:
                    continue
                fwd_w = pad_pos - np.array(site['pos'])
                fwd_w = fwd_w / np.linalg.norm(fwd_w)
                e1 = R_site.T @ fwd_w
                e2 = R_site.T @ pad_n
                e2 = e2 - np.dot(e2, e1) * e1
                e2 = e2 / np.linalg.norm(e2)
                e3 = np.cross(e1, e2)
                B_local = np.column_stack([e1, e2, e3])
                f = np.array(f_des)
                f = f / np.linalg.norm(f)
                n = np.array(n_left if side == 'left' else n_right)
                n = n - np.dot(n, f) * f
                n = n / np.linalg.norm(n)
                c = np.cross(f, n)
                B_des = np.column_stack([f, n, c])
                R_des = B_des @ B_local.T
                U, S, Vt = np.linalg.svd(R_des)
                R_des = U @ Vt
                r_des_side[side] = R_des
                R_site0 = np.asarray(api.R_site0[side])
                R_body = Rb.T @ R_des @ R_site0.T
                U, S, Vt = np.linalg.svd(R_body)
                R_body = U @ Vt
                r_body[side] = R_body.tolist()
            ip = api.ik_preview(targets=targets, orients=(orientation,),
                                iters=150, sides=('left', 'right'))
            ip_err = {}
            ip_dq = {}
            for row in (ip.get('rows') or []):
                s_ = row.get('side')
                ip_err[s_] = row.get('err')
                ip_dq[s_] = row.get('dq')
                qn = row.get('q_named') or {}
                if qn:
                    if seed_q is None:
                        seed_q = {}
                    for kk in qn:
                        seed_q[kk] = qn[kk]
            first_step = {'targets': targets, 'max_delta': max_delta,
                          'q': api.upper().tolist(),
                          'ik_preview_err': ip_err,
                          'ik_preview_dq': ip_dq,
                          'ik_err': None, 'joint_margin_min': None,
                          'solver': None}
            # budget the loop from the first-step IK joint travel instead of
            # a fixed step count: required travel / max_delta (+ margin).
            dq_need = 0.0
            for s_ in ip_dq:
                if ip_dq[s_] is not None and ip_dq[s_] > dq_need:
                    dq_need = ip_dq[s_]
            denom = max_delta
            if (wrist_guard and max_delta_near < max_delta
                    and not joint_mode):
                denom = max_delta_near
            steps_est = dq_need / denom + 10.0
            if joint_mode:
                steps_est = steps_est * 3.0
            if budget and steps_est > max_steps:
                max_steps = steps_est
            first_step['dq_need'] = dq_need
            first_step['steps_est'] = steps_est
        # Compensation, not chasing: the world orientation target R_des is
        # fixed, so re-derive the body-frame rotation from the CURRENT base
        # rotation each step.  Holding R_body would rotate the world target
        # with any base yaw drift during the descent.
        if reorient_each_step and r_des_side:
            q0n = np.asarray(api.raw[1].qpos[3:7])
            Rb_n = api._quat_to_mat(q0n)
            for side in r_des_side:
                Rbod = (Rb_n.T @ r_des_side[side]
                        @ np.asarray(api.R_site0[side]).T)
                U, S, Vt = np.linalg.svd(Rbod)
                r_body[side] = (U @ Vt).tolist()
        q = api.upper()
        r = None
        # Joint-space fallback (W2l): interpolate the live joints toward the
        # DLS winning q instead of re-solving the QP every step.  The QP's
        # endpoint drifted 1.6-3.9 rad from a pinned seed and the site error
        # never decreased, so the branch cannot be pinned through the QP.
        if joint_mode and seed_vec is None and seed_q:
            sv = api.upper().tolist()
            for nm_ in seed_q:
                if nm_ in api.addr:
                    jj = api.addr[nm_]
                    kk = 0
                    while kk < 31:
                        if api.qpos_addr[kk] == jj:
                            sv[kk] = seed_q[nm_]
                            break
                        kk = kk + 1
            seed_vec = sv
        if joint_mode and seed_vec is not None:
            r = {'upper_target': seed_vec, 'ik_err': {},
                 'joint_margin_min': None, 'joint_margins': None,
                 'wrist_solved': {}}
            seed_q_use = seed_q
        elif seed_mode == 'dls' and seed_q and seed_q_use is None:
            # Multi-seed once (object static, cached): solve unseeded and
            # seeded from the DLS winning q, keep the smaller worst residual.
            # The QP is single-seed and hit joint limits on the right arm
            # for large wrist reorientations (0.36 m residual).
            r_un = api.reach_box_qp(task_dim=12, targets=targets,
                                    R_body=r_body, orientation=orientation,
                                    dur=0.02, iters=iters,
                                    hand_hold=hand_hold, apply=False)
            r_sd = api.reach_box_qp(task_dim=12, targets=targets,
                                    R_body=r_body, orientation=orientation,
                                    dur=0.02, iters=iters,
                                    hand_hold=hand_hold, apply=False,
                                    seed_q=seed_q)
            wu = None
            for s_ in (r_un.get('ik_err') or {}):
                v = r_un['ik_err'][s_]
                if wu is None or v > wu:
                    wu = v
            ws = None
            for s_ in (r_sd.get('ik_err') or {}):
                v = r_sd['ik_err'][s_]
                if ws is None or v > ws:
                    ws = v
            # Lexicographic choice: (a) predicted wrist surface clear on
            # both sides, (b) smaller worst residual, (c) larger margin.
            su = None
            ss = None
            if radii:
                for rr_, tag_ in ((r_un, 'u'), (r_sd, 's')):
                    a_ = api.object_aabb()
                    lo_ = np.array(a_['lo'])
                    hi_ = np.array(a_['hi'])
                    ws_ = rr_.get('wrist_solved') or {}
                    mn_ = None
                    for nm_ in ws_:
                        if nm_ not in radii:
                            continue
                        p_ = np.array(ws_[nm_])
                        near_ = np.clip(p_, lo_, hi_)
                        s_ = np.linalg.norm(p_ - near_) - radii[nm_]
                        if mn_ is None or s_ < mn_:
                            mn_ = s_
                    if tag_ == 'u':
                        su = mn_
                    else:
                        ss = mn_
            pick_sd = None
            if su is not None or ss is not None:
                uc = su is not None and su >= 0.02
                sc = ss is not None and ss >= 0.02
                if sc and not uc:
                    pick_sd = True
                elif uc and not sc:
                    pick_sd = False
            if pick_sd is None:
                if ws is not None and (wu is None or ws < wu - 1e-9):
                    pick_sd = True
                elif wu is not None and (ws is None or wu < ws - 1e-9):
                    pick_sd = False
                else:
                    mu = r_un.get('joint_margin_min')
                    ms = r_sd.get('joint_margin_min')
                    if ms is not None and (mu is None or ms > mu):
                        pick_sd = True
                    else:
                        pick_sd = False
            if seed_force:
                pick_sd = True
            if pick_sd:
                r = r_sd
                seed_q_use = seed_q
            else:
                r = r_un
                seed_q_use = {}
            first_step['seed_choice'] = 'seeded' if pick_sd else 'unseeded'
            first_step['seed_surface'] = {'unseeded': su, 'seeded': ss}
        else:
            sq = None
            if seed_q_use:
                sq = seed_q_use
            r = api.reach_box_qp(task_dim=12, targets=targets,
                                 R_body=r_body, orientation=orientation,
                                 dur=0.02, iters=iters,
                                 hand_hold=hand_hold, apply=False,
                                 seed_q=sq)
        if first_step is not None and first_step.get('ik_err') is None:
            first_step['ik_err'] = r.get('ik_err')
            first_step['joint_margin_min'] = r.get('joint_margin_min')
            first_step['joint_margin_joint'] = r.get('joint_margin_joint')
            first_step['joint_margins'] = r.get('joint_margins')
            first_step['solver'] = r.get('solver')
        tq = r.get('upper_target')
        if tq is None:
            nans = nans + 1
            trace.append({'k': k, 't': api.t, 'nan_target': True})
            break
        md_step = max_delta
        wd = None
        wd_pred = None
        cmin = None
        pmin = None
        if wrist_guard:
            a = api.object_aabb()
            lo3 = np.array(a['lo'])
            hi3 = np.array(a['hi'])
            agg = api.arm_geoms()
            cur = {'left': None, 'right': None}
            for body in agg:
                for gg in agg[body]['geoms']:
                    nm_ = gg['name']
                    if nm_ not in radii:
                        continue
                    p3 = np.array(gg['world_pos'])
                    near3 = np.clip(p3, lo3, hi3)
                    s3 = np.linalg.norm(p3 - near3) - radii[nm_]
                    side = 'left' if (nm_ == 'robot0_g27_col'
                                      or nm_ == 'robot0_g28_col') else 'right'
                    if cur[side] is None or s3 < cur[side]:
                        cur[side] = s3
            pred = {'left': None, 'right': None}
            ws = r.get('wrist_solved') or {}
            for nm_ in ws:
                if nm_ not in radii:
                    continue
                p3 = np.array(ws[nm_])
                near3 = np.clip(p3, lo3, hi3)
                s3 = np.linalg.norm(p3 - near3) - radii[nm_]
                side = 'left' if (nm_ == 'robot0_g27_col'
                                  or nm_ == 'robot0_g28_col') else 'right'
                if pred[side] is None or s3 < pred[side]:
                    pred[side] = s3
            wd = cur
            wd_pred = pred
            for side in cur:
                if cur[side] is not None and (cmin is None
                                              or cur[side] < cmin):
                    cmin = cur[side]
            for side in pred:
                if pred[side] is not None and (pmin is None
                                               or pred[side] < pmin):
                    pmin = pred[side]
            if pmin is None:
                pmin = cmin
            if ((cmin is not None and cmin < near_dist)
                    or (pmin is not None and pmin < near_dist)):
                md_step = max_delta_near
        cmd = q.copy()
        bad = False
        dmax = 0.0
        i = 0
        while i < 31:
            d = tq[i] - q[i]
            if not np.isfinite(d):
                bad = True
                break
            c = np.clip(d, -md_step, md_step)
            if np.abs(c) > dmax:
                dmax = np.abs(c)
            cmd[i] = q[i] + c
            i = i + 1
        if bad:
            nans = nans + 1
            trace.append({'k': k, 't': api.t, 'nan_target': True})
            break
        if wrist_guard and ((pmin is not None and pmin < wrist_guard_dist)
                            or (cmin is not None
                                and cmin < live_guard_dist)):
            wrist_abort = True
            wd_min = pmin
            wd_side = wd_pred
            wd_step = k
            trace.append({'k': k, 't': api.t, 'wrist_abort': True,
                          'wrist_surface': wd, 'wrist_pred': wd_pred,
                          'wrist_dist_min': pmin})
            break
        api.step_goal(upper=cmd, nav=nav, dur=0.02)
        st_a = api.observe()
        t_a = api.t
        b_a = api.base_state()
        g = api.geom()
        pe = {}
        worst = None
        for side in targets:
            site = g['sites'].get(side)
            if site is None:
                continue
            t = targets[side]
            v = [site['pos'][0] - t[0], site['pos'][1] - t[1],
                 site['pos'][2] - t[2]]
            e = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
            pe[side] = e
            if worst is None or e > worst:
                worst = e
        base_jump = _dist(b_a['pos'], b['pos'])
        t_jump = t_a - t_b
        reset = (t_jump < -1e-6) or (t_jump > 0.05) or (base_jump > 0.2)
        if reset:
            resets = resets + 1
        q_now = api.upper()
        qm = 0.0
        iq = 0
        while iq < 31:
            dv = q_now[iq] - q[iq]
            if dv < 0:
                dv = -dv
            if dv > qm:
                qm = dv
            iq = iq + 1
        sd = None
        if seed_q_use:
            ut = r.get('upper_target') or []
            for nm_ in seed_q_use:
                if nm_ in api.addr:
                    jj = api.addr[nm_]
                    kk = 0
                    while kk < 31:
                        if api.qpos_addr[kk] == jj:
                            v = ut[kk] - seed_q_use[nm_]
                            if v < 0:
                                v = -v
                            if sd is None or v > sd:
                                sd = v
                            break
                        kk = kk + 1
        trace.append({'k': k, 't_before': t_b,
                      't_after': t_a, 'dt': t_jump,
                      'site_error': pe, 'worst': worst,
                      'base_before': b['pos'], 'base_after': b_a['pos'],
                      'yaw_before': b['yaw_deg'], 'yaw_after': b_a['yaw_deg'],
                      'yaw_err': hold_yaw - b['yaw_deg'],
                      'wrist_surface': wd, 'wrist_pred': wd_pred,
                      'seed_dist': sd, 'q_moved': qm,
                      'base_jump': base_jump,
                      'delta_max': dmax, 'reset': reset,
                      'nav': nav, 'base_err': [dxr, dyr],
                      'targets': targets, 'ik_err': r.get('ik_err'),
                      'margin': r.get('joint_margin_min'),
                      'margin_joint': r.get('joint_margin_joint'),
                      'contacts': (api.contact_detail(limit=40,
                                                      match=contact_match)
                                   if trace_contacts else [])})
        if worst is not None and worst <= stop_err:
            break
        k = k + 1
    b1 = api.base_state()
    st = api.observe()
    ob = st.get('object') or {}
    obj = ob.get('pos')
    b = api.base_state()
    dxv = obj[0] - b['pos'][0]
    dyv = obj[1] - b['pos'][1]
    dd = (dxv * dxv + dyv * dyv) ** 0.5
    fx = dxv / dd
    fy = dyv / dd
    px = -fy
    py = fx
    final_t = {'left': [obj[0] + dx0 * fx + dy0 * px,
                        obj[1] + dx0 * fy + dy0 * py, z0],
               'right': [obj[0] + dx0 * fx - dy0 * px,
                         obj[1] + dx0 * fy - dy0 * py, z0]}
    g = api.geom()
    pe = {}
    final = None
    for side in final_t:
        site = g['sites'].get(side)
        if site is None:
            continue
        t = final_t[side]
        v = [site['pos'][0] - t[0], site['pos'][1] - t[1],
             site['pos'][2] - t[2]]
        e = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
        pe[side] = e
        if final is None or e > final:
            final = e
    hg1 = api.hand_geoms()
    ne = {}
    for body in (hg1 if normals is not None else {}):
        for gg in hg1[body]['geoms']:
            for side in ('left', 'right'):
                if gg['name'] == 'gripper0_%s_g0_col' % side:
                    n1 = np.array(gg['xmat'])[:, 0]
                    tgt_n = np.array(normals[side])
                    tgt_n = tgt_n / np.linalg.norm(tgt_n)
                    d = np.clip(np.dot(tgt_n, n1), -1.0, 1.0)
                    ne[side] = np.degrees(np.arccos(d))
    max_normal = None
    for side in ne:
        if max_normal is None or ne[side] > max_normal:
            max_normal = ne[side]
    target_frame_offset = 0.0
    if max_normal is not None:
        target_frame_offset = 0.0838 * np.sin(np.radians(max_normal))
    base_drift = _dist(b1['pos'], b0['pos'])
    jr = api.joint_ranges()
    wrist_margin = None
    for nm in jr:
        if 'wrist' in nm:
            qq = jr[nm]['q']
            lo = jr[nm]['lo']
            hi = jr[nm]['hi']
            m = qq - lo
            if hi - qq < m:
                m = hi - qq
            if wrist_margin is None or m < wrist_margin:
                wrist_margin = m
    ik_err = (r or {}).get('ik_err') or {}
    solver_residual = None
    for side in ik_err:
        if solver_residual is None or ik_err[side] > solver_residual:
            solver_residual = ik_err[side]
    margin = (r or {}).get('joint_margin_min')
    solver = (r or {}).get('solver')
    ok = True if (final is not None and final <= 0.015
                  and margin is not None and margin >= 0.05
                  and solver == 'mink-qp' and resets == 0
                  and nans == 0) else False
    binding = None
    if resets > 0:
        binding = 'sim_reset'
    elif nans > 0:
        binding = 'nan_target'
    elif final is None:
        binding = 'no_solution'
    elif final > 0.015:
        binding = 'pad_error'
    elif margin is None or margin < 0.05:
        binding = 'joint_margin'
    else:
        binding = 'solver'
    return {'ok': ok, 'pad_error': pe, 'worst_pad_error': final,
            'normal_error_deg': ne, 'base_drift': base_drift,
            'joint_margin_min': margin, 'solver': solver,
            'joint_margin_joint': (r or {}).get('joint_margin_joint'),
            'joint_margins': (r or {}).get('joint_margins'),
            'ik_err': ik_err, 'solver_residual': solver_residual,
            'target_frame_offset': target_frame_offset,
            'wrist_margin': wrist_margin, 'steps': k + 1,
            'wrist_abort': wrist_abort, 'wrist_dist_min': wd_min,
            'wrist_dist_by_side': wd_side, 'wrist_abort_step': wd_step,
            'targets_final': final_t, 'live_object_final': obj,
            'sim_time': api.t, 'resets': resets, 'nans': nans,
            'trace': trace, 'first_step': first_step,
            'binding_constraint': binding}
