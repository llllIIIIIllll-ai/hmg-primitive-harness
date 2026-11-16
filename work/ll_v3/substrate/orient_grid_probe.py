def _radii(api):
    ag = api.arm_geoms()
    out = {}
    for body in ag:
        for gg in ag[body]['geoms']:
            nm = gg['name']
            if (nm == 'robot0_g27_col' or nm == 'robot0_g28_col'
                    or nm == 'robot0_g34_col' or nm == 'robot0_g35_col'):
                sz = gg.get('local_size') or [0.0]
                tp = gg.get('type') or ''
                rad = sz[0]
                if tp == 'box' or tp == 'cylinder':
                    rad = (sz[0] * sz[0] + sz[1] * sz[1]
                           + sz[2] * sz[2]) ** 0.5
                out[nm] = rad
    return out


def run(api, dx=-0.0838, dy=0.07, z=0.94,
        pitches=(-30, -15, 0, 15, 30), rolls=(-30, -15, 0, 15, 30), **kw):
    """Read-only hand-orientation grid at the grasp point: pitch (tilt the
    approach axis f up/down) x roll (about f), 15 deg steps, +/-30 deg.
    Seeded QP solve per orientation; reports predicted wrist surface."""
    st = api.observe()
    obj = (st.get('object') or {}).get('pos')
    b = api.base_state()
    dxv = obj[0] - b['pos'][0]
    dyv = obj[1] - b['pos'][1]
    dd = (dxv * dxv + dyv * dyv) ** 0.5
    fx = dxv / dd
    fy = dyv / dd
    px = -fy
    py = fx
    tgt = {'left': [obj[0] + dx * fx + dy * px,
                    obj[1] + dx * fy + dy * py, z],
           'right': [obj[0] + dx * fx - dy * px,
                     obj[1] + dx * fy - dy * py, z]}
    g = api.geom()
    hg = api.hand_geoms()
    q0 = np.asarray(api.raw[1].qpos[3:7])
    Rb = api._quat_to_mat(q0)
    a = api.object_aabb()
    lo = np.array(a['lo'])
    hi = np.array(a['hi'])
    radii = _radii(api)
    rows = []
    for pa in pitches:
        for ro in rolls:
            ar = np.radians(pa)
            br = np.radians(ro)
            f = np.array([fx * np.cos(ar), fy * np.cos(ar), np.sin(ar)])
            f = f / np.linalg.norm(f)
            p = np.array([px, py, 0.0])
            p = p - np.dot(p, f) * f
            p = p / np.linalg.norm(p)
            r_body = {}
            for side in ('left', 'right'):
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
                n0 = -p if side == 'left' else p
                n0 = n0 * np.cos(br) + np.cross(f, n0) * np.sin(br)
                n0 = n0 - np.dot(n0, f) * f
                n0 = n0 / np.linalg.norm(n0)
                B_des = np.column_stack([f, n0, np.cross(f, n0)])
                R_des = B_des @ B_local.T
                U, S, Vt = np.linalg.svd(R_des)
                R_des = U @ Vt
                Rbod = Rb.T @ R_des @ np.asarray(api.R_site0[side]).T
                U, S, Vt = np.linalg.svd(Rbod)
                r_body[side] = (U @ Vt).tolist()
            ip = api.ik_preview(targets=tgt, R_body=r_body,
                                orients=('keep',), iters=150,
                                sides=('left', 'right'))
            seed = {}
            for row in (ip.get('rows') or []):
                qn = row.get('q_named') or {}
                for kk in qn:
                    seed[kk] = qn[kk]
            r = api.reach_box_qp(task_dim=12, targets=tgt, R_body=r_body,
                                 orientation='keep', dur=0.02,
                                 apply=False, seed_q=seed)
            ws = r.get('wrist_solved') or {}
            surf = {}
            for nm in ws:
                if nm not in radii:
                    continue
                p3 = np.array(ws[nm])
                near = np.clip(p3, lo, hi)
                surf[nm] = np.linalg.norm(p3 - near) - radii[nm]
            rows.append({'pitch': pa, 'roll': ro,
                         'ik_err': r.get('ik_err'), 'surf': surf})
    return {'ok': True, 'targets': tgt, 'rows': rows}
