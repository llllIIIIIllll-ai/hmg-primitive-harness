"""align_pads_to_faces -- measured-pad closed-loop alignment onto the two side faces.

MECHANISM NAME
    Measured-pad face alignment: iterate the measured palm pads onto both side
    faces, one fresh object read per iteration, with a per-side clearance
    freeze that forbids further inward motion on a side already at its face
    while the other side keeps correcting, and a reduced step near the face.

FAILURE SIGNATURE ADDRESSED
    A single planned correction reach does not put both pads on the faces: the
    base and arms drift between the plan and the execution, so the pads arrive
    1-3 cm off.  Correcting both sides with full-size steps was measured to
    push a pad 15-17 mm INSIDE the face and shove the box (03_box_lift v4
    seeds 7/15), while a global stop when either side touched skipped the far
    hand's needed correction and left 2 contacts on close.  Either failure
    ends as a one-sided or empty grasp that slips at the first lift.

MECHANISM
    Per iteration: read the object; set the desired pad centres at
    x = lo_x + x_margin, y = hi_y + y_margin (left) and lo_y - y_margin
    (right), z = object z-centre.  Measure each pad's face clearance
    clr = (pad_y - face_off) - hi_y for the left and lo_y - (pad_y + face_off)
    for the right.  If clr < freeze_clr the side is frozen: its inward y
    component is clipped to non-negative (left) or non-positive (right) while x
    and z keep correcting.  Cap the commanded delta: x by cap_x, z by cap_z,
    and y by capy_near when clr <= near_thr else capy_far.  Convert the delta
    to a target with tg = desired - off (absorbing the measured pad residual),
    reach both sides, and take the newly measured pads.  Stop when every
    clipped delta is within its convergence threshold.

CONTRACT
    run(api, pads, off, iters=4, dur=1.8, x_margin=0.030, y_margin=0.0237,
        face_off=0.0217, freeze_clr=0.005, near_thr=0.020,
        capy_far=0.030, capy_near=0.012, cap_x=0.05, cap_z=0.04,
        done_x=0.030, done_y=0.010, done_z=0.012, R_body=None)
        -> {'status': 'ok'|'object_lost',
            'pads': {side: [x,y,z]}, 'off': {...}, 'log': [...], 'box': {...}}

    pads          measured palm-pad sites at entry
    off           pad-to-target residual offsets from the placement skill
    iters         iteration budget
    dur           duration of each two-side correction reach
    x_margin      desired pad x offset from the front face
    y_margin      desired pad y offset outside each side face
    face_off      pad half-thickness (clearance probe offset)
    freeze_clr    face clearance below which a side is frozen inward
    near_thr      clearance below which the y step is capped at capy_near
    cap_*         per-axis command caps, metres
    done_*        per-axis convergence thresholds, metres
    R_body        per-side 3x3 body rotation for the reach

    preconditions:  object pose/AABB available; pads placed near the faces;
                    `off` measured on the same base pose.
    postconditions: both pads are on or just outside their faces with the
                    inward component of a frozen side non-positive, or the
                    iteration budget expires; the returned pads are measured.
    side effects:   commands both arms each iteration; can nudge the object by
                    up to a millimetre-scale amount per iteration; no finger
                    or base motion.

SCOPE
    Opposed two-pad grasps.  Breaks when the object moves more than ~5 cm
    between iterations (the target is always rebuilt from the live AABB, so
    smaller drift is absorbed), when the reach residual is larger than the
    caps, or when a third contact (e.g. a wall) blocks the pad.
"""


def _read_box(api):
    st = api.observe()
    b = st.get('object') or {}
    if not b.get('pos'):
        return None
    a = None
    for o in api.scene_objects(detail=False):
        if o.get('name') == b.get('name'):
            a = o
    if a is None:
        return None
    return {'pos': np.asarray(b['pos'], np.float64),
            'lo': np.asarray(a['aabb_lo'], np.float64),
            'hi': np.asarray(a['aabb_hi'], np.float64)}


def _pads_of(r, fallback):
    p = r.get('pad') or {}
    out = {}
    for s_ in ('left', 'right'):
        if p.get(s_) is not None:
            out[s_] = np.asarray(p[s_]['pos'], np.float64)
        else:
            out[s_] = fallback[s_]
    return out


def run(api, pads, off, iters=4, dur=1.8, x_margin=0.030, y_margin=0.0237,
        face_off=0.0217, freeze_clr=0.005, near_thr=0.020,
        capy_far=0.030, capy_near=0.012, cap_x=0.05, cap_z=0.04,
        done_x=0.030, done_y=0.010, done_z=0.012, R_body=None):
    if R_body is None:
        R_body = {'left': [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
                  'right': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    pads = {'left': np.asarray(pads['left'], np.float64),
            'right': np.asarray(pads['right'], np.float64)}
    it_log = []
    tg = {}
    b = None
    it = 0
    while it < iters:
        b = _read_box(api)
        if b is None:
            return {'status': 'object_lost', 'pads': pads, 'off': off,
                    'log': it_log, 'box': None}
        zc = 0.5 * (b['lo'][2] + b['hi'][2])
        desired = {'left': np.array([b['lo'][0] + x_margin,
                                     b['hi'][1] + y_margin, zc]),
                   'right': np.array([b['lo'][0] + x_margin,
                                      b['lo'][1] - y_margin, zc])}
        d = {}
        done = True
        frozen = {'left': False, 'right': False}
        clr_now = []
        for s_ in ('left', 'right'):
            if s_ == 'left':
                clr = ((pads[s_][1] - np.float64(face_off))
                       - b['hi'][1]).item()
            else:
                clr = (b['lo'][1]
                       - (pads[s_][1] + np.float64(face_off))).item()
            clr_now.append(clr)
            dv = desired[s_] - pads[s_]
            if clr < freeze_clr:
                frozen[s_] = True
                if s_ == 'left':
                    dv[1] = np.maximum(dv[1], 0.0)
                else:
                    dv[1] = np.minimum(dv[1], 0.0)
            capy = capy_far if clr > near_thr else capy_near
            dv = np.array([np.clip(dv[0], -cap_x, cap_x),
                           np.clip(dv[1], -capy, capy),
                           np.clip(dv[2], -cap_z, cap_z)])
            d[s_] = dv
            if (np.abs(dv[0]) > done_x or np.abs(dv[1]) > done_y
                    or np.abs(dv[2]) > done_z):
                done = False
            tg[s_] = desired[s_] - off[s_]
        it_log.append({'pads': {'left': pads['left'].tolist(),
                                'right': pads['right'].tolist()},
                       'delta': {'left': d['left'].tolist(),
                                 'right': d['right'].tolist()},
                       'target': {'left': tg['left'].tolist(),
                                  'right': tg['right'].tolist()},
                       'clr': clr_now,
                       'frozen': {'left': frozen['left'],
                                  'right': frozen['right']},
                       'box': b['pos'].tolist()})
        if done:
            break
        r = api.reach_box(targets={'left': tg['left'].tolist(),
                                   'right': tg['right'].tolist()},
                          sides=['left', 'right'], R_body=R_body, dur=dur)
        pads = _pads_of(r, pads)
        it = it + 1
    return {'status': 'ok', 'pads': pads, 'off': off, 'log': it_log, 'box': b}
