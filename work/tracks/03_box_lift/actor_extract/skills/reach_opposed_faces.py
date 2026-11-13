"""reach_opposed_faces -- bimanual opposed-face pre-grasp reach with IK-branch guard.

MECHANISM NAME
    Opposed-face pre-grasp placement: one hand at a time onto the two side
    faces of a box, with the palm pads rotated to face each other, plus an
    IK-branch guard that retries a side through an intermediate point when the
    solver residual is large.

FAILURE SIGNATURE ADDRESSED
    Two independent measured failures.  (a) A simultaneous two-hand reach onto
    the side faces leaves the pads shy of the faces or inside them, because the
    second hand is planned from a stale box measurement and the arms couple
    through the WBC.  (b) The DLS solver occasionally settles in a local
    minimum after a retract+walk recovery and lands the pad past the far side
    of the box (measured left pad x = 1.914 on 03_box_lift) while reporting a
    small palm residual; without a branch guard the very next correction reach
    shoves the box.

MECHANISM
    Read the object fresh.  Build both side targets from that measurement:
    x = pos_x + x_off, y = y-centre +/- (half_y + gap), z = z-centre + dz.
    Reach the left hand to its target only; re-read the object; rebuild the
    targets; reach the right hand only.  If a side's IK error exceeds
    `ik_err_max`, read that hand's measured palm, reach the midpoint between
    the measured palm and the target (`via_dur`), then reach the target again
    (`retry_dur`).  Take the achieved pad sites and the achieved-minus-desired
    offsets from the returned palm residuals: off = pad - (target - residual),
    so later alignment can target a pad position without re-solving the IK.

CONTRACT
    run(api, box=None, gap=0.09, x_off=-0.15, dz=-0.02, dur=2.2,
        ik_err_max=0.09, via_dur=1.0, retry_dur=1.5, R_body=None)
        -> {'status': 'ok'|'no_object'|'no_pad',
            'ik_err': {side: err}, 'via_retry': {side: {...}},
            'pads': {side: [x,y,z]}, 'off': {side: [dx,dy,dz]},
            'targets': {side: [x,y,z]}, 'box': {...}}

    box           object measurement; read live when None
    gap           lateral standoff from each side face
    x_off         x target offset from the object body x
    dz            z target offset from the object z-centre
    dur           per-side reach duration
    ik_err_max    IK residual above which the branch guard fires
    via_dur       duration of the intermediate retry reach
    retry_dur     duration of the retried target reach
    R_body        per-side 3x3 body rotation, dict or None for the default
                  opposed-pad orientation (left flipped about body x)

    preconditions:  object pose and AABB available; base already parked at the
                    standoff; hands may be anywhere in front of the object.
    postconditions: both pads are placed at the opposed-face targets with
                    residual absorbed into `off`; `pads` are measured sites.
    side effects:   commands both arms; no base motion; no finger motion; the
                    object may be nudged if the pads hit it.

SCOPE
    Two-palm side grasps of box-like bodies with an opposed-pad orientation.
    Breaks when the two faces are not plane-parallel about the object y-centre,
    when the object is too wide for the reach, or when the DLS residual stays
    large for both sides (the guard retries once per side only).
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


def _targets(b, gap, x_off, dz):
    yc = 0.5 * (b['lo'][1] + b['hi'][1])
    half_y = 0.5 * (b['hi'][1] - b['lo'][1])
    zc = 0.5 * (b['lo'][2] + b['hi'][2])
    return {'left': np.array([b['pos'][0] + x_off, yc + half_y + gap,
                              zc + dz]),
            'right': np.array([b['pos'][0] + x_off, yc - half_y - gap,
                               zc + dz])}


def run(api, box=None, gap=0.09, x_off=-0.15, dz=-0.02, dur=2.2,
        ik_err_max=0.09, via_dur=1.0, retry_dur=1.5, R_body=None):
    if R_body is None:
        R_body = {'left': [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
                  'right': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}

    def reach_side(s_, tgt, d_):
        # a per-side dict has ndim 0; a bare 3x3 rotation has ndim 2
        if np.ndim(R_body) > 0:
            Rb = R_body
        else:
            Rb = R_body.get(s_)
        return api.reach_box(targets={s_: tgt.tolist()}, sides=[s_],
                             R_body=Rb, dur=d_)

    b = box
    if b is None:
        b = _read_box(api)
    if b is None:
        return {'status': 'no_object', 'pads': None, 'off': None,
                'targets': None, 'box': None, 'ik_err': {}, 'via_retry': {}}
    tg = _targets(b, gap, x_off, dz)
    rl = reach_side('left', tg['left'], dur)
    b = _read_box(api)
    if b is None:
        return {'status': 'no_object', 'pads': None, 'off': None,
                'targets': None, 'box': None, 'ik_err': {}, 'via_retry': {}}
    tg = _targets(b, gap, x_off, dz)
    rr = reach_side('right', tg['right'], dur)

    ike = {'left': (rl.get('ik_err') or {}).get('left', 0.0),
           'right': (rr.get('ik_err') or {}).get('right', 0.0)}
    via_log = {}
    for s_ in ('left', 'right'):
        if ike[s_] > ik_err_max:
            hand = (api.observe().get('object') or {}).get('hand') or {}
            cur = np.asarray(hand.get(s_, tg[s_].tolist()), np.float64)
            via = 0.5 * (cur + tg[s_])
            reach_side(s_, via, via_dur)
            if s_ == 'left':
                rl = reach_side(s_, tg[s_], retry_dur)
            else:
                rr = reach_side(s_, tg[s_], retry_dur)
            via_log[s_] = {'ik_err_before': ike[s_], 'via': via.tolist()}

    pad_d = {'left': (rl.get('pad') or {}).get('left'),
             'right': (rr.get('pad') or {}).get('right')}
    if pad_d['left'] is None or pad_d['right'] is None:
        return {'status': 'no_pad', 'ik_err': ike, 'via_retry': via_log,
                'pads': None, 'off': None, 'targets': tg, 'box': b}
    pads = {'left': np.asarray(pad_d['left']['pos'], np.float64),
            'right': np.asarray(pad_d['right']['pos'], np.float64)}
    off = {}
    for s_, r_, t_ in (('left', rl, tg['left']),
                       ('right', rr, tg['right'])):
        res = (r_.get('palm_residual') or {}).get(s_) or [0.0, 0.0, 0.0]
        off[s_] = pads[s_] - (t_ - np.asarray(res, np.float64))
    return {'status': 'ok', 'ik_err': ike, 'via_retry': via_log,
            'pads': pads, 'off': off, 'targets': tg, 'box': b}
