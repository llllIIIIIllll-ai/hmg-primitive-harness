"""approach_standoff -- measured-standoff base servoing with approach guards.

MECHANISM NAME
    Measured-standoff base servoing (closed-loop whole-body approach to a pose
    expressed relative to the CURRENT object AABB) with a fingertip guard and a
    tilt guard.

FAILURE SIGNATURE ADDRESSED
    `move_base` tracks only ~50% of a commanded displacement and yaw degrades
    with translation, so an open-loop walk overshoots or stalls short of a
    sampled standoff (03_box_lift; also the measured 09_drill_pick boundary).
    A long approach with the arms slightly forward can jam the fingertips into
    the object front face before any grasp starts, and a fast approach can tip
    the base.  Both are measured failures, not hypotheticals.

MECHANISM
    Each iteration reads the base pose, computes the signed x error to
    x = lo_x - standoff + closer, the lateral error to the object y-centre and
    the yaw error to 0, and issues one bounded `move_base` burst whose duration
    is proportional to the x error.  Optionally the object is re-read after
    every burst and the next target is taken from the fresh AABB.  The
    fingertip guard stops the burst sequence (does not move any more) once a
    palm x is within `guard` of the front face; the tilt guard aborts the walk
    when the base tilt exceeds `tilt_limit` degrees.

CONTRACT
    run(api, box=None, standoff=0.40, closer=0.0, max_it=16, reread=False,
        guard=0.20, tilt_limit=25.0, check_lateral=True,
        tol_x=0.04, tol_y=0.06, tol_yaw=6.0, stop_x=0.05,
        vx_gain=0.7, vx_clip=(0.20, 0.30), vy_gain=0.7, vy_clip=0.18,
        yaw_gain=0.03, yaw_clip=0.4, dur_gain=0.33, dur_clip=(0.2, 0.6))
        -> {'status': 'ok'|'fingertip_guard'|'tilt'|'no_object',
            'steps': [...], 'object': {'pos','lo','hi'}|None}

    box           object measurement {'pos','lo','hi'}; read live when None
    standoff      distance behind the front face lo_x where the base stops
    closer        signed offset toward the object (positive = closer)
    max_it        burst budget
    reread        re-read the object after every burst; target from fresh AABB
    guard         palm-x clearance from the front face; None disables the guard
    tilt_limit    tilt abort in degrees; None disables the check
    check_lateral finish only when |ex|<tol_x and |ey|<tol_y and
                  |eyaw|<tol_yaw; when False finish as soon as ex < stop_x
    stop_x        x-only finish threshold used when check_lateral is False

    preconditions:  an object with a live pose and world AABB; a base able to
                    walk; hands clear of the object unless guard is enabled.
    postconditions: base within tolerance of the standoff, or status says why
                    the walk stopped; when guard is enabled no palm is ever
                    commanded inside (lo_x - guard).
    side effects:   moves the base and the arms through the WBC; changes
                    episode time; no grasp state is touched.

SCOPE
    Any task with a box-like body whose AABB is stable and whose approach is
    along -x of the front face.  Breaks when the object moves during the walk
    and reread is False, when the approach axis differs, when the base cannot
    track (> 2x standoff error), or when the lateral finish corridor is not
    clear.
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


def run(api, box=None, standoff=0.40, closer=0.0, max_it=16, reread=False,
        guard=0.20, tilt_limit=25.0, check_lateral=True,
        tol_x=0.04, tol_y=0.06, tol_yaw=6.0, stop_x=0.05,
        vx_gain=0.7, vx_clip=(0.20, 0.30), vy_gain=0.7, vy_clip=0.18,
        yaw_gain=0.03, yaw_clip=0.4, dur_gain=0.33, dur_clip=(0.2, 0.6)):
    b = box
    if b is None:
        b = _read_box(api)
    if b is None:
        return {'status': 'no_object', 'steps': [], 'object': None}

    steps = []
    status = 'ok'
    yc = 0.5 * (b['lo'][1] + b['hi'][1])
    i = 0
    while i < max_it:
        bs = api.base_state()
        ex = ((b['lo'][0] - np.float64(standoff))
              + np.float64(closer)) - bs['pos'][0]
        if check_lateral:
            ey = yc - bs['pos'][1]
            eyaw = 0.0 - bs['yaw_deg']
            if (np.abs(ex) < tol_x and np.abs(ey) < tol_y
                    and np.abs(eyaw) < tol_yaw):
                break
        else:
            if ex < stop_x:
                break
        if guard is not None:
            hh = (api.observe().get('object') or {}).get('hand') or {}
            pmax = -9.0
            for side_ in hh:
                if hh[side_][0] > pmax:
                    pmax = hh[side_][0]
            if pmax > b['lo'][0] - np.float64(guard):
                steps.append({'stop': 'fingertip_guard', 'palm_x': pmax})
                status = 'fingertip_guard'
                break
        if check_lateral and np.abs(ex) <= 0.05:
            vx = np.float64(0.0)
        else:
            vx = np.sign(ex) * np.clip(vx_gain * np.abs(ex),
                                       vx_clip[0], vx_clip[1])
        vy = np.clip(vy_gain * (yc - bs['pos'][1]), -vy_clip, vy_clip)
        wz = np.clip(yaw_gain * (0.0 - bs['yaw_deg']), -yaw_clip, yaw_clip)
        dur = np.clip(np.abs(ex) / dur_gain, dur_clip[0], dur_clip[1])
        r = api.move_base(x=vx.item(), y=vy.item(), yaw=wz.item(),
                          dur=dur.item())
        if reread:
            nb = _read_box(api)
            steps.append({'base': api.base_state()['pos'],
                          'box': nb['pos'].tolist()})
            b = nb
        else:
            b2 = api.base_state()
            steps.append({'cmd': [vx.item(), vy.item(), wz.item(), dur.item()],
                          'moved': r['moved_xy'], 'base': b2['pos'],
                          'yaw': b2['yaw_deg'], 'tilt': b2['tilt_deg']})
            if tilt_limit is not None and b2['tilt_deg'] > tilt_limit:
                status = 'tilt'
                break
        i = i + 1
    return {'status': status, 'steps': steps, 'object': b}
