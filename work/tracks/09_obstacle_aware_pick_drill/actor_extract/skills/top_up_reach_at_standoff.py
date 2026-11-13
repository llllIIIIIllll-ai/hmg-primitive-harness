def run(api, **kw):
    """Close a small reach deficit by one-axis base top-up at a stand-off pose.

    Failure signature removed: a grasp whose planned target lies a few cm
    beyond the arm's reach from the shoulder.  The hand servo then stalls
    short of the target; a later depth assist or a larger servo budget does
    not recover it, and near grasp depth any correction move risks striking
    the object.  Here the base is advanced *before* the final approach, while
    the hand is still at stand-off distance and translation is free.

    Mechanism: measure the shoulder-to-target distance, and while it exceeds
    `reach_max` issue one closed-loop base burst (delegated to the catalogued
    `walk_base_to_coordinate`: read the base, burst until the measured
    coordinate reaches the target, return the base it stopped at), then
    re-measure.  Each burst, the base state, the object pose and the hand
    site are recorded so a caller can verify that the object did not move.

    Contract
      signature: run(api, target, reach_max=0.32, creep=0.20, dur=0.5,
                     dx=0.03, axis=0, max_nudges=3, side='left',
                     shoulder_off=(0.02, 0.10, 0.29))
      preconditions: the hand is parked at a stand-off pose, not yet at
        grasp depth; `target` is the planned grasp point in world
        coordinates; `walk_base_to_coordinate` is registered in the calling
        process; `reach_max` is calibrated for this robot/scene (on 09:
        0.295-0.313 converges, 0.336-0.344 stalls, 0.32 separates them).
      postconditions: returns {'dist', 'dist_after', 'nudges', 'nudge_log',
        'bursts', 'shoulder'}; the base has advanced by at most
        max_nudges * (one burst) along `axis`; no other command is issued.
      side effects: base pose changes toward the object; the object pose is
        only read (verified unchanged to 4 decimals on 09 seeds 4-5).
    """
    pfin = kw.get('target')
    reach_max = kw.get('reach_max', 0.32)
    creep = kw.get('creep', 0.20)
    dur = kw.get('dur', 0.5)
    dx = kw.get('dx', 0.03)
    axis = kw.get('axis', 0)
    max_nudges = kw.get('max_nudges', 3)
    side = kw.get('side', 'left')
    off = kw.get('shoulder_off', (0.02, 0.10, 0.29))

    def measure():
        b = api.base_state()
        sh = np.array([b['pos'][0] + off[0], b['pos'][1] + off[1],
                       b['pos'][2] + off[2]])
        d = math.sqrt((pfin[0] - sh[0]) ** 2 + (pfin[1] - sh[1]) ** 2
                      + (pfin[2] - sh[2]) ** 2)
        return d, b, sh

    need, b, sh = measure()
    first = np.round(need, 3).item()
    nlog = []
    bursts = []
    n = 0
    while need > reach_max and n < max_nudges:
        b0 = np.asarray(api.base_state()['pos'], np.float64)
        o0 = np.asarray(api.observe()['object']['pos'], np.float64)
        p0 = np.asarray(api.geom()['sites'][side]['pos'], np.float64)
        r = api.skill('walk_base_to_coordinate', axis=axis,
                      target=b0[axis] + dx, v=creep, lat=0.0, yaw=0.0,
                      dur=dur, max_bursts=2)
        b = api.base_state()
        o1 = np.asarray(api.observe()['object']['pos'], np.float64)
        p1 = np.asarray(api.geom()['sites'][side]['pos'], np.float64)
        sh = np.array([b['pos'][0] + off[0], b['pos'][1] + off[1],
                       b['pos'][2] + off[2]])
        need = math.sqrt((pfin[0] - sh[0]) ** 2 + (pfin[1] - sh[1]) ** 2
                         + (pfin[2] - sh[2]) ** 2)
        nlog.append({'k': n + 1,
                     'base': np.round(b0, 4).tolist(),
                     'base_after': np.round(np.array(b['pos']), 4).tolist(),
                     'dbase': np.round(np.array(b['pos']) - b0,
                                       4).tolist(),
                     'palm': np.round(p0, 4).tolist(),
                     'palm_after': np.round(p1, 4).tolist(),
                     'obj': np.round(o0, 4).tolist(),
                     'obj_after': np.round(o1, 4).tolist(),
                     'dobj': np.round(o1 - o0, 4).tolist(),
                     'need': np.round(need, 3).item()})
        bursts.append({'bursts': r['bursts'],
                       'nread': r['trace'].__len__()})
        n = n + 1
    return {'shoulder': np.round(sh, 3).tolist(), 'dist': first,
            'dist_after': np.round(need, 3).item(), 'nudges': n,
            'nudge_log': nlog,
            'bursts': [q['bursts'] for q in bursts],
            'nread': [q['nread'] for q in bursts]}
