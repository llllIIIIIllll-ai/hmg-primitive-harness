"""compose -- 03_box_lift task program: two-pad side grasp plus a vertical lift.

This is only glue: it sequences the six reusable skills through api.skill(...)
and keeps the bookkeeping the promoted program logged (grasp attempts, IK
residuals, pad alignment trace, grip evidence, lift samples).  Every physical
action is inside a skill.

Order (identical to the promoted program):
    approach_standoff      walk to the measured standoff behind the front face
    reach_opposed_faces    one hand at a time onto the two side faces, IK guard
    [retract_clear + approach_standoff] on a reach shortfall (recover_pre)
    align_pads_to_faces    measured-pad closed-loop correction
    limit_safe_close       joint-range-clamped close with a contact gate
    [retract_clear + approach_standoff] when the close leaves no grasp
    shoulder_lift_hold     staged stand-up plus shoulder-pitch ratchet
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


def _recover(api, b, extra, retract_dur=1.8, max_it=6):
    """Clear the palms, then re-walk to the measured standoff from the base."""
    api.skill('retract_clear', lo=b['lo'], hi=b['hi'],
              back_dur=1.0, sweep_dur=retract_dur)
    r = api.skill('approach_standoff', box=b, standoff=0.40, closer=extra,
                  max_it=max_it, reread=True, guard=None, tilt_limit=None,
                  check_lateral=False, stop_x=0.05)
    return r['steps'], r['object']


def run(api, **kw):
    log = {}
    b = _read_box(api)
    if b is None:
        return {'error': 'no object'}
    log['box0'] = {'pos': b['pos'].tolist(), 'lo': b['lo'].tolist(),
                   'hi': b['hi'].tolist()}

    # ---- 1. walk to a standoff behind the box front face ---------------
    a = api.skill('approach_standoff', box=b, standoff=0.40,
                  closer=-np.float64(kw.get('extra_standoff', 0.0)),
                  max_it=kw.get('walk_it', 16),
                  guard=kw.get('guard', 0.20), tilt_limit=25.0,
                  check_lateral=True)
    if a['status'] == 'tilt':
        return {'abort': 'walk_tilt', 'log': log}
    log['walk'] = a['steps']
    log['base_after_walk'] = api.base_state()

    # ---- 2/3/4. pre-grasp -> reach check -> pad correction -> close ----
    gap0 = kw.get('pre_gap', 0.09)
    attempts = kw.get('attempts', 3)
    retract_dur = kw.get('retract_dur', 1.8)
    attempt = 0
    closed = False
    pads = None
    off = None
    while attempt < attempts and not closed:
        attempt = attempt + 1
        key = 'attempt%d' % attempt
        pre = api.skill('reach_opposed_faces', gap=gap0, x_off=-0.15,
                        dz=-0.02, dur=kw.get('pre_dur', 2.2))
        if pre['status'] == 'no_object':
            return {'abort': 'object lost', 'log': log}
        if pre['status'] == 'no_pad':
            return {'abort': 'no pad', 'log': log}
        b = pre['box']
        pads = pre['pads']
        off = pre['off']
        log[key] = {'ik_err': pre['ik_err'],
                    'via_retry': pre['via_retry'],
                    'pre_pads': {'left': pads['left'].tolist(),
                                 'right': pads['right'].tolist()},
                    'pre_targets': {'left': pre['targets']['left'].tolist(),
                                    'right': pre['targets']['right'].tolist()},
                    'box_after_pre': {'pos': b['pos'].tolist(),
                                      'lo': b['lo'].tolist(),
                                      'hi': b['hi'].tolist()}}

        # reach check: a correction started from a short reach shoved the box
        short = b['lo'][0] - np.minimum(pads['left'][0], pads['right'][0])
        if short > 0.10 and attempt < attempts:
            extra = np.clip(short - 0.04, 0.0,
                            kw.get('recover_extra', 0.06)).item()
            rec, b = _recover(api, b, extra, retract_dur)
            log[key]['recover_pre'] = rec
            log[key]['recover_extra'] = extra
            continue

        al = api.skill('align_pads_to_faces', pads=pads, off=off,
                       iters=kw.get('corr_it', 4),
                       dur=kw.get('corr_dur', 1.8))
        log[key]['corr'] = al['log']
        pads = al['pads']
        off = al['off']

        cl = api.skill('limit_safe_close',
                       grip_angle=kw.get('grip_angle', 1.0),
                       grip_dur=kw.get('grip_dur', 1.2))
        log[key]['after_close'] = cl['state']
        log[key]['grip'] = {'angle': cl['angle'],
                            'hand_limit_min_abs': cl['hand_limit_min_abs'],
                            'cmd_hand_maxabs': cl['cmd_hand_maxabs']}
        if cl['closed']:
            closed = True
            break
        b = _read_box(api)
        if b is None:
            return {'abort': 'object lost', 'log': log}
        rec, b = _recover(api, b, 0.0, retract_dur)
        log[key]['recover'] = rec

    if not closed:
        return {'abort': 'no grasp contacts', 'log': log}

    # ---- 5. lift: stand up gently, then raise the palms ----------------
    cmd = api.commanded()
    if cmd is None:
        return {'abort': 'no commanded', 'log': log}
    lf = api.skill('shoulder_lift_hold', commanded=cmd, z_start=None,
                   stand_height1=0.82, stand_height2=0.90,
                   stand_dur=kw.get('stand_dur', 1.5),
                   stand_dur2=kw.get('stand_dur2', 1.5),
                   settle_dur=kw.get('settle_dur', 0.5),
                   iters=kw.get('lift_it', 8),
                   pitch_step=kw.get('pitch_step', 0.06),
                   lift_dur=kw.get('lift_dur', 1.6))
    if lf['abort'] is not None:
        log['abort'] = lf['abort']
    log['lifts'] = lf['lifts']
    log['final_geom'] = lf['final_geom']
    log['final_state'] = lf['final_state']
    return log
