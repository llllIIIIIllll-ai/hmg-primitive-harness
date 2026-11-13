"""press_until_joint_stop: object-state-watched press along one axis.

MECHANISM
    Servo the fingertip inward in small increments along a fixed world axis,
    after every reach read the TARGET OBJECT'S OWN joint coordinate, and stop
    as soon as that coordinate passes ``q_stop`` (the object has bottomed out)
    or the step cap is spent.  Then re-command the last upper-body goal for a
    settle interval so the press is sustained while the balance controller
    settles, and report the object's final state.

FAILURE SIGNATURE IT REMOVES
    An open-loop or fixed-duration press cannot tell whether the object was
    actually moved: it either stops short of contact or keeps loading the arm
    against the stop.  Reading the object's own joint is the ground truth for
    how far it moved, independent of the arm's own joint readings.  Measured
    on 02_push_button: the button slide joint has 0.015 m of travel (nominal
    [-0.015, 0]); the loop steps the fingertip 0.003 m at a time and the joint
    progresses 0 -> -0.0139..-0.0142 on seeds 1,2,3,5,7.  On these five seeds
    the armed stop (q_stop=-0.0145, the promoted value) is NOT reached and the
    press ends by the max_steps cap at q=-0.0139..-0.0142, i.e. against the
    end of travel; the object-state watch is what measures that.

CONTRACT
    preconditions
      - servo_fingertip_to_point is registered and the target is reachable;
      - the object is a movable body whose name contains ``object_match`` and
        whose motion is readable as a joint coordinate (slide/hinge);
      - the fingertip starts on the stand-off side of the object face.
    postconditions
      - the object joint q <= q_stop, or ``max_steps`` increments were spent;
      - the last commanded upper-body vector was held for ``settle_dur``;
      - returns the final object joint reading, pose, contact counts and the
        last observed state.
    side effects
      - commands the 31-D upper body (through the inner servo) and advances
        sim time by the step count and the settle interval;
      - presses the object physically (contact forces);
      - one bodies() read of the object after every commanded reach and after
        every step.
    args
      side           hand used
      target0        stand-off world point the fingertip converged on
      step           per-step world displacement (press axis and step size)
      max_steps      cap on press increments
      q_stop         stop when the object joint q <= q_stop
      object_match   body-name substring of the pressed object
      joint_index    joint of that body used as the stop/watch signal
      servo_gain/servo_dur/servo_iters/servo_tol
                     parameters forwarded to servo_fingertip_to_point
      tip_link/mesh_tag/tip_half
                     fingertip identification forwarded to the servo
      object_token   body-name token counted as an object contact
      support_token  body-name token counted as a hand-support contact
      contact_limit  api.contact_pairs cap
      settle_dur     hold time on the last command after the press (s)
    returns
      {'button_q', 'button_pos', 'btn_contacts', 'panel_contacts',
       'trace', 'observe', 'base'}
"""
TIP_HALF = 0.0334


def _tip(api, side, tip_link, mesh_tag, tip_half):
    hg = api.hand_geoms()
    for b in hg:
        if tip_link in b and ('_%s_' % side) in b:
            for gg in hg[b]['geoms']:
                if mesh_tag in gg['name']:
                    R = np.array(gg['xmat'])
                    return np.array(gg['world_pos']) + tip_half * R[:, 2]
    return None


def _target(api, match, joint_index):
    b = api.bodies(match=match)[0]
    q = b['joints'][joint_index]['q'] if b['joints'] else None
    return np.array(b['pos']), q


def _counts(api, object_token, support_token, limit):
    nb = 0
    npn = 0
    for c in api.contact_pairs(limit=limit):
        nm = c['b1'] + ' ' + c['b2']
        if object_token in nm:
            nb += 1
        elif support_token in nm and 'hand' in nm:
            npn += 1
    return nb, npn


def run(api, side='right', target0=(0.0, 0.0, 0.0),
        step=(0.003, 0.0, 0.0), max_steps=26, q_stop=-0.0145,
        object_match='button', joint_index=0,
        servo_gain=0.7, servo_dur=0.45, servo_iters=2, servo_tol=0.008,
        tip_link='index_1', mesh_tag='_vis', tip_half=TIP_HALF,
        object_token='control_panel_button',
        support_token='control_panel_main_body', contact_limit=60,
        settle_dur=0.6):
    trace = []
    tgt = np.array(target0)
    q = None
    nb = 0
    npn = 0
    for j in np.arange(max_steps):
        tgt = tgt + np.array(step)
        s = api.skill('servo_fingertip_to_point', side=side, target=tgt,
                      gain=servo_gain, dur=servo_dur, max_iters=servo_iters,
                      tol=servo_tol, tip_link=tip_link, mesh_tag=mesh_tag,
                      tip_half=tip_half, watch_match=object_match,
                      watch_joint=joint_index, phase='press', tag=j.item())
        for rec in s['trace']:
            trace.append(rec)
        tip = _tip(api, side, tip_link, mesh_tag, tip_half)
        objp, q = _target(api, object_match, joint_index)
        nb, npn = _counts(api, object_token, support_token, contact_limit)
        bs = api.base_state()
        trace.append({'phase': 'press_end', 'j': j.item(),
                      'tgt': tgt.round(4).tolist(),
                      'tip': tip.round(4).tolist(),
                      'btn': objp.round(4).tolist(), 'q': q,
                      'btn_contacts': nb, 'panel_contacts': npn,
                      'base': bs['pos'], 'yaw': bs['yaw_deg']})
        if q is not None and q <= q_stop:
            break
    # Hold the last commanded upper-body vector so the press is sustained
    # while balance settles.
    api.step_goal(upper=api.commanded(), dur=settle_dur)
    objp, q = _target(api, object_match, joint_index)
    nb, npn = _counts(api, object_token, support_token, contact_limit)
    return {'button_q': q, 'button_pos': objp.round(4).tolist(),
            'btn_contacts': nb, 'panel_contacts': npn,
            'trace': trace, 'observe': api.observe(),
            'base': api.base_state()}
