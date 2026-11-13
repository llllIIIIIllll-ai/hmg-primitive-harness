"""walk_to_reach: measured-gap base approach that puts a target in arm reach.

MECHANISM
    Base-locomotion reach gating.  A short scripted set of walk phases closes
    most of the approach distance; a feedback loop then keeps issuing forward
    bursts while the MEASURED target body is still farther away than a
    standoff gap along the approach axis.  Termination comes from the target's
    own measured pose, not from a fixed step count.

FAILURE SIGNATURE IT REMOVES
    A fixed walk script stops the base too far from the manipulation target,
    so the arm's position-only IK saturates with a large residual and the
    fingertip servo never reaches the target.  Measured on 02_push_button:
    the three-phase script leaves the button about 0.53 m ahead of the base
    on seed 1 (trace walk_extra k=0, gapx 0.53), beyond the arm's comfortable
    reach; one feedback burst closes it to about 0.41 m.

CONTRACT
    preconditions
      - the target body exists and its name contains ``target_match``;
      - the target is roughly ahead of the base along ``axis`` (+x here);
      - api.move_base is available and the base can walk on the floor.
    postconditions
      - the base was driven until
        (target_pos[axis] - base_pos[axis]) < gapx_max, or ``max_extra``
        bursts were spent;
      - returns the last measured target body pose and its joint reading.
    side effects
      - commands base motion (the 3 nav dof) and advances sim time by the
        phase and burst durations;
      - performs read-only bodies()/base_state() reads (bodies() runs
        mj_forward) after each phase and burst.
    args
      target_match  body-name substring of the manipulation target
      gapx_max      stop once the measured gap along ``axis`` is below it
      phases        list of (dx, dy, dur) scripted walk commands
      burst_x/y/dur one forward feedback burst
      max_extra     cap on feedback bursts
      axis          0/1/2 world axis used for the gap test
      joint_index   joint of the target body reported back (0 = button slide)
    returns
      {'target_pos': [x,y,z], 'target_q': float|None, 'trace': [...]}
"""


def _target(api, match, joint_index):
    """Measured pose and first-joint reading of the matched body."""
    b = api.bodies(match=match)[0]
    q = b['joints'][joint_index]['q'] if b['joints'] else None
    return np.array(b['pos']), q


def run(api, target_match='button', gapx_max=0.44,
        phases=((0.3, 0.0, 1.2), (0.3, 0.05, 0.5), (0.25, 0.02, 0.4)),
        burst_x=0.22, burst_y=0.02, burst_dur=0.4, max_extra=6,
        axis=0, joint_index=0):
    trace = []
    for ph in phases:
        w = api.move_base(x=ph[0], y=ph[1], yaw=0.0, dur=ph[2])
        trace.append({'phase': 'walk', 'cmd': [ph[0], ph[1], ph[2]],
                      'moved': w['moved_xy'],
                      'base': api.base_state()['pos']})
    for k in np.arange(max_extra):
        tp, _tq = _target(api, target_match, joint_index)
        base = api.base_state()['pos']
        gap = tp[axis] - base[axis]
        if gap < gapx_max:
            break
        w = api.move_base(x=burst_x, y=burst_y, yaw=0.0, dur=burst_dur)
        trace.append({'phase': 'walk_extra', 'k': k.item(),
                      'gapx': np.round(gap, 3).item(),
                      'moved': w['moved_xy'],
                      'base': api.base_state()['pos']})
    tp, tq = _target(api, target_match, joint_index)
    return {'target_pos': tp.tolist(), 'target_q': tq, 'trace': trace}
