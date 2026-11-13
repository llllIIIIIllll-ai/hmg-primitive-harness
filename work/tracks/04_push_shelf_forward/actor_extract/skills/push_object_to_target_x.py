"""Burst whole-body push of a rolling leader to a world-x target (push).

MECHANISM NAME
    push_object_to_target_x -- drive the body forward at a fixed nav speed in
    discrete bursts, steering with steer_behind_leader, until the LEADER's
    measured world x reaches a target threshold, with an optional
    command-preserving settle between bursts that turns the leader's measured
    coast into travel.

FAILURE SIGNATURE IT ADDRESSES
    Measured on 04_push_shelf_forward: open-loop walking never stops at a
    sampled floor target, and the cart keeps rolling after the push ends
    (coast 0.07-0.13 m), so a target reached by command count is missed by
    up to the coast distance.  A single high-speed approach also arrives too
    fast to stop on the sampled x (the cart overshot and the residual became
    lateral).  What works is three scales of the same burst: fast until a
    long lead, slow until a short lead, fine until a very short lead, where
    each lead is measured from the LEADER pose after the previous burst.  The
    settle converts the coast into a re-measurable displacement instead of an
    uncontrolled overshoot, and because the settle re-sends the last upper
    command (step_goal(upper=commanded)) it does not move the arms, which is
    what keeps the box on the cart's top board (16 shelf-box contacts at the
    end in every run; x_box = x_shelf - 0.019 throughout).

THE MECHANISM (what it does physically)
    Repeat up to max_bursts times:
      measure the scene (measure_scene_state);
      break if leader_x >= target_x - lead (stop short by the coast / braking
      distance);
      break if leader_x > target_x (overshoot guard, only reachable with a
      negative lead; with lead >= 0 the previous test subsumes it);
      ask steer_behind_leader for (y, yaw);
      api.move_base(x=vx, y=..., yaw=..., dur=dur)  (the whole body walks
      into the leader behind the held arm pose -- this is the push);
      if settle_dur > 0: hold_commanded_pose(dur=settle_dur)  (free coast);
      measure again and record the post-burst state.
    ``move_base`` re-commands the measured upper joints every control step,
    so the arm stays extended and the push is the body, not a finger press;
    contact is allowed to flicker, which is what the measured push does.

SIGNATURE (named params + defaults)
    run(api, **kwargs)
      target_x     float, REQUIRED (no default) leader world x to stop at
      vx           float, default 0.25  forward nav command per burst
      dur          float, default 1.0   burst length (s)
      lead         float, default 0.55  stop when leader_x >= target_x - lead
      max_bursts   int,   default 18    hard cap on bursts
      settle_dur   float, default 0.0   command-preserving coast between
                                        bursts (0 = no settle at all)
      ref_y        float, default None  lateral reference for the steer law
      ky, kb, eb_soft, cy_max, kyw, cw_max   steer-law params, defaults
                                       1.0, 0.8, 0.0, 0.08, 1.0, 0.25
      leader       str,   default 'shelf'  scene-object key of the leader
      phase        str,   default 'push'   trace label

PRECONDITIONS
    - The body is already behind the leader and in contact or near contact
      (walk_to_first_contact ran, or the caller measured contacts > 0).
    - leader_x < target_x - lead at entry; otherwise the skill executes zero
      bursts and is a no-op (that is the intended idempotent behaviour, so a
      caller may stage fast/slow/fine back to back).
    - target_x came from a real measurement (for 04: the target plane x).

POSTCONDITIONS
    - Returns {'bursts', 'trace', 'contacts', 'base', 'leader'}; 'bursts' is
      the number of move_base bursts executed.
    - After the call leader_x >= target_x - lead, or max_bursts was hit.
    - Exactly 1 measure at the top of every iteration plus 1 per executed
      burst.
    - last_cmd is the measured upper joints of the last burst (or the settle's
      command, identical), so a following hold or push keeps the same arm
      pose.

SIDE EFFECTS
    - Moves the base up to max_bursts * vx * dur metres along +x, displaces
      the leader (and anything riding on it), and rewrites last_cmd each
      burst.  A settle also advances simulation time by settle_dur without
      any new nav command.

SCOPE
    Reusable for any straight +x push of a movable leader towards a world-x
    goal with a quasi-static receiver: 04 (cart, three scales), 07 (box on a
    cart, where the push face is a cabinet at a different height and this
    skill's standoff must be re-tuned).  Breaks when (a) the push is not
    along +x or the body must face elsewhere, (b) the leader is not
    quasi-static (a light object tips instead of sliding: 07's ~10 g box and
    09's drill tip on first touch), (c) the leader's response to a burst is
    much larger than vx*dur (then one burst overshoots and the stop is
    missed), or (d) the leader can leave the +x line laterally and the caller
    gave no ref_y and no centring margin.
"""


def run(api, **kwargs):
    target_x = kwargs.get('target_x', None)
    vx = kwargs.get('vx', 0.25)
    dur = kwargs.get('dur', 1.0)
    lead = kwargs.get('lead', 0.55)
    max_bursts = kwargs.get('max_bursts', 18)
    settle_dur = kwargs.get('settle_dur', 0.0)
    ref_y = kwargs.get('ref_y', None)
    ky = kwargs.get('ky', 1.0)
    kb = kwargs.get('kb', 0.8)
    eb_soft = kwargs.get('eb_soft', 0.0)
    cy_max = kwargs.get('cy_max', 0.08)
    kyw = kwargs.get('kyw', 1.0)
    cw_max = kwargs.get('cw_max', 0.25)
    leader = kwargs.get('leader', 'shelf')
    phase = kwargs.get('phase', 'push')

    trace = []
    n_run = 0
    # no read before the loop: the promoted loops measure at the TOP of each
    # iteration, and an extra probe here would change the dynamics
    s = {'contacts': None, 'base': None, 'leader': None}
    for k in np.arange(max_bursts):
        s = api.skill('measure_scene_state', leader=leader)
        if s[leader][0] >= target_x - lead:
            break
        if s[leader][0] > target_x:
            break
        d = api.skill('steer_behind_leader',
                      ref_y=ref_y, leader_y=s[leader][1],
                      base_y=s['base']['pos'][1],
                      base_yaw_deg=s['base']['yaw_deg'],
                      ky=ky, kb=kb, eb_soft=eb_soft, cy_max=cy_max,
                      kyw=kyw, cw_max=cw_max)
        api.move_base(x=vx, y=d['y'], yaw=d['yaw'], dur=dur)
        if settle_dur > 0.0:
            api.skill('hold_commanded_pose', dur=settle_dur)
        s2 = api.skill('measure_scene_state', leader=leader)
        n_run += 1
        trace.append({'phase': phase, 'k': k.item(),
                      'cmd': [vx, np.round(d['y'], 3), np.round(d['yaw'], 3)],
                      'base': s2['base'], 'shelf': s2['shelf'],
                      'box': s2['box'], 'target': s2['target'],
                      'zup': s2['zup'], 'yaw': np.round(s2['yaw'], 2),
                      'contacts': s2['contacts']})
    return {'bursts': n_run, 'trace': trace, 'contacts': s['contacts'],
            'base': s['base'], 'leader': s.get(leader)}
