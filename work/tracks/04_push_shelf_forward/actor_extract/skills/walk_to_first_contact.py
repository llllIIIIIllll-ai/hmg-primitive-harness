"""Contact-free approach walk to the first contact / standoff (approach).

MECHANISM NAME
    walk_to_first_contact -- burst-walk the whole body straight at a leader
    along world +x, steering with steer_behind_leader, and STOP on a measured
    event instead of on a clock: a reported hand-object contact, or the
    leader coming within a chosen standoff gap of the base.

FAILURE SIGNATURE IT ADDRESSES
    Open-loop walking never stops at the right standoff.  Measured on
    04_push_shelf_forward: the first hand/cart contact begins only when the
    base is 0.72-0.88 m behind the cart body origin, and one 1 s burst at
    nav vx = 0.30 closes 0.25-0.30 m, so a fixed burst count either stops
    short (no push at all) or arrives late and shoves the cart.  The
    termination therefore has to be a per-burst MEASUREMENT, and the walk has
    to be burst-shaped so the loop can re-measure before every command.

THE MECHANISM (what it does physically)
    Repeat up to max_bursts times:
      measure the scene (measure_scene_state);
      break if contacts > 0 (a hand is already on the leader);
      break if leader_x - base_x < app_gap (standoff reached);
      ask steer_behind_leader for (y, yaw);
      api.move_base(x=vx, y=..., yaw=..., dur=dur)  (nav, body-frame +x);
      measure again and record the post-burst state.
    Because ``move_base`` commands the MEASURED upper joints for the whole
    burst, the arms are held by construction while the legs walk -- the body
    arrives behind an extended arm, which is what actually transfers the
    push.

SIGNATURE (named params + defaults)
    run(api, **kwargs)
      app_gap      float, default 0.80  stop when leader_x - base_x < this (m)
      vx           float, default 0.30  forward nav command per burst
      dur          float, default 1.0   burst length (s)
      max_bursts   int,   default 6     hard cap on bursts
      ref_y        float, default None  lateral reference passed to the steer
                                        law (None = pure centring)
      ky, kb, eb_soft, cy_max, kyw, cw_max   steer-law params, defaults
                                       1.0, 0.8, 0.0, 0.08, 1.0, 0.25
      leader       str,   default 'shelf'  scene-object key of the leader
      phase        str,   default 'approach'  trace label

PRECONDITIONS
    - Leader and base are both ahead-along-+x arranged (leader x > base x).
    - The leader exposes a name in measure_scene_state()['objects'].
    - No contact is yet desired at the end: this skill stops BEFORE or AT
      first contact; it never pushes.

POSTCONDITIONS
    - Returns {'bursts', 'trace', 'contacts', 'base', 'leader'}; 'bursts' is
      the number of move_base bursts actually executed.
    - After the call the base is within app_gap of the leader (or a contact
      is reported, or max_bursts was exhausted); the upper-body command is
      the one move_base last issued (the measured arm pose at that burst).
    - Exactly 1 measure at the top of every iteration plus 1 per executed
      burst: 2*bursts + (0 or 1) scene reads.

SIDE EFFECTS
    - Moves the base up to max_bursts * vx * dur metres along +x and
      rewrites last_cmd with the measured upper joints each burst.
    - Each measure performs the mj_forward/contact reads described in
      measure_scene_state, so the read count is load-bearing.

SCOPE
    Reusable for any task whose first act is to stand the body behind a
    movable object at hand height: 04 (cart), 07 (box on a cart), 09
    (pre-grasp placement).  Breaks when the leader is not roughly straight
    ahead (the law only corrects lateral error, and a leader off to the side
    makes leader_x - base_x meaningless), when the leader is taller than the
    default arm pose can touch, or when obstacles sit between base and leader
    (07's leg-to-cabinet contact at base y -0.355 was measured at this stage
    and this skill does not avoid it).
"""


def run(api, **kwargs):
    app_gap = kwargs.get('app_gap', 0.80)
    vx = kwargs.get('vx', 0.30)
    dur = kwargs.get('dur', 1.0)
    max_bursts = kwargs.get('max_bursts', 6)
    ref_y = kwargs.get('ref_y', None)
    ky = kwargs.get('ky', 1.0)
    kb = kwargs.get('kb', 0.8)
    eb_soft = kwargs.get('eb_soft', 0.0)
    cy_max = kwargs.get('cy_max', 0.08)
    kyw = kwargs.get('kyw', 1.0)
    cw_max = kwargs.get('cw_max', 0.25)
    leader = kwargs.get('leader', 'shelf')
    phase = kwargs.get('phase', 'approach')

    trace = []
    n_run = 0
    # no read before the loop: the promoted loop measures at the TOP of each
    # iteration, and an extra probe here would change the dynamics
    s = {'contacts': None, 'base': None, 'leader': None}
    for k in np.arange(max_bursts):
        s = api.skill('measure_scene_state', leader=leader)
        if s['contacts'] > 0:
            break
        gap = s[leader][0] - s['base']['pos'][0]
        if gap < app_gap:
            break
        d = api.skill('steer_behind_leader',
                      ref_y=ref_y, leader_y=s[leader][1],
                      base_y=s['base']['pos'][1],
                      base_yaw_deg=s['base']['yaw_deg'],
                      ky=ky, kb=kb, eb_soft=eb_soft, cy_max=cy_max,
                      kyw=kyw, cw_max=cw_max)
        api.move_base(x=vx, y=d['y'], yaw=d['yaw'], dur=dur)
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
