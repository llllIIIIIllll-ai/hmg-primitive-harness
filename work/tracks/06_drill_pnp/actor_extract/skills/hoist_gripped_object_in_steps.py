"""Skill: hoist a grasped object in small vertical palm steps, aborting on loss.

Mechanism
  Lift the palm by `step` metres at a time, re-commanding the SAME finger
  targets every step (hand_hold = the command captured when the skill starts),
  and after each step measure the object's height, uprightness and per-side
  hand contacts.  Abort as soon as the grasp signature degrades: contact on the
  idle side, no contact on the grasping side, or uprightness below `z_up_min`.

Failure signature it removes
  A single large lift yanks the object out of the fingers (or drags it across
  the support) and, worse, re-commanding a stale arm pose during the lift
  silently drops the object.  Stepwise lifting with the finger targets pinned
  keeps the grasp closed, and the per-step measurement turns "the object came
  loose" into an immediate, unambiguous abort instead of a confusing failure
  ten phases later.

Contract
  signature: run(api, side='right', step=0.015, max_steps=3, left_max=0,
                 right_min=1, z_up_min=0.85, dur=0.4)
  preconditions: the fingers are closed on the object with a valid grasp;
      api.commanded() holds that grasp's arm pose.
  postconditions: the object has been raised by up to max_steps*step, or the
      skill stopped early.  The last commanded arm pose, the object pose, the
      uprightness and the contact counts are returned.
  side effects: per step one api.geom() for the palm, one api.reach_box(dur),
      one object api.geom(), two contact scans; at the end two more object
      api.geom() reads plus two contact scans; and when the grasp is lost two
      further reads plus api.base_state().  Read order is part of the
      behaviour.
  returns: {'log', 'lost', 'cmd', 'obj', 'z_up', 'hand_contacts', 'sides',
            'phase'} plus 'final' (the promoted early-exit record) when lost.

Scope: vertical hoists of a handle-grasped object off a support.  Breaks if
the object is too heavy for the wrist to hold at the current grip angle, if
`step` is large enough that the fingers lose the handle in one window, or if
the object needs a rotation as well as a lift (use a wrist-leveling skill for
that).
"""


def _hand_contacts(api):
    pairs = api.contact_pairs(60)
    n = 0
    for p in pairs:
        if 'obj' in p['b1'] or 'obj' in p['b2']:
            if p['g1'].startswith('gripper0') or p['g2'].startswith('gripper0'):
                n = n + 1
    return n


def _side_contacts(api):
    pairs = api.contact_pairs(60)
    nl = 0
    nr = 0
    for p in pairs:
        if not ('obj' in p['b1'] or 'obj' in p['b2']):
            continue
        for g in (p['g1'], p['g2']):
            if g.startswith('gripper0_left'):
                nl = nl + 1
            elif g.startswith('gripper0_right'):
                nr = nr + 1
    return nl, nr


def run(api, side='right', step=0.015, max_steps=3, left_max=0, right_min=1,
        z_up_min=0.85, dur=0.4, **kw):
    cmd = api.commanded()
    log = []
    lost = False
    for i in np.arange(max_steps):
        p = np.asarray(api.geom()['sites'][side]['pos'], np.float64)
        t = p + np.array([0.0, 0.0, step])
        api.reach_box(targets={side: t.tolist()}, sides=(side,),
                      orientation='keep', hand_hold=cmd, dur=dur)
        cmd = api.commanded()
        g = api.geom()['object']
        sc = _side_contacts(api)
        zu = np.round(g['z_up'], 3).item()
        log.append([i.item(), np.round(api.t, 2).item(),
                    [np.round(v, 4).item() for v in g['pos']],
                    zu, _hand_contacts(api), sc])
        if sc[0] > left_max or sc[1] < right_min or zu < z_up_min:
            lost = True
            break
    out = {'log': log, 'lost': lost, 'cmd': cmd, 'obj': g['pos'],
           'z_up': g['z_up'], 'hand_contacts': _hand_contacts(api),
           'sides': _side_contacts(api)}
    out['phase'] = [api.geom()['object']['pos'], api.geom()['object']['z_up'],
                    _hand_contacts(api), _side_contacts(api)]
    if lost:
        out['final'] = {'obj': api.geom()['object']['pos'],
                        'z_up': api.geom()['object']['z_up'],
                        'hand_contacts': _hand_contacts(api),
                        'sides': _side_contacts(api),
                        'base': api.base_state()}
    return out
