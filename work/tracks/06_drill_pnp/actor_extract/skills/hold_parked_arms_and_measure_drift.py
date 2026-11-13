"""Skill: hold the whole body still and measure how far the object drifts.

Mechanism
  Record the object position, command the current upper-body posture with the
  current base height for `hold` seconds, and measure the object again.  The
  palms do not move during the hold, so any object motion is the environment's
  own settling.  Report the drift distance and the final, actor-visible
  evidence: object pose, uprightness, support contacts, hand contacts,
  per-side hand contacts, both palms and the base state.

Failure signature it removes
  Declaring success at the instant of release: a freshly released object can
  still be settling or still be touched by a retracting hand.  Holding the
  posture for a fixed window and measuring the object before and after turns a
  hopeful "it is placed" into a measured "it stayed placed", and a nonzero
  drift is the signal that the placement is not static.

Contract
  signature: run(api, hold=2.0)
  preconditions: the arms have been parked clear of the released object.
  postconditions: `hold` seconds of a frozen posture have elapsed; the object
      pose is measured; drift between the two reads is reported.
  side effects: one object api.geom() before the hold, one api.step_goal(dur),
      one api.geom() after (shared by the drift and by every final field), then
      pure contact and base reads.  The final `gripper_dist` uses the SAME
      post-hold read rather than a new one.
  returns: {'drift', 'final': {'obj', 'z_up', 'table_contacts',
            'hand_contacts', 'sides', 'palm', 'palm_left', 'base',
            'gripper_dist'}}

Scope: verifying a static placement for a rigid object on a horizontal
support.  Breaks if `hold` is shorter than the object's settling time, or if
the task's success condition is about something other than the final object
pose and contacts.
"""


def _table_contact(api):
    pairs = api.contact_pairs(60)
    n = 0
    for p in pairs:
        if 'obj' in p['b1'] or 'obj' in p['b2']:
            if 'table' in p['b1'] or 'table' in p['b2']:
                n = n + 1
    return n


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


def run(api, hold=2.0, right='right', left='left', **kw):
    p_pre_hold = np.asarray(api.geom()['object']['pos'], np.float64)
    api.step_goal(upper=api.commanded(), height=api.height, dur=hold)
    g = api.geom()
    obj_end = np.asarray(g['object']['pos'], np.float64)
    out = {'drift': np.round(np.sqrt(np.sum((obj_end - p_pre_hold) ** 2)),
                             5).item()}
    out['final'] = {'obj': g['object']['pos'],
                    'z_up': g['object']['z_up'],
                    'table_contacts': _table_contact(api),
                    'hand_contacts': _hand_contacts(api),
                    'sides': _side_contacts(api),
                    'palm': g['sites'][right]['pos'],
                    'palm_left': g['sites'][left]['pos'],
                    'base': api.base_state()}
    objp = np.asarray(g['object']['pos'], np.float64)
    rp = np.asarray(g['sites'][right]['pos'], np.float64)
    out['final']['gripper_dist'] = np.round(np.sqrt(
        np.sum((rp - objp) ** 2)), 3).item()
    return out
