"""Skill: open the fingers and let the seated object settle without a push.

Mechanism
  Command the finger joints to `open_angle` over `open_dur` and then hold the
  whole body posture with no palm motion at all for `settle_dur`.  Opening the
  fingers while the arm posture is frozen means the hand can only retract
  sideways out of the grasp; it cannot shove the object along the support.
  Then measure the settled object.

Failure signature it removes
  Opening the fingers while still moving the palm, or retreating the palm at
  the same time as the release: the hand pushes the freshly seated object and
  rotates it off vertical (measured on the drill, an early single-corner
  release left ~5 N on one corner and a 12 deg tilt).  Freezing the posture
  during the open removes the post-release hand push.

Contract
  signature: run(api, open_angle=0.02, open_dur=0.6, settle_dur=1.0)
  preconditions: the object is seated on the support and still gripped with the
      seated posture commanded.
  postconditions: fingers are at `open_angle`; the body has been held still for
      `settle_dur`; the settled contacts and object pose are measured.
  side effects: one api.close_grip, one api.step_goal; then a seat record made
      of two contact scans and two object api.geom() reads in the fixed order
      (position first, then uprightness).  No api.geom() before the settle.
  returns: {'seat': {'table', 'sides', 'obj', 'z_up', 'hand'}}

Scope: releasing a rigid object onto a support where the fingers can retract
sideways.  Breaks if the fingers are wedged so that opening them necessarily
drags the object, or if the support is inclined.
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


def _zup(api):
    return api.geom()['object']['z_up']


def run(api, open_angle=0.02, open_dur=0.6, settle_dur=1.0, **kw):
    api.close_grip(angle=open_angle, dur=open_dur)
    api.step_goal(upper=api.commanded(), height=api.height, dur=settle_dur)
    seat = {'table': _table_contact(api), 'sides': _side_contacts(api),
            'obj': api.geom()['object']['pos'],
            'z_up': np.round(_zup(api), 4).item(),
            'hand': _hand_contacts(api)}
    return {'seat': seat}
