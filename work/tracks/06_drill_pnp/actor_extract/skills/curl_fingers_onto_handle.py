"""Skill: curl the fingers onto a handle and verify the grasp from contacts.

Mechanism
  Close the fingers to a commanded curl angle while the palm sits at the
  standoff.  The finger sweep is the only motion, so the handle is captured
  rather than pushed.  Immediately measure the real object pose and the real
  hand-object contact counts per side, so the caller can tell a handle grasp
  (contacts on the grasping side, none on the other) from a miss or a pinch on
  the wrong side.

Failure signature it removes
  Trusting an open-loop curl angle with no verification: a standoff that is a
  few millimetres wrong closes the fingers on air (no contact) or on the wrong
  face (contact on the idle side, object dragged).  The measured per-side
  contact counts are the evidence that separates these cases and are what
  every later phase gates on.

Contract
  signature: run(api, angle=1.3, dur=1.1)
  preconditions: the grasping palm is parked at the standoff from the object;
      the object is at rest.
  postconditions: fingers commanded to `angle`; the object pose and the
      per-side hand-object contact counts are measured and returned.
  side effects: one api.close_grip(angle, dur), two object api.geom() reads
      (position, then uprightness), two api.contact_pairs() scans.
  returns: {'obj', 'z_up', 'hand_contacts', 'sides'}

Scope: parallel-jaw grasps on a handle narrower than the finger sweep, reached
from a standoff.  Breaks for objects wider than the open finger span, for
grasps that need the palm itself to press the object, and where the handle is
not between the fingers at the standoff depth.
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


def run(api, angle=1.3, dur=1.1, **kw):
    api.close_grip(angle=angle, dur=dur)
    obj = api.geom()['object']['pos']
    zu = api.geom()['object']['z_up']
    return {'obj': obj, 'z_up': zu, 'hand_contacts': _hand_contacts(api),
            'sides': _side_contacts(api)}
