"""Skill: park the non-grasping hand radially clear of the object.

Mechanism
  Measure the parking palm and the object, take the horizontal unit vector from
  the object to that palm (falling back to a fixed direction if they are
  coincident), and move the palm `park` metres further out along that same
  radial line, `dz` metres up, holding orientation and the current finger
  targets.  If the palm is already further than `trigger` metres from the
  object, do nothing.

Failure signature it removes
  A non-grasping hand that stays inside the object's contact shell gets swept
  into the object during the grasp and the carry: it pins the object, adds
  side contacts and tips it.  Retiring the idle hand along the direction it is
  already outside the object in cannot cross the object, and preserves any
  accidental contacts it already had (no sweeping motion through the object).

Contract
  signature: run(api, side='left', other='right', park=0.09, trigger=0.45,
                 dz=0.02, dur=0.9, orientation='keep')
  preconditions: the object pose is observable; the `side` palm is not yet
      grasping.
  postconditions: if the palm was within `trigger` metres of the object it now
      sits ~`park` metres further out along the same radial line and `dz`
      higher, orientation unchanged; otherwise the arms are untouched.  Contact
      counts and both palm-to-object distances are reported afterwards.
  side effects: one object api.geom() to measure, then in the moving branch
      five api.geom() reads total (one for the palm, one for the moved palm,
      one inside the distance check, one for each palm at the end); in the
      no-move branch three.  Exact read order is part of the behaviour.
  returns: {'info', 'phase', 'sides_after', 'dist_left', 'dist_right'}

Scope: two-handed tasks where one hand must be retired before a grasp or a
carry.  Breaks when the idle hand is trapped between the object and another
body (the radial direction is blocked), when the palm is coincident with the
object (fixed fallback direction may be wrong), or when `park` is small enough
that the palm is still inside the object's collision margin.
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


def _dist(api, side):
    g = api.geom()
    p = np.asarray(g['sites'][side]['pos'], np.float64)
    o = np.asarray(g['object']['pos'], np.float64)
    d = p - o
    return np.sqrt(np.sum(d * d)).item()


def run(api, side='left', other='right', park=0.09, trigger=0.45, dz=0.02,
        dur=0.9, orientation='keep', **kw):
    g = api.geom()
    pl = np.asarray(g['sites'][side]['pos'], np.float64)
    objp = np.asarray(g['object']['pos'], np.float64)
    u = pl[:2] - objp[:2]
    n = np.sqrt(np.sum(u * u)).item()
    if n < trigger:
        uu = u / n if n > 1e-6 else np.array([-1.0, 0.0])
        tgt = [(pl[0] + uu[0] * park).item(),
               (pl[1] + uu[1] * park).item(), (pl[2] + dz).item()]
        rr = api.reach_box(targets={side: tgt}, sides=(side,),
                           orientation=orientation, hand_hold=api.commanded(),
                           dur=dur)
        info = {'moved': True, 'target': tgt,
                'residual': rr['palm_residual'],
                'palm_before': pl.tolist(),
                'palm_after': api.geom()['sites'][side]['pos'],
                'dist_before': np.round(n, 4).item(),
                'dist_after': np.round(_dist(api, side), 4).item(),
                'sides': _side_contacts(api),
                'hand_n': _hand_contacts(api),
                't': np.round(api.t, 2).item()}
    else:
        info = {'moved': False,
                'dist': np.round(n, 4).item(),
                'sides': _side_contacts(api),
                't': np.round(api.t, 2).item()}
    sc = _side_contacts(api)
    dl = _dist(api, side)
    dr = _dist(api, other)
    return {'info': info, 'phase': [sc, dl, dr], 'sides_after': sc,
            'dist_left': dl, 'dist_right': dr}
