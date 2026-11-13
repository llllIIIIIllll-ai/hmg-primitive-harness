"""Skill: park both arms radially away from a released object, then hold.

Mechanism
  Repeat up to `max_iter` times: read the object and both palms.  Declare
  done only when no hand touches the object AND each palm is at least its
  clearance (`right_clear`, `left_clear`) from the object.  Otherwise push
  each hand that is still too close radially outward by `step` in the world
  plane, holding the finger targets so the open hand never closes on the
  object.  After every push, check that the object did not move vertically or
  horizontally and did not lose uprightness, and stop the whole retreat the
  moment any of that happens.

Failure signature it removes
  Parking an arm by retracting it toward the body or toward the object's axis:
  the open hand brushes the seated object and nudges it (measured on the
  drill, post-release palm motion rotated the drill off vertical).  A radial
  in-plane push away from the object, verified against the object's measured
  displacement after each step, removes contact without disturbing the seat.

Contract
  signature: run(api, right_clear=0.26, left_clear=0.22, step=0.015,
                 max_iter=14, move_dur=0.45, hold_dur=0.35, abort_z_up=0.99,
                 abort_dz=0.004, abort_dxy=0.05, right='right', left='left')
  preconditions: the object has been released and is seated; both hands are
      open and near the object.
  postconditions: no hand touches the object and both palms clear their
      clearance, or the iteration budget is spent, or an abort condition
      tripped (object rose/dropped/translated too much or lost uprightness).
  side effects: one object api.geom() per iteration; per right push one
      api.reach_box(move_dur) plus one api.step_goal(hold_dur) plus one object
      api.geom(); per left attempt two object api.geom() reads for the outward
      direction, then one api.geom() for the palm, the same push pair, and one
      object api.geom().  These read counts are part of the behaviour.
  returns: {'log', 'parked', 'object', 'sides', 'hand_n'}

Scope: clearing open hands away from a seated object on a support.  Breaks if
the object sits in a corner where the outward direction points into an
obstacle, or if the arm cannot reach the clearance distance.
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


def run(api, right_clear=0.26, left_clear=0.22, step=0.015, max_iter=14,
        move_dur=0.45, hold_dur=0.35, abort_z_up=0.99, abort_dz=0.004,
        abort_dxy=0.05, right='right', left='left', **kw):
    log = []
    parked = False
    for i in np.arange(max_iter):
        g = api.geom()
        obj = np.asarray(g['object']['pos'], np.float64)
        if _hand_contacts(api) == 0:
            pd = np.asarray(g['sites'][right]['pos'], np.float64) - obj
            pl = np.asarray(g['sites'][left]['pos'], np.float64) - obj
            dr = np.sqrt(np.sum(pd * pd))
            dl = np.sqrt(np.sum(pl * pl))
            if dr >= right_clear and dl >= left_clear:
                parked = True
                log.append([i.item(), np.round(api.t, 2).item(),
                            np.round(dr, 4).item(), np.round(dl, 4).item()])
                break
        do_right = _side_contacts(api)[1] > 0 or np.sqrt(np.sum(
            (np.asarray(g['sites'][right]['pos'], np.float64) - obj) ** 2)) \
            < right_clear
        if do_right:
            d = np.asarray(g['sites'][right]['pos'], np.float64)[:2] - obj[:2]
            n = np.sqrt(np.sum(d * d)).item()
            u = d / n if n > 1e-6 else np.array([0.0, -1.0])
            p = np.asarray(g['sites'][right]['pos'], np.float64)
            t = p + np.array([u[0] * step, u[1] * step, 0.0])
            api.reach_box(targets={right: t.tolist()}, sides=(right,),
                          orientation='keep', hand_hold=api.commanded(),
                          dur=move_dur)
            api.step_goal(upper=api.commanded(), height=api.height,
                          dur=hold_dur)
            g1 = api.geom()['object']
            dz = g1['pos'][2] - g['object']['pos'][2]
            dxy = np.sqrt((g1['pos'][0] - g['object']['pos'][0]) ** 2 +
                          (g1['pos'][1] - g['object']['pos'][1]) ** 2).item()
            log.append([i.item(), np.round(api.t, 2).item(), right,
                        np.round(dz, 5).item(), np.round(dxy, 5).item()])
            if g1['z_up'] < abort_z_up or dz > abort_dz or dxy > abort_dxy:
                break
        nl = _side_contacts(api)[0]
        d = np.asarray(api.geom()['sites'][left]['pos'], np.float64)[:2] - \
            np.asarray(api.geom()['object']['pos'], np.float64)[:2]
        n = np.sqrt(np.sum(d * d)).item()
        if nl > 0 or n < left_clear:
            u = d / n if n > 1e-6 else np.array([0.0, -1.0])
            p = np.asarray(api.geom()['sites'][left]['pos'], np.float64)
            t = p + np.array([u[0] * step, u[1] * step, 0.0])
            api.reach_box(targets={left: t.tolist()}, sides=(left,),
                          orientation='keep', hand_hold=api.commanded(),
                          dur=move_dur)
            api.step_goal(upper=api.commanded(), height=api.height,
                          dur=hold_dur)
            g1 = api.geom()['object']
            dz = g1['pos'][2] - g['object']['pos'][2]
            dxy = np.sqrt((g1['pos'][0] - g['object']['pos'][0]) ** 2 +
                          (g1['pos'][1] - g['object']['pos'][1]) ** 2).item()
            log.append([i.item(), np.round(api.t, 2).item(), left,
                        np.round(dz, 5).item(), np.round(dxy, 5).item()])
            if g1['z_up'] < abort_z_up or dz > abort_dz or dxy > abort_dxy:
                break
    return {'log': log, 'parked': parked,
            'hand_n': _hand_contacts(api), 'sides': _side_contacts(api)}
