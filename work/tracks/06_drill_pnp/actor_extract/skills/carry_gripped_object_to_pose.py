"""Skill: carry a grasped object to a world pose, with a stall recovery raise.

Mechanism
  Walk the base to (x, y, yaw_deg) with the arm posture held by the servo
  skill, tracing the object at every window.  Afterwards measure the base
  heading.  If the heading is still short of the goal and the grasp is intact,
  lift the object `recover_dz` and re-walk in shorter windows; repeat up to
  `recover` times.  A held rigid object turns the base into a pin whenever it
  presses a support, so raising it is the only way to give the walk its
  freedom back.

Failure signature it removes
  A carry that ends short in heading and never recovers: the object's lowest
  collider is dragging over a table edge, the base velocity is saturated, and
  the yaw plateaus (drill: 64-65 deg against a 90 deg goal with wz pinned at
  0.4).  Measuring heading after the walk and escaping with a raise turns a
  silent stall into a recovered carry.

Contract
  signature: run(api, x=0.15, y=0.55, yaw_deg=90.0, tol=0.06, yaw_tol=8.0,
                 max_iter=10, upper=None, recover=2, recover_iter=6,
                 recover_dz=0.02, recover_yaw_tol=8.0, trace_obj=True)
  preconditions: a grasp is held; `upper` is the 31-D posture to carry with
      (normally the command captured after the clearance raise).
  postconditions: base near the goal or recovery exhausted; object pose and
      base heading measured afterwards; the object is still grasped unless it
      was already lost.
  side effects: the servo skill's per-window reads (base state, and with
      trace_obj one object geom plus a table and a hand contact count); two
      object geoms after the walk for the phase record; per recovery one raise
      (two geoms) plus another servo walk; one object geom at the end.  Exact
      read order is part of the behaviour.
  returns: {'carry_log', 'obj_after_carry', 'z_up_after_carry',
            'carry_recovery', 'base_end', 'yerr', 'obj_end'}

Scope: carrying a grasped object between tables with a holonomic base.  Breaks
if the target is further than the walk can travel in `max_iter` windows, if the
object cannot be lifted clear of whatever pins it, or if the grasp is lost
before the recovery raise.
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


def run(api, x=0.15, y=0.55, yaw_deg=90.0, tol=0.06, yaw_tol=8.0,
        max_iter=10, upper=None, recover=2, recover_iter=6, recover_dz=0.02,
        recover_yaw_tol=8.0, trace_obj=True, **kw):
    res = api.skill('servo_base_to_target_pose', x=x, y=y, yaw_deg=yaw_deg,
                    upper=upper, tol=tol, yaw_tol=yaw_tol, max_iter=max_iter,
                    trace_obj=trace_obj)
    clog = res['log']
    obj_after = api.geom()['object']['pos']
    z_up_after = api.geom()['object']['z_up']

    b = api.base_state()
    yerr = np.mod(yaw_deg - b['yaw_deg'] + 180.0, 360.0) - 180.0
    recov = []
    for k in np.arange(recover):
        if np.abs(yerr) < recover_yaw_tol:
            break
        if _hand_contacts(api) == 0:
            break
        r = api.skill('raise_palm_holding_grip', dz=recover_dz,
                      hand_hold=api.commanded())
        recov.append([k.item(), np.round(api.t, 2).item(), r['target'],
                      [np.round(v, 4).item() for v in r['object']['pos']],
                      np.round(r['object']['z_up'], 4).item(),
                      _table_contact(api), _hand_contacts(api)])
        res2 = api.skill('servo_base_to_target_pose', x=x, y=y,
                         yaw_deg=yaw_deg, upper=api.commanded(), tol=tol,
                         yaw_tol=yaw_tol, max_iter=recover_iter,
                         trace_obj=trace_obj)
        recov.append(res2['last'])
        b = api.base_state()
        yerr = np.mod(yaw_deg - b['yaw_deg'] + 180.0, 360.0) - 180.0

    base_end = api.base_state()
    obj_end = api.geom()['object']['pos']
    return {'carry_log': clog, 'obj_after_carry': obj_after,
            'z_up_after_carry': z_up_after, 'carry_recovery': recov,
            'base_end': base_end, 'yerr': yerr.item(),
            'obj_end': obj_end}
