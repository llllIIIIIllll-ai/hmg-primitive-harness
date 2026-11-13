"""Skill: closed-loop holonomic base servo to a world pose.

Mechanism
  Re-plan a body-frame velocity command from the measured base pose at every
  control window: rotate the world position error into the base frame, clamp
  each axis to `vmax`, enforce a floor speed `min_speed` so stiction cannot
  stall the walk, and drive yaw with a proportional yaw rate.  Stop only when
  BOTH a position tolerance and a heading tolerance are met.

Failure signature it removes
  Open-loop base commands miss a randomized stance/heading: the hand targets
  that follow are then expressed in the wrong base frame, the palm arrives
  offset from the object and the first thing to touch the handle is the palm
  instead of the closing fingers.  It also removes the "walk stalls just short
  of the goal because the last error is below the velocity that overcomes
  friction" stall (the speed floor).

Contract
  signature: run(api, x, y, yaw_deg, upper=None, tol=0.05, yaw_tol=6.0,
                 max_iter=10, dur=0.5, vmax=0.26, min_speed=0.22,
                 gain=1.5, yaw_gain=0.02, wz_max=0.4, trace_obj=False)
  preconditions: a live episode; `upper` is None or a 31-D joint command in
      contract order.
  postconditions: base position within `tol` metres of (x, y) and heading
      within `yaw_tol` degrees of yaw_deg, or `max_iter` windows spent;
      api nav command is left at zero
  side effects: steps the sim for `dur` seconds per window (base translation
      and yaw only); sets api.nav = zeros at exit; reads api.base_state() once
      per window and, when trace_obj is set, one object api.geom() plus a
      table-contact and a hand-contact count per window.  Those reads are part
      of the measured behaviour - do not strip them.
  returns: {'log': per-window rows, 'last': last row}

Scope: any task that needs a differential/holonomic base driven to a world
pose while holding an arm posture.  Breaks if the base is not holonomically
commandable, if a held object makes the base a rigid pin (the walk then
saturates and the caller must raise the object clear first), or if vmax is
above what the WBC will track.
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


def run(api, x, y, yaw_deg, upper=None, tol=0.05, yaw_tol=6.0, max_iter=10,
        dur=0.5, vmax=0.26, min_speed=0.22, gain=1.5, yaw_gain=0.02,
        wz_max=0.4, trace_obj=False, **kw):
    log = []
    for it in np.arange(max_iter):
        b = api.base_state()
        pos = np.asarray(b['pos'][:2], np.float64)
        err = np.array([x - pos[0], y - pos[1]], np.float64)
        dist = np.sqrt(np.sum(err * err)).item()
        yerr = np.mod(yaw_deg - b['yaw_deg'] + 180.0, 360.0) - 180.0
        log.append([it.item(), b['pos'], b['yaw_deg'], np.round(dist, 3).item()])
        if trace_obj:
            g = api.geom()['object']
            log[-1].extend([np.round(g['pos'][2], 4).item(),
                            np.round(g['z_up'], 4).item(),
                            _table_contact(api), _hand_contacts(api)])
        if dist < tol and np.abs(yerr) < yaw_tol:
            break
        yr = b['yaw_deg'] * np.pi / 180.0
        cs, sn = np.cos(yr), np.sin(yr)
        vxw = np.clip(err[0] * gain, -vmax, vmax).item()
        vyw = np.clip(err[1] * gain, -vmax, vmax).item()
        sp = np.sqrt(vxw * vxw + vyw * vyw).item()
        if 1e-6 < sp < min_speed:
            k = min_speed / sp
            vxw = vxw * k
            vyw = vyw * k
        vx = np.clip(cs * vxw + sn * vyw, -vmax, vmax).item()
        vy = np.clip(-sn * vxw + cs * vyw, -vmax, vmax).item()
        wz = np.clip(yerr * yaw_gain, -wz_max, wz_max).item()
        u = api.upper() if upper is None else upper
        api.step_goal(upper=u, nav=[vx, vy, wz], dur=dur)
    api.nav = np.zeros(3)
    return {'log': log, 'last': log[-1]}
