"""servo_fingertip_to_point: closed-loop world-point placement of a fingertip.

MECHANISM
    Measured-fingertip servoing with palm-site command compensation.  The
    skill measures the physical fingertip (the distal phalanx mesh tip, from
    api.hand_geoms()) and the palm site, then commands the site by
    ``site + gain * (target - tip)`` through position-only DLS IK
    (api.reach_box orientation='none'), so the IK moves the palm by that delta
    and the fingertip closes the residual.  Iterates until the measured tip is
    within ``tol`` of the target.

FAILURE SIGNATURE IT REMOVES
    Open-loop arm reaching never lands the fingertip on a desired world point:
    the IK residual and the unknown fingertip offset from the palm site leave
    a steady error that no single open-loop reach removes.  Measured on
    02_push_button seed 1: the first approach iteration has a 0.126 m tip
    error, and the servo drives it under the 0.008 m tolerance in 5 reaches
    (4 on seed 5) instead of relying on one blind reach.

CONTRACT
    preconditions
      - the named side's hand is present and ``api.hand_geoms()`` lists its
        fingertip link and mesh;
      - the target is inside the arm workspace reachable from the current base.
    postconditions
      - either the measured tip is within ``tol`` of the target, or
        ``max_iters`` reach commands were issued;
      - returns the last measured tip, residual and a per-iteration trace
        carrying ``phase``/``tag`` labels chosen by the caller.
    side effects
      - commands the 31-D upper-body goal through api.reach_box, advancing sim
        time by up to ``max_iters * dur``;
      - read-only hand_geoms()/geom() reads (both run mj_forward); when
        ``watch_match`` is set, one bodies() read after every commanded reach.
    args
      side         'left'/'right' hand
      target       world point (x,y,z)
      gain         fraction of the measured tip error applied per iteration
      dur          duration of each reach command (s)
      max_iters    iteration cap
      tol          convergence radius on the measured tip error (m)
      tip_link     body-name tag of the fingertip link
      mesh_tag     geom-name tag of the fingertip mesh
      tip_half     half-extent of that mesh along its local z (m)
      watch_match  optional body-name substring whose joint is logged after
                   every commanded reach (None = no such read)
      watch_joint  joint index logged when watch_match is set
      phase        label written into the trace records
      tag          optional per-call label written into the trace records
    returns
      {'tip': [x,y,z], 'err': [ex,ey,ez], 'converged': bool, 'iters': int,
       'trace': [...]}
"""
TIP_HALF = 0.0334          # index_1 vis mesh half-extent along its local z


def _tip(api, side, tip_link, mesh_tag, tip_half):
    """World position of a fingertip (distal phalanx mesh tip)."""
    hg = api.hand_geoms()
    for b in hg:
        if tip_link in b and ('_%s_' % side) in b:
            for gg in hg[b]['geoms']:
                if mesh_tag in gg['name']:
                    R = np.array(gg['xmat'])
                    return np.array(gg['world_pos']) + tip_half * R[:, 2]
    return None


def run(api, side='right', target=(0.0, 0.0, 0.0), gain=0.7, dur=0.45,
        max_iters=8, tol=0.008, tip_link='index_1', mesh_tag='_vis',
        tip_half=TIP_HALF, watch_match=None, watch_joint=0,
        phase='servo', tag=None):
    tgt = np.array(target)
    trace = []
    tip = None
    err = None
    iters = 0
    for it in np.arange(max_iters):
        tip = _tip(api, side, tip_link, mesh_tag, tip_half)
        site = np.array(api.geom()['sites'][side]['pos'])
        err = tgt - tip
        en = np.linalg.norm(err)
        iters = it.item() + 1
        if en < tol:
            rec = {'phase': phase, 'it': it.item(), 'converged': True,
                   'tip': tip.round(4).tolist(),
                   'err': err.round(4).tolist()}
            if tag is not None:
                rec['tag'] = tag
            trace.append(rec)
            break
        cmd = site + gain * err
        r = api.reach_box(sides=(side,), targets={side: cmd.tolist()},
                          orientation='none', dur=dur)
        rec = {'phase': phase, 'it': it.item(),
               'tip': tip.round(4).tolist(),
               'err': err.round(4).tolist(),
               'resid': r['palm_residual'], 'seed': r['ik_seed']}
        if tag is not None:
            rec['tag'] = tag
        if watch_match is not None:
            b = api.bodies(match=watch_match)[0]
            rec['q'] = b['joints'][watch_joint]['q'] if b['joints'] else None
        trace.append(rec)
    return {'tip': tip.round(4).tolist(), 'err': err.round(4).tolist(),
            'converged': True if np.linalg.norm(err) < tol else False,
            'iters': iters, 'trace': trace}
