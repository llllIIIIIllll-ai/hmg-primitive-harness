"""Skill: signed hill-climb over wrist joints that raises object uprightness.

Mechanism
  Read the object's uprightness once, then sweep one or more wrist joints.
  Every probe is an atomic one-joint nudge, api.skill('nudge_wrist_joint'),
  which holds the rest of the commanded posture and measures the object.  A
  probe that gains more than `min_gain` is kept and the same joint is probed
  again with the current signed step.  A probe that does not gain is handled
  in one of two ways:

    sign_reversal=False : undo it with nudge(-d) and abandon this joint.
    sign_reversal=True  : answer it with an opposite probe of magnitude
                          reverse*step.  If that gains, keep it, flip the
                          sign of the step (d = -d) and continue this joint.
                          If it does not gain, restore the original probe
                          (nudge(d)) and abandon this joint.

  The sweep stops when the object is upright enough, or after `max_tries`
  accepted probes on the current joint.  A final uprightness read is always
  taken, including when no probe is accepted.

Read policy (a physical parameter, not a diagnostic)
  Reads run mj_forward, perturb the solver warmstart and are part of the
  verified sequence, so the read count and order are explicit parameters.

    read_mode='nudge' : use the z_up returned by the nudge itself.  One read
                        before the sweep, one read per probe (inside the
                        nudge), no read for the undo, one read at the end.
    read_mode='geom'  : ignore the nudge's z_up and perform an explicit
                        api.geom() after every nudge, every opposite probe and
                        every restore.  One read before the sweep, two reads
                        per probe, one read at the end.

  The opposite probe and restore always take a read in 'geom' mode; in
  'nudge' mode they use the return value they were given, so no read is
  added.  (The 'nudge' path never enters the opposite-probe branch in the
  reproductions below.)

Target rule (a physical parameter)
    target_stop='gt' : keep probing unless z_up > target  (single-axis rule;
                       at exactly target it probes again).
    target_stop='ge' : keep probing while z_up < target   (two-axis rule; at
                       exactly target it stops).

Output shape (a return-value parameter, no physical effect)
    out='single' : {'zup_before', 'zup_after'}
    out='two'    : {'log', 'zup_after'}, log rows are
                   [joint, signed_step_at_accept, round(z_before,4),
                    round(z_after,4)].

Contract
  signature: run(api, joints=(14, 15), step=0.08, target=0.996,
                 min_gain=0.001, max_tries=4, dur=0.35, reverse=2.0,
                 sign_reversal=True, read_mode='geom', target_stop='ge',
                 out='two')
  preconditions: api.commanded() is a 31-D posture (a grasp is held or the
      arm is parked); every joint indexes the contract joint order and is a
      wrist roll/pitch joint.
  postconditions: every accepted probe raised z_up by more than min_gain over
      the value it was compared with; the opposite/restore branch can end with
      z_up BELOW the entry value (the restore is a command, not a rewind of
      the opposite probe's physical effect).  zup_after is always measured
      after a final read.  No guarantee that target is reached.
  side effects: one object read before and one after the sweep; per probe one
      step_goal and 1 (read_mode='nudge') or 2 (read_mode='geom') object
      reads, except the sign_reversal=False undo which adds one step_goal and
      zero object reads (the nudge's own return is discarded).
  returns: see Output shape.

Reproductions (verified element-for-element in merge_round)
  level_upright_single_axis(joint=15, delta=0.2, target=0.97,
      min_gain=0.002, max_tries=3, dur=0.5)
    == level_upright(joints=(15,), step=0.2, target=0.97, min_gain=0.002,
                     max_tries=3, dur=0.5, reverse=1.0,
                     sign_reversal=False, read_mode='nudge',
                     target_stop='gt', out='single')

  level_upright_two_axis(joints=(14, 15), step=0.08, target=0.996,
      min_gain=0.001, max_tries=4, dur=0.35, reverse=2.0)
    == level_upright(joints=(14, 15), step=0.08, target=0.996,
                     min_gain=0.001, max_tries=4, dur=0.35, reverse=2.0,
                     sign_reversal=True, read_mode='geom',
                     target_stop='ge', out='two')

Scope
  Right-wrist (or any single-joint) orientation adjustment while a grasp is
  held.  Breaks for joints whose motion translates the palm into the object,
  for steps large enough to hit a joint limit, and when no command exists yet
  (api.commanded() is None in a freshly reset episode).
"""


def run(api, joints=(14, 15), step=0.08, target=0.996, min_gain=0.001,
        max_tries=4, dur=0.35, reverse=2.0, sign_reversal=True,
        read_mode='geom', target_stop='ge', out='two', **kw):
    z = api.geom()['object']['z_up']
    before = np.round(z, 4).item()
    log = []
    for jj in joints:
        d = step
        improved = True
        tries = 0
        while tries < max_tries:
            if not improved:
                break
            keep = z < target
            if target_stop == 'gt':
                keep = not (z > target)
            if not keep:
                break
            improved = False
            z0 = z
            r = api.skill('nudge_wrist_joint', joint=jj, delta=d, dur=dur)
            if read_mode == 'geom':
                z1 = api.geom()['object']['z_up']
            else:
                z1 = r['z_up']
            if z1 > z0 + min_gain:
                z = z1
                improved = True
                log.append([jj, np.round(d, 3).item(),
                            np.round(z0, 4).item(), np.round(z1, 4).item()])
            else:
                if sign_reversal:
                    r2 = api.skill('nudge_wrist_joint', joint=jj,
                                   delta=-reverse * d, dur=dur)
                    if read_mode == 'geom':
                        z2 = api.geom()['object']['z_up']
                    else:
                        z2 = r2['z_up']
                    if z2 > z0 + min_gain:
                        z = z2
                        d = -d
                        improved = True
                        log.append([jj, np.round(d, 3).item(),
                                    np.round(z0, 4).item(),
                                    np.round(z2, 4).item()])
                    else:
                        r3 = api.skill('nudge_wrist_joint', joint=jj,
                                       delta=d, dur=dur)
                        if read_mode == 'geom':
                            z = api.geom()['object']['z_up']
                        else:
                            z = r3['z_up']
                        break
                else:
                    api.skill('nudge_wrist_joint', joint=jj, delta=-d,
                              dur=dur)
                    break
            tries += 1
    after = np.round(api.geom()['object']['z_up'], 4).item()
    if out == 'single':
        return {'zup_before': before, 'zup_after': after}
    return {'log': log, 'zup_after': after}
