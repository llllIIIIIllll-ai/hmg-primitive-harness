"""Command-preserving settle / hold (no new arm command).

MECHANISM NAME
    hold_commanded_pose -- let simulation time pass while re-sending the
    LAST commanded 31-D upper-body vector unchanged, so the arms neither
    move nor are re-planned from the measured state.

FAILURE SIGNATURE IT ADDRESSES
    Re-reading the measured joints and commanding them back ("freeze where
    you are") is NOT a hold: the measured pose lags the command, so a
    measured-joint re-command relaxes the arm towards the tracked pose and,
    on a free-rolling or lightly supported object, that small arm motion is
    enough to disturb it.  Measured on 06_drill_pnp: a closed grip relaxes
    under plain base motion and a carry released with only two one-sided
    support contacts tips flat, so the carry and the post-release hold must
    re-send the commanded vector, not the measured one.  On
    04_push_shelf_forward the same hold is what converts the cart's measured
    0.07-0.13 m coast into travel without changing the push: the body stops
    driving but the arm pose stays exactly as it was at the last burst.

THE MECHANISM (what it does physically)
    u = api.commanded() (the last 31-D target actually sent; None if none
    was ever sent) unless the caller passes an explicit upper vector, then
    api.step_goal(upper=u, dur=dur).  step_goal executes
    max(1, int(dur/control_timestep)) control steps, so dur > 0 is required
    for time to actually pass; the body's WBC keeps balancing while the
    upper target is held.

SIGNATURE (named params + defaults)
    run(api, **kwargs)
      dur    float, default 1.0   hold length in seconds (> 0 to advance time)
      upper  array, default None  explicit 31-D target; None = use
                                  api.commanded()

PRECONDITIONS
    - A prior command exists (api.commanded() is not None) OR the caller
      passes `upper`.  If neither holds, step_goal(upper=None) falls back to
      the measured joints -- the same thing the promoted program did, so the
      fallback is preserved rather than defended against.
    - dur > 0; dur <= 0 would still execute one control step and is NOT a
      no-op (the caller must guard with `if dur > 0`).

POSTCONDITIONS
    - Advances api.t by dur (at least one control step) with the 31-D upper
      target unchanged.
    - Returns {'t': api.t}.  Issues NO scene read: adding one would change
      the trajectory (see the observation-reads lesson).

SIDE EFFECTS
    - Simulation time advances; the WBC balances the body; any free object
      coasts, settles or tips under gravity and its own contacts.  Nothing
      about the upper-body target changes.

SCOPE
    This is the generic "hold still without disturbing what you hold" skill:
    04 (coast settle between push bursts and the final hold), 06 (carry and
    post-release park), 08 (clearance-first extraction's hold before the
    height boost), 07 (post-release forward margin).  Breaks if the caller
    needs the arm to TRACK something during the hold (a moving object, a
    closed grip on a slipping object) -- this skill deliberately does not
    correct, so a commanded pose that no longer matches the object cannot be
    repaired here.
"""


def run(api, **kwargs):
    dur = kwargs.get('dur', 1.0)
    upper = kwargs.get('upper', None)
    u = api.commanded() if upper is None else upper
    api.step_goal(upper=u, dur=dur)
    return {'t': api.t}
