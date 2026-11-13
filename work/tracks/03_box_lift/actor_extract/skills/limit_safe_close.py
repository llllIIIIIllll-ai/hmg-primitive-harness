"""limit_safe_close -- finger closure clamped to the hand joint range, with contact gate.

MECHANISM NAME
    Joint-range-clamped finger closure with a contact gate and release-on-fail.

FAILURE SIGNATURE ADDRESSED
    Commanding a fixed grip angle that exceeds the hand joint limit makes the
    position servo saturate: the fingers stop early, the pads never load the
    side faces, and the measured contact count on close is 2 in the best case
    and 0 when the pads were also shy.  A commanded angle above the limit was
    measured to leave the 03_box_lift grasp with a one-sided load that slipped
    at the first lift.  Closing blind, without checking the resulting contact
    count, has no way to notice the empty or one-sided grasp.

MECHANISM
    Read the hand joint ranges from the live model and take
    amax = min over every joint whose name contains `_hand_` of
    max(|lo|, |hi|) -- the tightest symmetric bound the fingers can reach.
    Command the grip at min(grip_angle, amax - margin) so the command stays
    inside the range.  After the closure, read the live hand-object contact
    count and the commanded 31-D vector, and measure the largest commanded
    magnitude over the same hand joints.  If the contact count is below
    min_contacts the grasp is not holding: on request, command a small
    negative angle for `release_dur` to unload the pads so the caller can
    retract without dragging the object.

CONTRACT
    run(api, grip_angle=1.0, grip_dur=1.2, margin=0.02, min_contacts=2,
        release_on_fail=True, release_angle=-0.5, release_dur=0.6)
        -> {'closed': bool, 'contacts': int,
            'angle': float, 'hand_limit_min_abs': float,
            'cmd_hand_maxabs': float, 'commanded': [31 floats]|None,
            'state': {...}}

    grip_angle       requested closure angle, radians
    grip_dur         duration of the closing command
    margin           safety margin subtracted from the joint bound
    min_contacts     contact count at or above which the grasp counts as closed
    release_on_fail  whether to unload the fingers when the grasp failed
    release_angle    unloading angle, radians
    release_dur      duration of the unloading command

    preconditions:  hand joints present in joint_ranges(); pads already placed
                    on the object faces; `commanded()` returns the held 31-D
                    vector (or None when no goal was ever commanded).
    postconditions: fingers are commanded inside the joint range; `contacts`
                    is the measured hand-object contact count after closure;
                    when the grasp failed and release_on_fail is set the fingers
                    have been unloaded.  `commanded` is the post-close vector.
    side effects:   moves the finger joints only; if the grasp fails the
                    unload motion can let the object settle by a few mm.
"""


def run(api, grip_angle=1.0, grip_dur=1.2, margin=0.02, min_contacts=2,
        release_on_fail=True, release_angle=-0.5, release_dur=0.6):
    rr = api.joint_ranges()
    amax = 9.0
    for nm in rr:
        if '_hand_' in nm:
            mm = np.maximum(np.abs(rr[nm]['lo']), np.abs(rr[nm]['hi']))
            if mm < amax:
                amax = mm
    ang = np.minimum(np.float64(grip_angle),
                     amax - np.float64(margin)).item()
    api.close_grip(angle=ang, dur=grip_dur)
    st1 = api.observe()
    nc = st1.get('hand_object_contacts') or 0
    cmdv = api.commanded()
    hmax = 0.0
    if cmdv is not None:
        i = 0
        for nm in api.addr:
            if '_hand_' in nm:
                v = np.abs(cmdv[i]).item()
                if v > hmax:
                    hmax = v
            i = i + 1
    closed = nc >= min_contacts
    if (not closed) and release_on_fail:
        api.close_grip(angle=release_angle, dur=release_dur)
    return {'closed': closed, 'contacts': nc, 'angle': ang,
            'hand_limit_min_abs': amax.item(),
            'cmd_hand_maxabs': hmax, 'commanded': cmdv,
            'state': st1}
