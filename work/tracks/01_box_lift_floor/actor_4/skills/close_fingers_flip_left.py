def run(api, angle=0.9, dur=0.8, left_finger_flip=1, rthumb_min=0.30,
        box_ref=None):
    """Close both hands on the opposed faces with the LEFT index/middle
    finger command signs reversed relative to the joint-range sign rule.

    Removes: finger tips that travel OUTBOARD on the left hand.  The
    range-sign rule sends the left index/middle joints negative, which
    carries their tips away from the box face (left index_1 y 0.1607 ->
    0.1914 m against a 0.132 m face) while the right tips travel inboard and
    make the contacts; the result is a right-side-only grip.  Reversing the
    sign of the left index_0/index_1/middle_0/middle_1 commands only (the
    thumbs keep the sign rule, and the right thumb keeps its 0.30 rad floor
    against a new torque-saturation path) sends the left tips inboard.

    Contract: pre: both pads seated on the opposed faces and the commanded
    pose is a valid upper-body target -> post: one smoothstep ramp (dur s at
    0.02 s steps) of the hand joints to +-`angle` clipped to each joint
    range, left index/middle negated when `left_finger_flip` is truthy;
    returns `{target, finger_cmd, contacts, sides, rthumb_cmd, moved_l1_m,
    record}` where `contacts`/`sides` are measured immediately after the
    ramp and `sides` is a SECOND, separate contact read (both reads are part
    of the pinned sequence).  Does not move the base or the arms.

    Side effects: `upper` and `joint_ranges` queries, the step_goal ramp,
    then observe -> hand_contacts_side -> hand_contacts_side -> observe.
    WARNING (PRIMITIVES.md line 288): these reads are physical, so this
    exact order is part of the verified behaviour.
    """
    cur = np.array(api.upper(), dtype='float64')
    rng = api.joint_ranges()
    tgt = np.array(cur, dtype='float64')
    i_ = 0
    fcmd = []
    for nm in rng:
        if '_hand_' in nm:
            lo_ = rng[nm]['lo']
            hi_ = rng[nm]['hi']
            sg = 0.0
            if np.abs(hi_) > np.abs(lo_):
                sg = 1.0
            if np.abs(lo_) > np.abs(hi_):
                sg = -1.0
            v = np.clip(sg * angle, lo_, hi_)
            if nm.startswith('right_hand_thumb_1'):
                v = np.maximum(v, rthumb_min)
            if left_finger_flip and nm.startswith('left_hand_') and (
                    'index_' in nm or 'middle_' in nm):
                v = np.clip(-sg * angle, lo_, hi_)
            tgt[i_] = v
            fcmd.append([nm, np.round(v, 4).item()])
        i_ += 1
    n = np.maximum(1.0, np.ceil(dur / 0.02))
    kk = 0
    while kk < n:
        aa = (kk + 1.0) / n
        aa = aa * aa * (3.0 - 2.0 * aa)
        api.step_goal(upper=cur + aa * (tgt - cur), dur=0.02)
        kk += 1
    o = api.observe()
    cc = o['hand_object_contacts']
    sr = api.hand_contacts_side()
    cs = {'left': sr.get('left'), 'right': sr.get('right')}
    sr2 = api.hand_contacts_side()
    b_after = np.array(api.observe()['object']['pos'])
    rec = {'phase': 'close', 'base': o['base'],
           'box': o['object']['pos'],
           'contacts': o['hand_object_contacts'],
           'sides': {'left': sr2.get('left'), 'right': sr2.get('right')},
           'rthumb_cmd': tgt[29].item(),
           'finger_cmd': fcmd,
           'moved_l1_m': np.round(np.sum(np.abs(
               b_after - np.array(box_ref, dtype='float64'))), 4).item()}
    return {'target': tgt.tolist(), 'finger_cmd': fcmd,
            'contacts': cc, 'sides': cs,
            'rthumb_cmd': tgt[29].item(), 'moved_l1_m': rec['moved_l1_m'],
            'record': rec, 'log': [rec]}
