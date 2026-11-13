def run(api, lift_h=0.68, lift_dur=2.5, sample_every=20):
    """Stand the whole body up while holding the last commanded upper pose,
    sampling the box height and hand contacts during the rise.

    Removes: a lift with the arms re-solved mid-rise.  Re-solving the arms
    while the base rises moves the pads relative to the grasped faces; holding
    the commanded upper pose and raising only the base height (nav zeroed)
    keeps the grip, and the box rises with the body.

    Contract: pre: grip closed on the opposed faces and the fingers still
    commanded closed (last commanded upper pose is the grasp pose) -> post:
    the base height ramps in a smoothstep from its measured value to
    `lift_h` m over `lift_dur` s at 0.02 s steps with `nav=[0,0,0]` and
    `upper` fixed to `api.commanded()`; returns `{h_from, h_to, samples,
    commanded}`.  Field names and read order match the floor-task evidence
    (`lift<k>` with base, box, box_z_hi, contacts, sides).

    Side effects: every `sample_every`-th step (k % 20 == 19) one `observe`
    plus one `geom_detail`, and one `hand_contacts_side` for the sample.
    Commands the base height and nav only; never the arms.
    """
    qc2 = api.commanded()
    h_from = api.observe()['base'][2]
    h_to = lift_h
    nsteps = np.maximum(1.0, np.ceil(lift_dur / 0.02))
    samples = []
    k = 0
    while k < nsteps:
        aa = (k + 1.0) / nsteps
        aa = aa * aa * (3.0 - 2.0 * aa)
        hc = h_from + (h_to - h_from) * aa
        api.step_goal(upper=qc2, nav=[0.0, 0.0, 0.0], height=hc, dur=0.02)
        if k % sample_every == sample_every - 1:
            o = api.observe()
            g2 = api.geom_detail()
            sr = api.hand_contacts_side()
            samples.append({'phase': 'lift%d' % k, 'base': o['base'],
                            'box': o['object']['pos'],
                            'box_z_hi': g2['world_hi'][2],
                            'contacts': o['hand_object_contacts'],
                            'sides': {'left': sr.get('left'),
                                      'right': sr.get('right')}})
        k += 1
    return {'h_from': np.float64(h_from).item(),
            'h_to': np.float64(h_to).item(),
            'samples': samples, 'log': samples,
            'commanded': [np.float64(x).item() for x in qc2]}
