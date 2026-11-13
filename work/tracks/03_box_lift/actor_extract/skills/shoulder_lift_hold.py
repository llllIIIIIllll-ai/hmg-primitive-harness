"""shoulder_lift_hold -- staged stand-up with shoulder-pitch ratchet and contact gates.

MECHANISM NAME
    Ratcheted shoulder-pitch lift: raise the whole-body goal to a standing
    height in three stages, then repeat bounded shoulder-pitch decrements on the
    commanded 31-D vector while polling the object height and contact count,
    stopping on a rise gate, a lost-grasp gate, or a contact-loss gate.

FAILURE SIGNATURE ADDRESSED
    Two measured failures on 03_box_lift.  (a) Commanding the full lift as one
    long goal step outruns the grasp: the object slips before the fingers can
    load, and the measured final state shows the base at the old height with
    the object back on the table.  (b) Continuing to ratchet after the object
    has risen past the target drives the arms into the body and the reported
    contact count climbs while the object stops rising; without a stop gate the
    loop wastes the budget and can drop the object.

MECHANISM
    Read the object z once for reference (the start height).  Command
    step_goal(upper=held 31-D vector, height=h1) and then (h2) in `stand_dur`
    and `stand_dur2`, then the same held vector at h2 for `settle_dur` to let
    the pads take the load.  Then per iteration: read the object z and the
    contact count; stop when z >= z_start + target_rise (rise gate); abort with
    'grip lost' when z < z_start + min_rise (the object is still on the table);
    stop when contacts < contact_gate_n while z >= z_start + contact_gate_rise
    (the grasp is unloading after it already lifted).  Otherwise copy the
    commanded vector, subtract `pitch_step` from every joint whose name ends in
    `shoulder_pitch_joint`, and command it at height h2 for `lift_dur`.

CONTRACT
    run(api, commanded=None, z_start=None, stand_height1=0.82,
        stand_height2=0.90, stand_dur=1.5, stand_dur2=1.5, settle_dur=0.5,
        target_rise=0.14, min_rise=0.02, contact_gate_n=3,
        contact_gate_rise=0.09, iters=8, pitch_step=0.06, lift_dur=1.6,
        height_final=0.90)
        -> {'lifts': [{'z','contacts','base_z','stepped'?}],
            'abort': str|None, 'final_geom': {...}|None,
            'final_state': {...}}

    commanded        31-D upper-body vector holding the grasp; read live when None
    z_start          object z reference; read from the live object when None
    stand_height1    first stand height goal, metres
    stand_height2    second and final stand height goal, metres
    stand_dur        duration of the first stand goal
    stand_dur2       duration of the second stand goal
    settle_dur       duration of the settle goal that ends the stand
    target_rise      rise above z_start that ends the lift, metres
    min_rise         rise below which the grasp is declared lost, metres
    contact_gate_n   contact count below which an already-risen lift stops
    contact_gate_rise rise above z_start required before that gate applies
    iters            ratchet budget
    pitch_step       shoulder-pitch decrement per ratchet, radians
    lift_dur         duration of each ratchet step
    height_final     base height commanded with every ratchet and final settle

    preconditions:  the grasp is closed and `commanded()` holds the closed
                    posture; the object has a live pose.
    postconditions: the object has been lifted by at least target_rise, or the
                    loop ended through the lost-grasp or contact-loss gate, or
                    the ratchet budget expired; `lifts` holds one measured
                    sample per iteration and `final_state` is the last read.
    side effects:   commands the arms and the base height through the WBC;
                    moves the object.  The commanded vector is never written
                    back to the caller -- callers that want the last ratcheted
                    posture must read `commanded()` themselves.
"""


def _read_box(api):
    st = api.observe()
    b = st.get('object') or {}
    if not b.get('pos'):
        return None
    a = None
    for o in api.scene_objects(detail=False):
        if o.get('name') == b.get('name'):
            a = o
    if a is None:
        return None
    return {'pos': np.asarray(b['pos'], np.float64),
            'lo': np.asarray(a['aabb_lo'], np.float64),
            'hi': np.asarray(a['aabb_hi'], np.float64)}


def run(api, commanded=None, z_start=None, stand_height1=0.82,
        stand_height2=0.90, stand_dur=1.5, stand_dur2=1.5, settle_dur=0.5,
        target_rise=0.14, min_rise=0.02, contact_gate_n=3,
        contact_gate_rise=0.09, iters=8, pitch_step=0.06, lift_dur=1.6,
        height_final=0.90):
    cmd = commanded
    if cmd is None:
        cmd = api.commanded()
    lifts = []
    abort = None
    final_geom = None
    final_state = None
    if cmd is None:
        return {'lifts': lifts, 'abort': 'no commanded',
                'final_geom': final_geom, 'final_state': final_state}
    cmd_closed = np.asarray(cmd, np.float64)
    z0 = z_start
    if z0 is None:
        b0 = _read_box(api)
        if b0 is None:
            return {'lifts': lifts, 'abort': 'no object',
                    'final_geom': final_geom, 'final_state': final_state}
        z0 = b0['pos'][2]

    api.step_goal(upper=cmd_closed.tolist(), height=stand_height1,
                  dur=stand_dur)
    api.step_goal(upper=cmd_closed.tolist(), height=stand_height2,
                  dur=stand_dur2)
    api.step_goal(upper=cmd_closed.tolist(), height=stand_height2,
                  dur=settle_dur)

    it = 0
    while it < iters:
        st2 = api.observe()
        ob = st2.get('object') or {}
        if not ob.get('pos'):
            abort = 'no object'
            break
        z = ob['pos'][2]
        nc = st2.get('hand_object_contacts') or 0
        lifts.append({'z': z, 'contacts': nc, 'base_z': st2['base'][2]})
        if z >= z0 + np.float64(target_rise):
            break
        if z < z0 + np.float64(min_rise):
            abort = 'grip lost'
            break
        if nc < contact_gate_n and z >= z0 + np.float64(contact_gate_rise):
            break
        q = np.asarray(api.commanded(), np.float64).copy()
        i = 0
        for nm in api.addr:
            if nm.endswith('shoulder_pitch_joint'):
                q[i] = q[i] - np.float64(pitch_step)
            i = i + 1
        api.step_goal(upper=q.tolist(), height=height_final, dur=lift_dur)
        lifts[-1]['stepped'] = True
        it = it + 1

    g = api.geom().get('object')
    final_geom = g
    final_state = api.observe()
    return {'lifts': lifts, 'abort': abort,
            'final_geom': final_geom, 'final_state': final_state}
