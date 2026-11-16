def run(api, shoulder_pitch=0.5, elbow=0.5, wrist_pitch=0.0, dur=1.0,
        tol=0.2, **kw):
    """Folded arm pre-shape in JOINT space (a posture, not a Cartesian
    target): shoulders back, elbows bent, wrists neutral; the other arm joints
    hold.  The pads should clear the drill while the base moves."""
    jr = api.joint_ranges()
    q = api.upper()
    arm = []
    for nm in jr:
        if '_hand_' in nm:
            continue
        if '_shoulder_' in nm or '_elbow_' in nm or '_wrist_' in nm:
            arm.append(nm)
    idx = {}
    for nm in arm:
        addr = api.addr[nm]
        k = 0
        while k < 31:
            if api.qpos_addr[k] == addr:
                idx[nm] = k
                break
            k = k + 1
    for nm in arm:
        if nm not in idx:
            continue
        t = q[idx[nm]]
        if 'shoulder_pitch' in nm:
            t = shoulder_pitch
        elif 'elbow' in nm:
            t = elbow
        elif 'wrist_pitch' in nm:
            t = wrist_pitch
        q[idx[nm]] = np.clip(t, jr[nm]['lo'], jr[nm]['hi'])
    api.step_goal(upper=q, dur=dur)
    q1 = api.upper()
    err = 0.0
    for nm in arm:
        if nm not in idx:
            continue
        e = np.abs(q1[idx[nm]] - q[idx[nm]])
        if e > err:
            err = e
    st = api.observe()
    ob = st.get('object') or {}
    ok = True if err <= tol else False
    return {'ok': ok, 'arm_target_error': err,
            'target_joints': q.tolist(), 'pads': ob.get('hand'),
            'binding_constraint': None if ok else 'joint_tracking'}
