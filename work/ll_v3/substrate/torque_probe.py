def run(api, joint='left_shoulder_roll_joint', step=0.002, steps=3,
        sign=1.0, **kw):
    """Command a small joint step and read the SIM actuator force vs its
    ctrlrange: confirms whether the small-step non-motion is a friction
    deadband (torque below frictionloss) or a ctrlrange saturation."""
    m, d = api.raw
    jid = m.joint(api.sim_name[joint]).id
    aid = -1
    a = 0
    while a < m.nu:
        if m.actuator_trnid[a][0] == jid:
            aid = a
            break
        a = a + 1
    jj = api.addr[joint]
    idx = None
    k = 0
    while k < 31:
        if api.qpos_addr[k] == jj:
            idx = k
            break
        k = k + 1
    tr = []
    n = 0
    while n < steps:
        q = api.upper()
        cmd = q.copy()
        cmd[idx] = q[idx] + sign * step
        api.step_goal(upper=cmd, nav=[0.0, 0.0, 0.0], dur=0.02)
        q1 = api.upper()
        f = None
        cr = None
        fr = None
        if aid >= 0:
            f = d.actuator_force[aid]
            cr = [m.actuator_ctrlrange[aid][0], m.actuator_ctrlrange[aid][1]]
            fr = m.dof_frictionloss[m.jnt_dofadr[jid]]
        tr.append({'k': n, 'delta': q1[idx] - q[idx], 'force': f,
                   'ctrlrange': cr, 'frictionloss': fr})
        n = n + 1
    return {'ok': True, 'joint': joint, 'aid': aid,
            'step': sign * step, 'trace': tr}
