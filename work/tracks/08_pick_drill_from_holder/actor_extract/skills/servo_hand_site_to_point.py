def run(api, **kw):
    """Servo a measured hand site to a world-space point in capped steps.

    Mechanism: closed-loop fingertip servo -- every iteration reads the live
    grip site, moves it toward the point by at most a step cap, and stops
    inside a tolerance.  Removes the open-loop reach failure where a single
    arm command lands short or overshoots because the tracked hand pose
    differs from the commanded one.
    """
    target = np.array(kw.get('target'))
    side = kw.get('side', 'left')
    tol = kw.get('tol', 0.006)
    step_cap = kw.get('step', 0.022)
    dur = kw.get('dur', 0.5)
    iters = kw.get('iters', 220)
    max_steps = kw.get('max_steps', 12)
    RB = kw.get('R_body', [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])
    k = 0
    site = target
    while k < max_steps:
        site = np.array(api.observe()['object']['hand'][side])
        err = target - site
        errn = np.sqrt(np.sum(err * err))
        if errn < tol:
            break
        step = err * (step_cap / errn) if errn > step_cap else err
        api.reach_box(targets={side: (site + step).tolist()}, sides=(side,),
                      R_body=RB, dur=dur, iters=iters)
        k += 1
    return {'steps': k, 'site': site.tolist()}
