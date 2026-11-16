def run(api, dx=-0.16, dy=0.11, z=0.94, stop_err=0.02, max_steps=30,
        max_delta=0.08, trace_contacts=False, contact_match='drill',
        orient_mode='side', orientation='keep', **kw):
    """Untuck to a pre-grasp pose AFTER the base servo, routed through the
    same true closed loop as the live reach (arm_pose_ik_live: solve with
    apply=False, clamp the per-step joint delta, apply one control step,
    re-observe, live target each iteration).

    The target is measured from the LIVE object: obj + dx*f +/- dy*p with
    f the base->object approach axis and p its lateral, so the palms end
    behind the grasp point on the same branch the reach will use.  The
    closed loop -- not a single open-loop QP apply -- is what closes the
    gap from the folded tuck.
    """
    r = api.skill('arm_pose_ik_live',
                  grasp={'dx': dx, 'dy': dy, 'z': z},
                  orientation=orientation, stop_err=stop_err,
                  max_steps=max_steps, max_delta=max_delta,
                  trace_contacts=trace_contacts,
                  contact_match=contact_match, orient_mode=orient_mode,
                  **kw)
    worst = r.get('worst_pad_error')
    ok = True if (worst is not None and worst <= stop_err
                  and r.get('resets') == 0 and r.get('nans') == 0) else False
    binding = None
    if r.get('resets'):
        binding = 'sim_reset'
    elif r.get('nans'):
        binding = 'nan_target'
    elif worst is None:
        binding = 'no_solution'
    elif worst > stop_err:
        binding = 'pad_error'
    return {'ok': ok, 'pad_error': r.get('pad_error'),
            'worst_pad_error': worst,
            'targets_final': r.get('targets_final'),
            'solver': r.get('solver'), 'joint_margin_min':
            r.get('joint_margin_min'),
            'sim_time': r.get('sim_time'), 'steps': r.get('steps'),
            'resets': r.get('resets'), 'nans': r.get('nans'),
            'trace': r.get('trace'), 'first_step': r.get('first_step'),
            'base_drift': r.get('base_drift'),
            'normal_error_deg': r.get('normal_error_deg'),
            'binding_constraint': binding}
