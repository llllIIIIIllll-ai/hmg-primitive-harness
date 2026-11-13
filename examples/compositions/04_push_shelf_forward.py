"""Task program: push the mobile shelving cart to the sampled target zone.

Composition only -- every physical action lives in a registered skill:

    measure_scene_state        pinned ordered scene probe (sensing)
    walk_to_first_contact      burst walk to the contact standoff
    steer_behind_leader        bounded lateral lead + heading hold
    push_object_to_target_x    burst push to a world-x threshold (+ settle)
    hold_commanded_pose        command-preserving settle / final hold

The program is the promoted 04_push_shelf_forward behaviour re-expressed:
the SAME api reads and commands in the SAME order (the read sequence is
load-bearing -- see the observation-reads lesson), staged as
approach -> fast -> slow -> settle -> fine -> hold.

Kwarg defaults are the promoted program's defaults, so calling this program
with NO kwargs reproduces the promoted per-seed outcome.
"""


def run(api, **kwargs):
    vx_fast = kwargs.get('vx_fast', 0.25)
    vx_slow = kwargs.get('vx_slow', 0.15)
    vx_fine = kwargs.get('vx_fine', 0.12)
    dur_fast = kwargs.get('dur_fast', 1.0)
    dur_slow = kwargs.get('dur_slow', 0.5)
    dur_fine = kwargs.get('dur_fine', 0.4)
    app_gap = kwargs.get('app_gap', 0.80)
    slow_lead = kwargs.get('slow_lead', 0.55)
    stop_lead = kwargs.get('stop_lead', 0.22)
    fine_lead = kwargs.get('fine_lead', 0.08)
    ky = kwargs.get('ky', 1.0)
    kb = kwargs.get('kb', 0.8)
    eb_soft = kwargs.get('eb_soft', 0.0)
    cy_max = kwargs.get('cy_max', 0.08)
    kyw = kwargs.get('kyw', 1.0)
    cw_max = kwargs.get('cw_max', 0.25)
    ref_mode = kwargs.get('ref_mode', 'target')
    n_app = kwargs.get('n_app', 6)
    n_fast = kwargs.get('n_fast', 18)
    n_slow = kwargs.get('n_slow', 8)
    n_fine = kwargs.get('n_fine', 5)
    settle_dur = kwargs.get('settle_dur', 0.8)
    hold_dur = kwargs.get('hold_dur', 1.0)

    gains = {'ky': ky, 'kb': kb, 'eb_soft': eb_soft, 'cy_max': cy_max,
             'kyw': kyw, 'cw_max': cw_max}

    trace = []

    def rec(phase, k, s, cmd):
        trace.append({'phase': phase, 'k': k, 'cmd': cmd,
                      'base': s['base'], 'shelf': s['shelf'],
                      'box': s['box'], 'target': s['target'],
                      'zup': s['zup'], 'yaw': np.round(s['yaw'], 2),
                      'contacts': s['contacts']})

    # measured reference line: the sampled target y, or the cart's own y
    s = api.skill('measure_scene_state')
    tx = s['target'][0]
    ref_y = s['target'][1] if ref_mode == 'target' else s['shelf'][1]
    rec('start', 0, s, None)

    # 1. approach to the contact standoff (stops on contact or gap)
    a = api.skill('walk_to_first_contact', app_gap=app_gap, max_bursts=n_app,
                  ref_y=ref_y, **gains)
    trace.extend(a['trace'])

    # 2. fast push until the coast-sized lead, 3. slow push until the
    #    braking-sized lead -- same skill, one scale apart
    f = api.skill('push_object_to_target_x', target_x=tx, vx=vx_fast,
                  dur=dur_fast, lead=slow_lead, max_bursts=n_fast,
                  settle_dur=0.0, phase='push', ref_y=ref_y, **gains)
    trace.extend(f['trace'])
    sl = api.skill('push_object_to_target_x', target_x=tx, vx=vx_slow,
                   dur=dur_slow, lead=stop_lead, max_bursts=n_slow,
                   settle_dur=0.0, phase='slow', ref_y=ref_y, **gains)
    trace.extend(sl['trace'])

    # 4. let the cart coast, then fine pushes with a settle between them
    api.skill('hold_commanded_pose', dur=hold_dur)
    s = api.skill('measure_scene_state')
    rec('settle', 0, s, None)
    fi = api.skill('push_object_to_target_x', target_x=tx, vx=vx_fine,
                   dur=dur_fine, lead=fine_lead, max_bursts=n_fine,
                   settle_dur=settle_dur, phase='fine', ref_y=ref_y, **gains)
    trace.extend(fi['trace'])

    # 5. hold and report the final geometry
    api.skill('hold_commanded_pose', dur=hold_dur)
    s = api.skill('measure_scene_state')
    rec('end', 0, s, None)
    nb = 0
    for c in api.contact_pairs(limit=60):
        nm = c['b1'] + ' ' + c['b2']
        if 'shelf_main' in nm and 'box_main' in nm:
            nb += 1
    dx = s['shelf'][0] - s['target'][0]
    dy = s['shelf'][1] - s['target'][1]
    return {'shelf': s['shelf'], 'box': s['box'], 'target': s['target'],
            'shelf_zup': s['zup'], 'cart_yaw': np.round(s['yaw'], 2),
            'dist_xy_est': np.round(np.sqrt(dx * dx + dy * dy), 4),
            'shelf_box_contacts': nb, 'base': s['base'],
            'bursts': {'approach': a['bursts'], 'fast': f['bursts'],
                       'slow': sl['bursts'], 'fine': fi['bursts']},
            'trace': trace}
