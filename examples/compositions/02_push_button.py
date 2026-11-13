"""02_push_button: press the red button on the control panel.

Task program, a composition of three reusable whole-body skills:

  1. walk_to_reach            puts the button inside arm reach, gated by the
                              MEASURED button-to-base gap;
  2. servo_fingertip_to_point moves the index fingertip to a stand-off point
                              in front of the button, closing on the MEASURED
                              fingertip;
  3. press_until_joint_stop   steps the fingertip in along +x while watching
                              the button's own slide joint, stops when the
                              button bottoms out, then holds and settles.

The task-level judgement (a press counts as success at q <= -0.010) stays
here, out of the skills.
"""

STANDOFF_BACKOFF = 0.011     # button front face -> nominal fingertip point (m)
PRESSED_Q = -0.010           # button slide joint value counted as pressed


def run(api, side='right', standoff=0.04, approach_iters=8, gain=0.7,
        dur=0.45, press_dx=0.003, press_steps=26, q_stop=-0.0145,
        press_iters=2, walk_phases=None, max_walk_extra=6):
    if walk_phases is None:
        walk_phases = [[0.3, 0.0, 1.2], [0.3, 0.05, 0.5], [0.25, 0.02, 0.4]]

    walk = api.skill('walk_to_reach', target_match='button', gapx_max=0.44,
                     phases=walk_phases, burst_x=0.22, burst_y=0.02,
                     burst_dur=0.4, max_extra=max_walk_extra,
                     axis=0, joint_index=0)

    tgt = np.array(walk['target_pos']) + np.array(
        [-STANDOFF_BACKOFF - standoff, 0.0, 0.0])

    appr = api.skill('servo_fingertip_to_point', side=side, target=tgt,
                     gain=gain, dur=dur, max_iters=approach_iters,
                     tol=0.008, tip_link='index_1', mesh_tag='_vis',
                     tip_half=0.0334, watch_match='button',
                     phase='approach', tag='main')

    pr = api.skill('press_until_joint_stop', side=side, target0=tgt,
                   step=(press_dx, 0.0, 0.0), max_steps=press_steps,
                   q_stop=q_stop, object_match='button', joint_index=0,
                   servo_gain=gain, servo_dur=dur,
                   servo_iters=press_iters, servo_tol=0.008,
                   tip_link='index_1', mesh_tag='_vis', tip_half=0.0334,
                   object_token='control_panel_button',
                   support_token='control_panel_main_body',
                   contact_limit=60, settle_dur=0.6)

    q = pr['button_q']
    return {'pressed': (q is not None and q <= PRESSED_Q),
            'button_q': q,
            'button_pos': pr['button_pos'],
            'btn_contacts': pr['btn_contacts'],
            'panel_contacts': pr['panel_contacts'],
            'trace': walk['trace'] + appr['trace'] + pr['trace'],
            'observe': pr['observe'],
            'base': pr['base']}
