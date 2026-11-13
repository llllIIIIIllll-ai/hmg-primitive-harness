"""Task program: drill pick-and-place, composed from whole-body skills.

Copy of actor_extract/compose.py with ONLY the two leveling calls replaced by
the merged `level_upright` skill, at the parameter settings that reproduce
each original element-for-element. Call order, glue, keys and every other
skill call are unchanged.

`level_upright_single_axis(joint=15, delta=0.2, target=0.97, min_gain=0.002,
max_tries=3, dur=0.5)`
  -> level_upright(joints=(15,), step=0.2, target=0.97, min_gain=0.002,
                   max_tries=3, dur=0.5, reverse=1.0, sign_reversal=False,
                   read_mode='nudge', target_stop='gt', out='single')

`level_upright_two_axis(joints=(14, 15), step=0.08, target=0.996,
min_gain=0.001, max_tries=4, dur=0.35, reverse=2.0)`
  -> level_upright(joints=(14, 15), step=0.08, target=0.996, min_gain=0.001,
                   max_tries=4, dur=0.35, reverse=2.0, sign_reversal=True,
                   read_mode='geom', target_stop='ge', out='two')

The composed sequence:
  1. servo the base to the origin-table stance
  2. reach the right palm to a standoff short of the drill grip column
  3. park the left hand radially clear before the fingers close
  4. curl the fingers onto the handle at the standoff
  5. hoist in small palm steps, aborting on grasp loss
  6. coarse wrist-roll leveling (two nudges)
  7. raise until the object clears the target table top
  8. carry to the target pose, with a raise-and-re-walk stall recovery
  9. coarse single-axis leveling
 10. two-axis leveling to upright
 11. seat the still-gripped object on the support
 12. open the fingers and let it settle with no palm motion
 13. park both hands radially away from the seated object
 14. hold the posture and measure settle drift plus final evidence
"""


def run(api, hold=2.0, grip=1.3, park=0.09, clear=0.85, **kw):
    out = {'phases': []}

    w = api.skill('servo_base_to_target_pose', x=0.50, y=0.0, yaw_deg=0.0,
                  upper=None, tol=0.04, yaw_tol=6.0, max_iter=8, dur=0.8,
                  vmax=0.22)
    out['phases'].append(['walk', w['last'], api.base_state()])

    r = api.skill('reach_palm_to_object_standoff', side='right', back=0.22,
                  lateral=-0.046, z=0.86, dur=1.2)
    out['standoff'] = r['target']
    out['phases'].append(['reach_pre', r['residual'], r['obj_after'],
                          r['z_up_after']])

    p = api.skill('park_hand_clear_of_object', side='left', other='right',
                  park=park, trigger=0.45, dz=0.02, dur=0.9)
    out['left_park'] = p['info']
    out['phases'].append(['left_park', p['phase'][0], p['phase'][1],
                          p['phase'][2]])

    c = api.skill('curl_fingers_onto_handle', angle=grip, dur=1.1)
    out['phases'].append(['close', c['obj'], c['z_up'], c['hand_contacts'],
                          c['sides']])

    h = api.skill('hoist_gripped_object_in_steps', side='right', step=0.015,
                  max_steps=3, left_max=0, right_min=1, z_up_min=0.85,
                  dur=0.4)
    out['lift_log'] = h['log']
    out['lift_lost'] = h['lost']
    out['phases'].append(['lift', h['phase'][0], h['phase'][1],
                          h['phase'][2], h['phase'][3]])
    if h['lost']:
        out['final'] = h['final']
        return out

    api.skill('nudge_wrist_joint', joint=14, delta=0.25, dur=0.5)
    api.skill('nudge_wrist_joint', joint=14, delta=0.15, dur=0.5)

    u = api.skill('raise_gripped_until_clear', clear=clear, margin=0.005,
                  step=0.02, max_steps=2, side='right', min_hand=1,
                  min_right=1, dur=0.4)
    out['precarry'] = {'palm_z': u['palm_z'], 'obj': u['obj'],
                       'z_up': u['z_up'], 'clear': u['clear'],
                       'raises': u['raises'], 'sides': u['sides'],
                       'hand_n': u['hand_n']}
    out['phases'].append(['precarry', u['phase'][0], u['phase'][1],
                          u['phase'][2], u['phase'][3]])

    k = api.skill('carry_gripped_object_to_pose', x=0.15, y=0.55,
                  yaw_deg=90.0, tol=0.06, yaw_tol=8.0, max_iter=10,
                  upper=u['cmd'], recover=2, recover_iter=6, recover_dz=0.02,
                  recover_yaw_tol=8.0, trace_obj=True)
    out['carry_log'] = k['carry_log']
    out['phases'].append(['carry', k['carry_log'][-1], k['obj_after_carry'],
                          k['z_up_after_carry']])
    out['carry_recovery'] = k['carry_recovery']
    out['phases'].append(['carry_end', k['base_end'], k['yerr'],
                          k['obj_end']])

    l1 = api.skill('level_upright', joints=(15,), step=0.2, target=0.97,
                   min_gain=0.002, max_tries=3, dur=0.5, reverse=1.0,
                   sign_reversal=False, read_mode='nudge',
                   target_stop='gt', out='single')
    out['zup_carry'] = l1['zup_before']
    out['zup_leveled'] = l1['zup_after']

    l2 = api.skill('level_upright', joints=(14, 15), step=0.08,
                   target=0.996, min_gain=0.001, max_tries=4, dur=0.35,
                   reverse=2.0, sign_reversal=True, read_mode='geom',
                   target_stop='ge', out='two')
    out['level2'] = l2['log']
    out['zup_level2'] = l2['zup_after']

    s = api.skill('seat_gripped_object_on_support', step=0.008, max_steps=8,
                  min_support=3, z_up_min=0.995, tilt_abort=0.94, dur=0.45,
                  settle_dur=0.2, side='right')
    out['lower_log'] = s['lower_log']
    out['table_contacts_before_release'] = s['table_contacts']
    out['obj_before_release'] = s['obj']
    out['zup_before_release'] = s['z_up']
    out['maxforce_before_release'] = s['maxforce']

    rel = api.skill('release_grip_and_settle_in_place', open_angle=0.02,
                    open_dur=0.6, settle_dur=1.0)
    out['seat'] = rel['seat']

    api.skill('retreat_hands_radially_from_object', right_clear=0.26,
              left_clear=0.22, step=0.015, max_iter=14, move_dur=0.45,
              hold_dur=0.35, abort_z_up=0.99, abort_dz=0.004,
              abort_dxy=0.05, right='right', left='left')

    d = api.skill('hold_parked_arms_and_measure_drift', hold=hold)
    out['settle_drift'] = d['drift']
    out['final'] = d['final']

    return out
