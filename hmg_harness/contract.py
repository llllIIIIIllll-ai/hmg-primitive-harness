"""The plan contract the actor MUST satisfy (enforced, not suggested).

35-D WBC goal = 31 upper-body joint targets + 3 navigation + 1 base height.
The LOWER BODY belongs to the WBC balance policy: hip/knee/ankle entries in a
plan are rejected, not silently applied.
"""
from .fullbody import JOINTS, UPPER_BODY_JOINTS

ALLOWED_UPPER = list(UPPER_BODY_JOINTS)          # 31 exact names
LOWER = [j for j in JOINTS if j not in UPPER_BODY_JOINTS]   # 12, WBC-owned
TOP_KEYS = {'task', 'seed', 'ramp_s', 'steps', 'joints', 'reason',
            'navigate_cmd', 'base_height'}

# The WBC policy indexes its 31-D ``target_upper_body_pose`` with the robot
# model's own joint-group order, which is NOT the contract order above:
#     policy : waist(3) + LEFT ARM(7) + LEFT HAND(7) + RIGHT ARM(7) + RIGHT HAND(7)
#     contract: waist(3) + LEFT ARM(7) + RIGHT ARM(7) + LEFT HAND(7) + RIGHT HAND(7)
# (the contract order is JOINTS[12:], and BODY_JOINTS lists BOTH arms before
# the hand block).  Measured on 01_box_lift_floor (2026-11-09, actor_2) with
# the single-channel identification: with the permutation below, contract
# channel 10 (right_shoulder_pitch) moves robot0_right_shoulder_pitch (+0.506
# for +0.5), contract 17-20 move nothing for +0.5 because those left fingers
# have the one-sided range [-1.57, 0] (they close negative: -0.8 gives -0.77),
# contract 21-23 move the left thumb, contract 24-29 the right index/middle/
# thumb.  So PERM_TO_WBC must map contract -> policy as written below.
# NOTE: replacing this list with the contract order (identity permutation)
# silently feeds right-arm targets into the left hand and left-hand targets
# into the right arm -- that was tested and is wrong.
WBC_UPPER = [
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint', 'left_elbow_joint',
    'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'left_hand_index_0_joint', 'left_hand_index_1_joint',
    'left_hand_middle_0_joint', 'left_hand_middle_1_joint',
    'left_hand_thumb_0_joint', 'left_hand_thumb_1_joint',
    'left_hand_thumb_2_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint', 'right_elbow_joint',
    'right_wrist_roll_joint', 'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
    'right_hand_index_0_joint', 'right_hand_index_1_joint',
    'right_hand_middle_0_joint', 'right_hand_middle_1_joint',
    'right_hand_thumb_0_joint', 'right_hand_thumb_1_joint',
    'right_hand_thumb_2_joint',
]
assert sorted(WBC_UPPER) == sorted(ALLOWED_UPPER), 'WBC order covers 31 names'
PERM_TO_WBC = [ALLOWED_UPPER.index(n) for n in WBC_UPPER]


def validate(plan):
    """Return (errors, clean_plan).  Errors are fed back to the LLM verbatim."""
    err = []
    clean = {k: v for k, v in plan.items() if k in TOP_KEYS}
    for k in plan:
        if k not in TOP_KEYS:
            err.append('unknown top-level key %r' % k)
    j = {}
    for name, v in (plan.get('joints') or {}).items():
        n = name if name.endswith('_joint') else name + '_joint'
        if n in ALLOWED_UPPER:
            j[n] = float(v)
        elif n in LOWER:
            err.append('%s is lower-body: the WBC owns it, remove it' % name)
        else:
            err.append('%s does not exist; use one of the 31 listed names' % name)
    clean['joints'] = j
    nav = plan.get('navigate_cmd', [0., 0., 0.])
    if len(nav) != 3:
        err.append('navigate_cmd must be 3 numbers')
        nav = [0., 0., 0.]
    clean['navigate_cmd'] = [float(x) for x in nav]
    clean['base_height'] = float(plan.get('base_height', 0.74))
    clean['ramp_s'] = float(plan.get('ramp_s', 2.0))
    clean['steps'] = int(plan.get('steps', 600))
    return err, clean


def schema_text():
    return ('{"task": str, "seed": int, "ramp_s": float, "steps": int, '
            '"joints": {<one of the 31 allowed names>: float}, '
            '"navigate_cmd": [vx, vy, wz], "base_height": float, '
            '"reason": str}')
