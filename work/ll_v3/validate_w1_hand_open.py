#!/usr/bin/env python3
"""Independent W1 validator for hand_open on 05_drill_lift seed 0.

Runs the delegated sequence on queue 05_drill_lift__4 and records raw
responses.  Read-only helper primitive measures truthful contacts via
api.contact_detail (the drill's collision geoms sit on child bodies, so
hand_contacts_side can read 0).
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
RAW = os.path.join(HERE, "reports", "w1_hand_open_raw2")
TASK = "05_drill_lift__4"
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")

HELPER = '''def run(api, hold_upper=None, dur=0.2, **kw):
    if hold_upper is not None:
        api.step_goal(upper=hold_upper, dur=dur)
    cd = api.contact_detail(limit=200)
    hs = api.hand_contacts_side()
    st = api.observe()
    ob = st.get('object') or {}
    b = api.base_state()
    jr = api.joint_ranges()
    hand = {}
    for nm in jr:
        if '_hand_' in nm:
            hand[nm] = jr[nm]['q']
    return {'contacts_detail': cd, 'hand_contacts_side': hs,
            'object': ob.get('pos'), 'base': b, 'hand_joints': hand,
            't': api.t}
'''


def qtool(req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run([QTOOL, json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def save(name, obj):
    os.makedirs(RAW, exist_ok=True)
    with open(os.path.join(RAW, name + ".json"), "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1)


def main():
    # 1. register the four substrate sources, names matching the files
    for name in ("base_standoff_servo", "arm_pose_ik",
                 "hand_close_contact", "hand_open"):
        src = open(os.path.join(SUB, name + ".py"), encoding="utf-8").read()
        w = qtool({"tool": "write", "name": name, "source": src})
        save("register_" + name, w)
        print("registered", name, "ok", w.get("ok"))
    w = qtool({"tool": "write", "name": "validator_probe", "source": HELPER})
    save("register_validator_probe", w)
    print("registered validator_probe ok", w.get("ok"))

    # 2. reset seed 0
    r = qtool({"tool": "reset", "seed": 0})
    save("reset", r)
    print("reset", r)

    # 3. standoff
    r = qtool({"tool": "call", "name": "base_standoff_servo",
               "kwargs": {"standoff": 0.15}})
    save("base_standoff_servo", r)
    print("standoff ok", r.get("ok"), "inner", (r.get("result") or {}).get("ok"),
          "binding", (r.get("result") or {}).get("binding_constraint"))

    # 4. object pose -> grasp targets
    p = qtool({"tool": "call", "name": "validator_probe", "kwargs": {}})
    save("probe_after_standoff", p)
    obj = (p.get("result") or {}).get("object")
    print("object after standoff", obj)
    if obj is None:
        print("NO OBJECT POSE")
        return 1
    objx, objy = obj[0], obj[1]
    targets = {"left": [objx - 0.0838, objy + 0.07, 0.94],
               "right": [objx - 0.0838, objy - 0.07, 0.94]}
    save("grasp_targets", targets)
    print("grasp targets", targets)

    # 5. arm pose IK
    r = qtool({"tool": "call", "name": "arm_pose_ik",
               "kwargs": {"targets": targets}})
    save("arm_pose_ik", r)
    ik = r.get("result") or {}
    print("arm_pose_ik ok", r.get("ok"), "inner", ik.get("ok"),
          "worst_pad", ik.get("worst_pad_error"),
          "binding", ik.get("binding_constraint"))

    # 6. contact-triggered close
    r = qtool({"tool": "call", "name": "hand_close_contact", "kwargs": {}})
    save("hand_close_contact", r)
    close = r.get("result") or {}
    print("close ok", r.get("ok"), "inner", close.get("ok"),
          "contacts", close.get("contacts"),
          "bilateral_at_close", close.get("bilateral_at_close"),
          "binding", close.get("binding_constraint"))
    closed_upper = close.get("closed_upper")

    # 7. lift via server qp_reach using the arm_pose_ik target
    ik_target = ik.get("target") or targets
    lift_targets = {}
    for side in ik_target:
        t = ik_target[side]
        lift_targets[side] = [t[0], t[1], t[2] + 0.10]
    save("lift_targets", lift_targets)
    r = qtool({"tool": "qp_reach",
               "kwargs": {"task_dim": 12, "targets": lift_targets,
                          "orientation": "keep", "hand_hold": closed_upper,
                          "dur": 0.35, "iters": 80}})
    save("qp_reach_lift", r)
    lift = r.get("result") or {}
    print("qp_reach lift ok", r.get("ok"), "solver", lift.get("solver"),
          "joint_margin", lift.get("joint_margin_min"))
    upper = lift.get("upper_target")

    # 8. hold 0.2 s and measure truthful contacts after the lift
    r = qtool({"tool": "call", "name": "validator_probe",
               "kwargs": {"hold_upper": upper, "dur": 0.2}})
    save("probe_after_lift", r)
    after_lift = r.get("result") or {}
    print("after lift contacts_detail n",
          len(after_lift.get("contacts_detail") or []),
          "hand_contacts_side", after_lift.get("hand_contacts_side"),
          "object", after_lift.get("object"))

    # 9. open
    r = qtool({"tool": "call", "name": "hand_open", "kwargs": {}})
    save("hand_open", r)
    op = r.get("result") or {}
    print("hand_open ok", r.get("ok"), "inner", op.get("ok"),
          "before", op.get("contacts_before"), "after", op.get("contacts_after"),
          "disturbance", op.get("object_disturbance"),
          "open_joint_error", op.get("open_joint_error"),
          "binding", op.get("binding_constraint"))

    # 10. truthful contacts after open
    r = qtool({"tool": "call", "name": "validator_probe", "kwargs": {}})
    save("probe_after_open", r)
    after_open = r.get("result") or {}
    print("after open contacts_detail n",
          len(after_open.get("contacts_detail") or []),
          "hand_contacts_side", after_open.get("hand_contacts_side"),
          "object", after_open.get("object"))
    hj = after_open.get("hand_joints") or {}
    mx = 0.0
    mxnm = None
    for nm in hj:
        q = hj[nm]
        if q < 0:
            q = -q
        if q > mx:
            mx = q
            mxnm = nm
    print("after open max_abs_hand_joint", mx, "at", mxnm)
    print("after open hand_joints", json.dumps(hj))
    return 0


if __name__ == "__main__":
    sys.exit(main())
