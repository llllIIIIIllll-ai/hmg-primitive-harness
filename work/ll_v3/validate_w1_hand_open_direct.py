#!/usr/bin/env python3
"""Bounded direct-grasp setup for hand_open on 05_drill_lift seed 0.

The delegated sequence fails at arm_pose_ik (no bilateral contact).  This
script makes ONE attempt at a direct two-pad side grasp using the documented
per-side R_body fix (the two G1 hands share the same world palm normal at
reset, so a two-sided pad grasp needs opposite body-frame rotations:
left 180 deg about body x, right identity).  It then runs the substrate
hand_close_contact / lift / hand_open unchanged, and records truthful
contacts via contact_detail plus the hand joints.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
RAW = os.path.join(HERE, "reports", "w1_hand_open_direct_raw")
TASK = "05_drill_lift__4"
QTOOL = os.path.join(ROOT, "scripts", "qtool.sh")

HELPER = '''def run(api, hold_upper=None, dur=0.2, **kw):
    if hold_upper is not None:
        api.step_goal(upper=hold_upper, dur=dur)
    cd = api.contact_detail(limit=200)
    hs = api.hand_contacts_side()
    st = api.observe()
    ob = st.get('object') or {}
    a = api.object_aabb()
    b = api.base_state()
    jr = api.joint_ranges()
    hand = {}
    for nm in jr:
        if '_hand_' in nm:
            hand[nm] = jr[nm]['q']
    return {'contacts_detail': cd, 'hand_contacts_side': hs,
            'object': ob.get('pos'), 'aabb': a, 'base': b,
            'hand_joints': hand, 't': api.t}
'''

R_BODY = {"left": [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
          "right": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}


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


def probe(name, kwargs=None):
    r = qtool({"tool": "call", "name": "validator_probe",
               "kwargs": kwargs or {}})
    save(name, r)
    return r.get("result") or {}


def main():
    for name in ("base_standoff_servo", "arm_pose_ik",
                 "hand_close_contact", "hand_open"):
        src = open(os.path.join(SUB, name + ".py"), encoding="utf-8").read()
        w = qtool({"tool": "write", "name": name, "source": src})
        save("register_" + name, w)
    w = qtool({"tool": "write", "name": "validator_probe", "source": HELPER})
    save("register_validator_probe", w)

    print("reset", qtool({"tool": "reset", "seed": 0}).get("ok"))
    r = qtool({"tool": "call", "name": "base_standoff_servo",
               "kwargs": {"standoff": 0.15}})
    save("base_standoff_servo", r)
    p0 = probe("probe_after_standoff")
    obj = p0.get("object")
    a = p0.get("aabb") or {}
    lo = a.get("lo")
    hi = a.get("hi")
    print("object", obj, "aabb lo", lo, "hi", hi)
    if obj is None or lo is None or hi is None:
        print("missing object/aabb")
        return 1

    # two-pad side grasp: pads just outside the +y and -y faces, x at the
    # object x-centre, z at 0.94 m (the delegated grasp height).
    cx = 0.5 * (lo[0] + hi[0])
    cy = 0.5 * (lo[1] + hi[1])
    hy = 0.5 * (hi[1] - lo[1])
    z = 0.94
    targets = {"left": [cx, cy + hy + 0.015, z],
               "right": [cx, cy - hy - 0.015, z]}
    save("direct_targets", targets)
    print("direct targets", targets, "aabb_hy", hy)

    r = qtool({"tool": "qp_reach",
               "kwargs": {"task_dim": 12, "targets": targets,
                          "R_body": R_BODY, "dur": 0.6, "iters": 80}})
    save("qp_reach_direct_grasp", r)
    g = r.get("result") or {}
    print("direct qp_reach ok", r.get("ok"), "ik_err", g.get("ik_err"),
          "pad", g.get("pad"))
    p1 = probe("probe_after_direct_reach")
    print("pre-close hand_contacts_side", p1.get("hand_contacts_side"))

    r = qtool({"tool": "call", "name": "hand_close_contact", "kwargs": {}})
    save("hand_close_contact", r)
    c = r.get("result") or {}
    closed_upper = c.get("closed_upper")
    print("close inner_ok", c.get("ok"), "contacts", c.get("contacts"),
          "bilateral", c.get("bilateral_at_close"),
          "dist", c.get("object_disturbance"),
          "binding", c.get("binding_constraint"))
    p2 = probe("probe_after_close")
    print("post-close hand_contacts_side", p2.get("hand_contacts_side"),
          "truthful n", len(p2.get("contacts_detail") or []))

    lift_targets = {}
    for side in targets:
        t = targets[side]
        lift_targets[side] = [t[0], t[1], t[2] + 0.10]
    save("direct_lift_targets", lift_targets)
    r = qtool({"tool": "qp_reach",
               "kwargs": {"task_dim": 12, "targets": lift_targets,
                          "R_body": R_BODY, "hand_hold": closed_upper,
                          "dur": 0.35, "iters": 80}})
    save("qp_reach_lift", r)
    lf = r.get("result") or {}
    p3 = probe("probe_after_lift", {"hold_upper": lf.get("upper_target"),
                                    "dur": 0.2})
    print("after lift hand_contacts_side", p3.get("hand_contacts_side"),
          "object", p3.get("object"),
          "truthful n", len(p3.get("contacts_detail") or []))

    r = qtool({"tool": "call", "name": "hand_open", "kwargs": {}})
    save("hand_open", r)
    op = r.get("result") or {}
    print("hand_open inner_ok", op.get("ok"), "before", op.get("contacts_before"),
          "after", op.get("contacts_after"),
          "dist", op.get("object_disturbance"),
          "open_err", op.get("open_joint_error"),
          "binding", op.get("binding_constraint"))
    p4 = probe("probe_after_open")
    print("after open hand_joints", p4.get("hand_joints"),
          "hand_contacts_side", p4.get("hand_contacts_side"),
          "truthful n", len(p4.get("contacts_detail") or []))
    return 0


if __name__ == "__main__":
    sys.exit(main())
