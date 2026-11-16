#!/usr/bin/env python3
"""Second bounded direct-grasp attempt, correcting the pad-site offset.

Run 1 measured pad_geom - ik_site ~= [0.071,-0.035,0.019] (left) and
[0.077,-0.034,0.015] (right) with the per-side R_body orientation.  This run
asks for pad *faces* just outside the drill's +y/-y faces at the object
x-centre and z=0.94 by subtracting those offsets from the site targets.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
RAW = os.path.join(HERE, "reports", "w1_hand_open_direct2_raw")
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
# pad_geom - site, measured on the first direct attempt with this R_body
DELTA = {"left": [0.07115, -0.0349, 0.0186],
         "right": [0.07705, -0.0336, 0.0146]}


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


def short_contacts(cd, limit=8):
    out = []
    for c in (cd or [])[:limit]:
        out.append([c.get('g1'), c.get('g2'), c.get('dist')])
    return out


def main():
    for name in ("base_standoff_servo", "arm_pose_ik",
                 "hand_close_contact", "hand_open"):
        src = open(os.path.join(SUB, name + ".py"), encoding="utf-8").read()
        qtool({"tool": "write", "name": name, "source": src})
    qtool({"tool": "write", "name": "validator_probe", "source": HELPER})

    print("reset", qtool({"tool": "reset", "seed": 0}).get("ok"))
    r = qtool({"tool": "call", "name": "base_standoff_servo",
               "kwargs": {"standoff": 0.15}})
    save("base_standoff_servo", r)
    p0 = probe("probe_after_standoff")
    obj = p0.get("object")
    a = p0.get("aabb") or {}
    lo, hi = a.get("lo"), a.get("hi")
    print("object", obj, "aabb", lo, hi)

    cx = 0.5 * (lo[0] + hi[0])
    z = 0.94
    pad_des = {"left": [cx, hi[1] + 0.005, z],
               "right": [cx, lo[1] - 0.005, z]}
    targets = {s: [pad_des[s][0] - DELTA[s][0],
                   pad_des[s][1] - DELTA[s][1],
                   pad_des[s][2] - DELTA[s][2]] for s in pad_des}
    save("direct_targets_site", targets)
    save("direct_targets_pad_desired", pad_des)
    print("site targets", targets)
    print("desired pad", pad_des)

    r = qtool({"tool": "qp_reach",
               "kwargs": {"task_dim": 12, "targets": targets,
                          "R_body": R_BODY, "dur": 0.6, "iters": 80}})
    save("qp_reach_direct_grasp", r)
    g = r.get("result") or {}
    print("direct qp_reach ok", r.get("ok"), "ik_err", g.get("ik_err"),
          "pad", g.get("pad"))
    p1 = probe("probe_after_direct_reach")
    print("pre-close contacts", p1.get("hand_contacts_side"))

    r = qtool({"tool": "call", "name": "hand_close_contact", "kwargs": {}})
    save("hand_close_contact", r)
    c = r.get("result") or {}
    closed_upper = c.get("closed_upper")
    print("close inner_ok", c.get("ok"), "contacts", c.get("contacts"),
          "bilateral", c.get("bilateral_at_close"),
          "dist", c.get("object_disturbance"),
          "binding", c.get("binding_constraint"))
    p2 = probe("probe_after_close")
    print("post-close contacts", p2.get("hand_contacts_side"),
          "truthful", short_contacts(p2.get("contacts_detail")))

    lift_targets = {s: [targets[s][0], targets[s][1], targets[s][2] + 0.10]
                    for s in targets}
    save("direct_lift_targets_site", lift_targets)
    r = qtool({"tool": "qp_reach",
               "kwargs": {"task_dim": 12, "targets": lift_targets,
                          "R_body": R_BODY, "hand_hold": closed_upper,
                          "dur": 0.35, "iters": 80}})
    save("qp_reach_lift", r)
    lf = r.get("result") or {}
    p3 = probe("probe_after_lift", {"hold_upper": lf.get("upper_target"),
                                    "dur": 0.2})
    print("after lift contacts", p3.get("hand_contacts_side"),
          "object", p3.get("object"),
          "truthful", short_contacts(p3.get("contacts_detail")))

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
          "contacts", p4.get("hand_contacts_side"),
          "truthful", short_contacts(p4.get("contacts_detail")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
