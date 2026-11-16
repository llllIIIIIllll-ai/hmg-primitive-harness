#!/usr/bin/env python3
"""Contract test: reach_box_qp(apply=False) must not change qpos/qvel/t and
must return upper_target, joint_margin_joint, joint_margins, wrist_solved.
apply=True must ramp exactly dur."""
import hashlib
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SUB = os.path.join(HERE, "substrate")
TASK = "05_drill_lift"


def qtool(req, timeout=300):
    env = os.environ.copy()
    env["HMG_TASK"] = TASK
    p = subprocess.run(["./scripts/qtool.sh", json.dumps(req), str(timeout)],
                       cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout + 30)
    if p.returncode != 0:
        raise RuntimeError(p.stdout + p.stderr)
    return json.loads(p.stdout)


def h(x):
    return hashlib.sha256(json.dumps(x).encode()).hexdigest()


def state():
    r = qtool({"tool": "call", "name": "state_probe", "kwargs": {}})
    return (r.get("result") or {})


def main():
    for name, f in (("contract_probe", "contract_probe.py"),
                    ("state_probe", "state_probe.py")):
        qtool({"tool": "write", "name": name,
               "source": open(os.path.join(SUB, f)).read()})
    qtool({"tool": "reset", "seed": 0})
    s0 = state()
    h0 = (h(s0["qpos"]), h(s0["qvel"]), s0["t"])
    r0 = qtool({"tool": "call", "name": "contract_probe",
                "kwargs": {"apply": False}}).get("result") or {}
    s1 = state()
    h1 = (h(s1["qpos"]), h(s1["qvel"]), s1["t"])
    qp = r0.get("qp") or {}
    print("READ-ONLY: qpos same", h0[0] == h1[0], "qvel same",
          h0[1] == h1[1], "t same", h0[2] == h1[2])
    for k in ("upper_target", "joint_margin_joint", "joint_margins",
              "wrist_solved"):
        print("  field", k, "present", k in qp,
              "nonnull", qp.get(k) is not None)
    print("  applied flag", qp.get("applied"))
    r1 = qtool({"tool": "call", "name": "contract_probe",
                "kwargs": {"apply": True, "dur": 0.06}}).get("result") or {}
    s2 = state()
    h2 = (h(s2["qpos"]), h(s2["qvel"]), s2["t"])
    print("APPLY: qpos changed", h1[0] != h2[0], "dt", round(s2["t"] - s1["t"], 6),
          "(expected 0.06)")
    print("  qp applied flag", (r1.get("qp") or {}).get("applied"))


if __name__ == "__main__":
    main()
