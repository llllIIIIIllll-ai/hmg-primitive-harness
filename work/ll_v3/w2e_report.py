#!/usr/bin/env python3
"""Summarise a W2e run: phase displacements, path margin, orientation, steps."""
import json
import sys

fn = sys.argv[1] if len(sys.argv) > 1 else (
    "work/ll_v3/reports/w1_raw/w2e_offset_+0.00.json")
res = (json.load(open(fn)) or {}).get("result") or {}
print("== %s" % fn)
print("ok", res.get("ok"), "stage", res.get("stage"),
      "binding", res.get("binding_constraint"), "err", res.get("error"))
ph = res.get("phase") or {}
print("object poses:")
for k in ("initial", "after_tuck", "after_standoff", "after_untuck",
          "after_rest_standoff", "after_reach", "after_close", "after_lift"):
    if k in ph:
        print("  %-20s %s" % (k, ph[k]))
print("displacements", json.dumps(ph.get("displacements")))


def path_min(trace):
    best = None
    for e in trace or []:
        m = e.get("margin")
        if m is None:
            continue
        if best is None or m < best[0]:
            best = (m, e.get("margin_joint"), e.get("k"))
    return best


for label in ("unfold", "untuck"):
    u = res.get(label) or {}
    if not u:
        continue
    print("%s: ok %s worst %s steps %s resets %s nans %s sim_t %s" % (
        label, u.get("ok"), u.get("worst_pad_error"), u.get("steps"),
        u.get("resets"), u.get("nans"), u.get("sim_time")))
    fs = u.get("first_step") or {}
    print("  dq_need %s steps_est %s ik_err %s" % (
        fs.get("dq_need"), fs.get("steps_est"), fs.get("ik_err")))
    print("  normal_error_deg %s" % (u.get("normal_error_deg"),))
    pm = path_min(u.get("trace"))
    print("  path margin min %s (joint %s at k %s)" % pm)

r2 = res.get("reach") or {}
if r2:
    fs = r2.get("first_step") or {}
    print("reach: ok %s worst %s steps %s resets %s nans %s sim_t %s" % (
        r2.get("ok"), r2.get("worst_pad_error"), r2.get("steps"),
        r2.get("resets"), r2.get("nans"), r2.get("sim_time")))
    print("  dq_need %s steps_est %s ik_err %s margin %s joint %s" % (
        fs.get("dq_need"), fs.get("steps_est"), fs.get("ik_err"),
        r2.get("joint_margin_min"), r2.get("joint_margin_joint")))
    pm = path_min(r2.get("trace"))
    print("  path margin min %s (joint %s at k %s)" % pm)
    print("  site", r2.get("pad_error"), "normal", r2.get("normal_error_deg"),
          "base_drift", r2.get("base_drift"))
    print("  joint_margins", json.dumps(r2.get("joint_margins")))
rs = res.get("rest_standoff") or res.get("standoff_after_untuck") or {}
if rs:
    print("rest_standoff ok %s aabb %s lateral %s base %s bursts %s" % (
        rs.get("ok"), rs.get("standoff_aabb"), rs.get("lateral_error"),
        rs.get("base"), rs.get("bursts")))
print("contacts", res.get("contacts"), "rise", res.get("object_rise"),
      "guard", (res.get("guard") or {}).get("ok"),
      "pad_error", res.get("pad_error"),
      "normal", res.get("normal_error_deg"))
print("budget", json.dumps(res.get("budget"))[:1200])
