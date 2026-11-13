"""Skill: pre-shaped palm approach to an object-relative standoff.

Mechanism
  Measure the object once, aim the palm at a point held `back` metres short of
  the object along one horizontal axis and `lateral` metres across, at a fixed
  height, with orientation unchanged.  Approach is therefore a straight,
  collision-free line that ends BEFORE the grasp column, so the fingers - not
  the palm - make first contact when they close.

Failure signature it removes
  A palm move that aims at the object itself reaches the grasp column and
  shoves the object (drill handle pushes the body over, object slides out of
  the workspace).  Standing off lets the curling fingers sweep onto the handle
  and take the object with them, so the object pose after the approach is
  essentially unchanged from before.

Contract
  signature: run(api, side='right', back=0.22, lateral=-0.046, z=0.86,
                 dur=1.2, orientation='keep')
  preconditions: base at the stance the offsets were measured in (for the
      drill: yaw 0, ~0.5 m in front of the object table); the object pose is
      observable through api.observe().
  postconditions: the `side` palm sits at
      (obj_x - back, obj_y + lateral, z) in the world frame, orientation
      held; residual palm error reported.
  side effects: one api.observe(), one api.reach_box(dur), and exactly two
      object api.geom() reads afterwards (object position, then its
      uprightness z_up).  Those reads perturb the solver warmstart and are
      part of the measured behaviour.
  returns: {'target', 'residual', 'obj0', 'obj_after', 'z_up_after'}

Scope: reach-and-grasp tasks with a known approach axis and a standoff the
fingers can cross.  Breaks if the object is not roughly axis-aligned in the
base frame (offsets are world-frame constants), if another body occupies the
standoff, or if the object is taller than `z` permits the palm to sit above
its handle.
"""


def run(api, side='right', back=0.22, lateral=-0.046, z=0.86, dur=1.2,
        orientation='keep', **kw):
    ob0 = api.observe()['object']
    obj0 = np.asarray(ob0['pos'], np.float64)
    tgt = [(obj0[0] - back).item(), (obj0[1] + lateral).item(), z]
    r = api.reach_box(targets={side: tgt}, orientation=orientation,
                      sides=(side,), dur=dur)
    out = {'target': tgt, 'residual': r['palm_residual'], 'obj0': ob0['pos']}
    out['obj_after'] = api.geom()['object']['pos']
    out['z_up_after'] = api.geom()['object']['z_up']
    return out
