"""Pinned whole-scene probe (sensing skill).

MECHANISM NAME
    measure_scene_state -- one pinned, ordered multi-read of the scene: the
    named SceneObject poses, the task primary object's orientation, the hand-
    object contact count and the base pose.

FAILURE SIGNATURE IT ADDRESSES
    A reader that differs in WHICH engine reads it performs (or in their
    order) changes the trajectory: every ``scene_objects`` and ``geom`` call
    runs ``mj_forward`` on the live data, which refreshes derived state and
    the solver warm start.  Measured elsewhere in this harness: two copies of
    one primitive that differ only in whether diagnostic read blocks are
    present give bit-identical 31-D commands through step 280 but a physical
    state that first differs at the first removed read block, exceeds 1e-6
    rad by step 205-260, and flips an engine outcome at identical cmd time.
    A composition that re-reads the scene a different number of times than
    the promoted program therefore is NOT behaviour preserving even when
    every command is identical.

THE MECHANISM (what it does physically)
    Read-only against the model: it issues, in this exact order,
    ``scene_objects()`` (mj_forward + body poses), ``geom()`` (mj_forward +
    object pose/tilt), ``observe()`` (hand sites + contact count) and
    ``base_state()``.  It commands nothing, so its effect on the robot is
    confined to the warm-start/derived refresh those reads cause.  Because
    the read count is pinned here, any skill or task program that composes
    through this one entry point reproduces the same read pattern by
    construction.

SIGNATURE (named params + defaults)
    run(api, **kwargs)
      leader   str, default 'shelf'   key of the object that will be pushed
                                       (used by callers to index the result)
      (no other parameters: the read set is deliberately fixed)

PRECONDITIONS
    - api is the harness Tools object on a live episode.
    - The named scene objects exist in the env (for 04: shelf, box, target).

POSTCONDITIONS
    - Returns a dict with keys:
        objects      {name: [x, y, z]} for every SceneObject root body
        shelf/box/target  body_pos of those three named objects (or None)
        zup          primary object (task_object) body-frame up alignment
        yaw          primary object yaw in degrees from its body x axis
        contacts     observe()['hand_object_contacts'] (root-body count)
        base         {'pos','yaw_deg','tilt_deg'} of the robot base
    - No command is sent; ``api.t`` is unchanged.

SIDE EFFECTS
    - The engine derives fresh forward kinematics (mj_forward) twice and
      refreshes contact bookkeeping once; this is what makes the read count
      load-bearing for the trajectory (see above).

SCOPE
    Covers any HMG task whose solution is birthed by scene poses of named
    mujoco_objects plus base pose (all push/align tasks: 04, 07, 09).  It
    does NOT generalise to a task whose leader is not a registered
    SceneObject by name, and it deliberately returns the ROOT-body contact
    count, which under-reports objects whose collision geoms sit on child
    bodies (drill and some boxes); use contact_pairs there.
"""


def run(api, **kwargs):
    leader = kwargs.get('leader', 'shelf')
    objs = api.scene_objects()
    where = {}
    for o in objs:
        where[o['name']] = o['body_pos']
    g = api.geom()['object']
    xm = g['xmat']
    s = {'objects': where,
         'shelf': where.get('shelf'),
         'box': where.get('box'),
         'target': where.get('target'),
         'zup': g['z_up'],
         'yaw': np.degrees(np.arctan2(xm[1][0], xm[0][0])),
         'contacts': api.observe()['hand_object_contacts'],
         'base': api.base_state()}
    s['leader'] = where.get(leader)
    return s
