"""Bounded lateral lead with centring and heading hold (steering law).

MECHANISM NAME
    steer_behind_leader -- convert (lateral reference, leader y, base y,
    base yaw) into the two nav commands (lateral velocity y, yaw rate) that
    (a) bias the leader sideways towards a reference line and (b) keep the
    BASE centred behind the leader and facing straight down +x.

FAILURE SIGNATURE IT ADDRESSES
    Two coupled failures, both measured on 04_push_shelf_forward:
    1. A pure reference-y command applied for the whole push walks the BASE
       off the side of the leader: on seed 1 (v4) the base reached y = +0.39
       while the cart stayed at y = +0.006 -- 0.38 m off-centre against a
       +-0.25 m cart half-width -- and only the inner hand still touched, so
       the push degenerated into a one-sided shove.
    2. The walking direction is BODY-frame, so a residual base yaw steers the
       whole push sideways: with no heading term (v1) the base yaw drifted
       -9 to -15 deg over the 3 m push and the cart drifted -0.12 m on
       seed 1.  (Measured nav scale: wz = 0.2 rad/s for 1 s turned the base
       +11.7 deg.)
    A raw reference-y error also asks for more lateral velocity than the
    contact can survive, so the command must be bounded.

THE MECHANISM (what it does physically)
    ey = ref_y - leader_y          (leader error against the reference line)
    eb = base_y - leader_y         (how far the base sits off the leader)
    ex = sign(eb) * max(|eb| - eb_soft, 0)   (dead zone, then pull back)
    y  = clip(ky * ey - kb * ex, +-cy_max)
    w  = clip(-radians(base_yaw_deg) * kyw, +-cw_max)
    The centring term converts a lateral offset into a lateral correction
    that the leader follows with roughly 0.5-0.7 gain (base +0.06 -> cart
    +0.042, seed 2), so this one law both removes the reference-line error
    and bounds how far off the leader's side the body can drift.  The heading
    term is a proportional yaw-rate hold that drives base yaw back to 0.

SIGNATURE (named params + defaults)
    run(api, **kwargs)
      ref_y        float, default None   lateral reference line in world y;
                                         None means "the leader's own current
                                         y" (pure centring, no lateral lead)
      leader_y     float, default 0.0    leader (pushed object) world y
      base_y       float, default 0.0    base world y
      base_yaw_deg float, default 0.0    base yaw in degrees (+ = ccw)
      ky           float, default 1.0    reference-line gain
      kb           float, default 0.8    centring gain
      eb_soft      float, default 0.0    centring dead zone (m)
      cy_max       float, default 0.08   lateral command bound (m/s)
      kyw          float, default 1.0    heading gain
      cw_max       float, default 0.25   yaw-rate bound (rad/s)

PRECONDITIONS
    - Caller has just measured base and leader in the same instant (the
      caller's probe), so base/leader are consistent with the base pose the
      command will be applied from.
    - The intended push direction is world +x and the base is meant to end at
      yaw 0 facing +x (this law holds that heading, it does not plan a new
      one).

POSTCONDITIONS
    - Returns {'y': cy, 'yaw': cw, 'ey': ey, 'eb': eb}; cy and cw are
      numpy float64 already clipped to their bounds.
    - Issues NO api call and NO command: it is a pure law, so composing it
      cannot change the trajectory, only the numbers stage 3,4 command.

SIDE EFFECTS
    - None on the env (no reads, no commands); only the returned numbers.

SCOPE
    Applies to any task that walks a humanoid behind a movable leader and
    wants the leader held on a lateral reference line while the body stays
    behind it: 04 (cart), 07 (box on a cart), 09 (pre-grasp base placement).
    Breaks when (a) the leader is not roughly ahead along body +x (the law
    assumes the push axis is +x and yaw 0 is the goal), (b) the base yaw is
    large enough that +-cw_max cannot recover it before contact (measured
    boundary: more than about 15 deg of drift is not caught in one burst), or
    (c) the leader's lateral response gain is much lower than 0.5 (then the
    lead is ineffective and the base drifts to +-cy_max*bursts).
"""


def run(api, **kwargs):
    ref_y = kwargs.get('ref_y', None)
    leader_y = kwargs.get('leader_y', 0.0)
    base_y = kwargs.get('base_y', 0.0)
    base_yaw_deg = kwargs.get('base_yaw_deg', 0.0)
    ky = kwargs.get('ky', 1.0)
    kb = kwargs.get('kb', 0.8)
    eb_soft = kwargs.get('eb_soft', 0.0)
    cy_max = kwargs.get('cy_max', 0.08)
    kyw = kwargs.get('kyw', 1.0)
    cw_max = kwargs.get('cw_max', 0.25)

    if ref_y is None:
        ref_y = leader_y
    ey = ref_y - leader_y
    eb = base_y - leader_y
    ex = np.sign(eb) * np.maximum(np.abs(eb) - eb_soft, 0.0)
    cy = np.clip(ky * ey - kb * ex, -cy_max, cy_max)
    cw = np.clip(-np.radians(base_yaw_deg) * kyw, -cw_max, cw_max)
    return {'y': cy, 'yaw': cw, 'ey': ey, 'eb': eb}
