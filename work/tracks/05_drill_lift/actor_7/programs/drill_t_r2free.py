# 05_drill_lift v34 = v33 with ONE change: the legacy `mode=padservo`
# correction (block 3b) is now gated by its own knob `servo_on`, default 0.
# Every other v33 knob/default is untouched, so servo_on=1 is byte-exact v33.
#
# WHY (measured, v33b vs v33c, 13 seeds each, same off except the servo):
#   v33b (servo on, gate=1): pass {0,1,2,3,4,8,9,11} = 8/13; fail
#     {5,6,7,10,12} all end at obj z 0.838 (the drill's resting height).
#     The servo runs on 9 seeds (gated = last pinch left ik_seed == 0, i.e.
#     {1,4,5,6,7,9,10,11,12}) and NEVER converges (converged=0, iters=8).
#     On exactly {5,6,7,10,12} it displaces the drill before the seat:
#     servo.trace[0].obj -> trace[-1].obj, e.g. s5 [1.721,0.002,0.799] ->
#     [1.697,-0.141,0.838], s12 [1.741,-0.017,0.801] -> [1.920,-0.208,0.838].
#     On {1,4,9,11} it is a no-op (|dz| <= 0.006) and those seeds pass.
#   v33c (gate=0, which removes the servo AND the object_high stop):
#     seat_pre obj z is 0.799-0.803 on EVERY seed (no displacement), and
#     11/13 lift: {0,1,2,3,4,5,6,8,9,10,11} end at 0.925-0.965; only s7
#     (flat 0.838 at all four lift steps) and s12 (0.872->0.894->0.953 then
#     DROPS to 0.837 on the last step) do not.
#   So the servo is a measured confound, not a fix: it is the stage that
#   creates the 0.838 fail state on 5/6/7/10/12.  v34 removes only that.
#
# OPEN (v34's own run decides): with the stop restored (gate=1) and the servo
# off, does the pass set stay 11/13, and do 0-3 still pass?  s7 and s12 are
# the remaining targets and they have DIFFERENT signatures (never lifted vs
# lifted-then-dropped), so they are not one bug.
#
# HISTORICAL HEADER (v32) FOLLOWS; its routing claims are still the measured
# record for br/pre_stance, but its "expected pass set" line is superseded.
#
# 05_drill_lift v32 = v31 with ONE change: the post-pinch branch-guard
# correction (br, the registered arm_ramp transport) is now ROUTED BY THE
# PRE-PINCH ARM-BRANCH GATE that v31 already measured.  Every v31 knob/default
# is untouched, so br_gate=0 is exact v31.
# Why, all measured in v31c/v31d on the same hard seeds (seat=branch, ps_on=0,
# tw=0, pin1_ramp=0):
#   (1) The v31 gate |measured left shoulder_yaw| at pre_pinch (contract j5)
#       separates the adjudicated sets EXACTLY, 13/13: fire=0 on the pass set
#       11(0.0668) 3(0.1233) 2(0.3362) 0(0.4488) 1(0.6636) 9(0.7233), fire=1 on
#       the fail set 7(2.5492) 12(2.5583) 8(2.5597) 5(2.5653) 6(2.5710)
#       10(2.5734) 4(2.5771).  Thr 1.0 sits in an empty 0.55 rad band.
#   (2) pre_stance=1 vs 0 on the SAME seeds 4,5,6,7,8,10,12 is a causal flip
#       for 4 and 8 only: pre0 -> all seven end at obj z 0.838 (the drill's
#       resting height on its support) with the object never leaving it; pre1
#       -> s4 0.852->0.889->0.913->0.936 and s8 0.896->0.940, both progressive,
#       palm_res <=0.065, s8 terminal 'object_high'.  That is the passer lift
#       shape (v31b s0-s3 0.812->0.965/0.944/0.939/0.936, palm_res <=0.043).
#   (3) br is still REQUIRED for the fire=0 hard pair 9 and 11 (v31c, br_on=0:
#       s9 and s11 both 0.838), and it destroys s4 (v31b, br_on=1: s4 0.838,
#       seat_pre obj already displaced to 0.838) while leaving s8 unrouted.
# Hence the routing: when the pre-pinch gate fires, the arm was already put on
# the passer branch by pre_stance and the br transport is skipped (and logged);
# when it does not fire, br runs exactly as in v31.  Expected pass set grows
# from the measured baseline {0,1,2,3} (+{9,11} via br) to include 4 and 8.
# 05_drill_lift v31 = v30 with ONE added block (2b3, pre_stance), placed
# between pre_pinch and the two pinch reaches and gated so the passing seeds
# never fire.  Every v30 knob/default outside that block is untouched, so
# pre_stance=0 behaves as v30.  Why, all measured in v30a on seeds 0/2/4/6:
# the left arm is ALREADY on a different IK branch at pre_pinch --
# shoulder_yaw 0.3362-0.4488 on the two passers vs 2.5710-2.5771 on both
# failers -- and that split is created by the far/near warm-up reaches, while
# pre_pinch_base x is identical (1.3719-1.3723).  The +6..+7 cm base drift
# appears only AFTER the pinch reaches and is not closed-loop correctable, so
# the fix has to be upstream: start the pinch reaches from the passer arm.
def run(api, **kw):
    # 05_drill_lift v30 = v29 with THREE changes inside block 3a2 only; every
    # v28 knob/default outside it is untouched, so pin1_ramp=0 is byte-exact
    # v28 (unchanged from v29).  v29b (seeds 0/2/4/6) MEASURED: passers exact
    # no-ops (fire 0, dev_max 0.0000/0.0939, final_obj and stop 4 unchanged);
    # failers put the arm ON the passer branch (arm_vs_passer all <=0.13 rad,
    # wrist j4/j6 0.7647/1.4587 vs 0.7363/1.5082) and moved the pad 11.9 cm,
    # but the pad error grew in -y and grip.held waist went to
    # [-0.4053,...]/[-0.0192,...] where v28a had -0.08.  So the wrist test was
    # CONFOUNDED by the torso.  v30 changes:
    #   (1) pin1_full=1: build the arm_ramp target from the MEASURED 31-vector
    #       and overwrite only [3:10], freezing the waist (see the comment at
    #       the knob).  This is the confound control.
    #   (2) pin1_rebase default 1 -> 0: v29b measured 40/40 iterations, yaw
    #       -1.9 -> -22.66 deg (s6), x never below 1.4209 vs a 1.380 target,
    #       5.6 s wasted.  Harmful; opt-in only.
    #   (3) waist-inclusive logging: pre_upper10 / pre_cmd10 / target_waist /
    #       ramp_upper10 / rb_upper10.  This is what decides whether the
    #       -0.4 rad waist came from the commanded start or from the WBC.
    #
    # 05_drill_lift v29 = v28 with ONE new gated block (3a2, `pin1_ramp`)
    # and its log field `pin1`.  Nothing else is changed; every v28 knob and
    # every v28 default is untouched, so pin1_ramp=0 is byte-exact v28.
    #
    # SUPERSEDES THE v28 HEADER BELOW.  The v28 torso hypothesis is now
    # MEASURED AND REFUTED (v28a, the un-ramped control, seeds 0/2/4/6):
    # grip['held'][0:3] on the FAILERS is [-0.0800,-0.0407,0.0105] /
    # [-0.0489,-0.0569,0.0391] against the PASSERS' [0.0216,-0.1293,0.0525] /
    # [0.0299,-0.1365,0.0495] -- a waist_yaw separation of only 0.06-0.10 rad,
    # not 0.41-0.52 rad.  The -0.41/-0.52 values quoted below were produced by
    # v27d's OWN per-segment ramp plus servo, not by the pinch branch; the
    # waist cannot be the carrier.  Two more channels are likewise measured
    # non-carriers at the pinch: base x (failers +1.6/+3.6 cm at grip,
    # 1.423/1.398 vs 1.387/1.382) and base y (failers' pinch pad_rel y is
    # 0.0889/0.0748 against the commanded dy_pinch=0.080, i.e. within 1.3 cm).
    # base_align's yaw lever was already refuted in v21 (align2 drove the pad
    # to +0.026/+0.013/+0.034 and the grasp still failed).
    #
    # REMAINING SEPARATION THAT SURVIVES (v28a/v27d, identical on both):
    # the second pinch (pinch1) reports ik_rot 0.3204/0.2233 on the failers
    # against 0.0260/0.0402 on the passers, and the returned joint vectors
    # differ by ~1 rad on OPPOSITE SIGNS in wrist_roll (j4) and wrist_yaw (j6):
    #     passers measured  j4 0.7363/0.7315   j6 1.5245/1.5084
    #     failers measured  j4 1.4832/1.8009   j6 0.6185/0.4087
    # j5 saturates at 2.618 on all four (both groups are on the same j5
    # branch), so the branch difference is j0/j3/j4/j6.  The pad geom sits
    # 8.4 cm from the grip site along the hand forward axis, so a ~1 rad wrist
    # error rotates that lever and displaces the pad by ~8-10 cm -- exactly the
    # measured pad_rel x +0.0154/+0.0366 (passers -0.1013/-0.1052, target
    # -0.091) and z +0.2942/0.2477 (passers 0.2619/0.2641, target 0.263)
    # while y is within 1.3 cm.  The GRIP SITE is only 2-6 cm off (palm_res
    # [-0.046,0.008,0.017]/[-0.059,0.010,0.018]), so the arm's end-effector
    # placement is not the miss; the wrist ROTATION is.
    #
    # WHY THE ARM RAMP MUST BE RETESTED.  v26 ramped to br_arm (the passers'
    # COMMANDED mean) and the pad did not move.  That run is confounded twice
    # over: its base was +7.0 cm forward of the passers at that instant
    # (1.457/1.434 vs 1.387/1.382) because the ramp itself kicked the base,
    # and its elbow landed 0.131 rad short (achieved j3 0.9298 vs the passers'
    # measured 1.0611).  Budget, first order, all in pad_rel x:
    #     passers -0.1013
    #     + base  +0.070     -> -0.031
    #     + elbow +0.046     -> +0.015   (measured v26: +0.0148)
    # i.e. the wrist DID land on the passer branch in v26 (achieved j4 0.7279,
    # j6 1.4672 vs measured 0.7363, 1.5084) and the entire residual is
    # explained by the base kick plus the elbow lag.  The clean test -- wrist
    # on the passer branch at a passer-equivalent base -- has never been run.
    #
    # BLOCK 3a2 (`pin1_ramp`, default 0).  Immediately after the two pinch
    # reaches and BEFORE any settle/gate/branch/seat machinery, with all of
    # those switches independent of it:
    #   gate   fire iff max|measured_arm[3:10] - pin1_target| > pin1_gate
    #          (0.15 rad).  Measured: s0 dev 0.000, s2 dev 0.094 (j3) -> the
    #          passers never fire and stay exact no-ops; s4 dev 1.16, s6 dev
    #          1.17 -> the failers fire.
    #   target pin1_target = the PASSERS' MEASURED pinch1 arm
    #          [-2.1222,-0.0534,2.6635,1.0611,0.7363,1.5245,1.5082].  The
    #          measured convention is chosen deliberately: v26 ramped to the
    #          COMMANDED passer mean and its achieved vector undershot the
    #          measured one by 0.131 rad in the elbow, so the commanded vector
    #          is the wrong reference for a residual test expressed on measured
    #          joints.  pin1_lag=1 adds the v26-measured per-joint lag
    #          (achieved - target, j3 +0.111) to the target so the ACHIEVED
    #          vector lands on the measured passer vector; the log keeps both
    #          the target and the achieved vector so the compensation is
    #          auditable and can be turned off (pin1_lag=0).
    #   rebase pin1_rebase=1 re-translates the base to the pose the passers
    #          hold at this instant, x = ox - 0.351, y = oy + 0.006, with the
    #          arm holding its joints (a pure base command, no IK, no new
    #          reach).  ox here is the START object x, and the passers sit at
    #          exactly ox-0.351 from the walk phase, so this reproduces their
    #          measured base geometry.  Without it the ramp's own base kick
    #          re-creates the v26 confound.
    #   The block logs pre_arm/pre_base/pre_pad/pre_pad_rel, the full arm_ramp
    #   record, base_after_ramp/base_drift, the rebase count, and
    #   post_arm/post_base/post_pad/post_pad_rel/post_obj so the funder can
    #   read the pad motion against both the joints and the base separately.
    #
    # v28 header (hypothesis now refuted; kept verbatim for the record):
    #
    # NEW HYPOTHESIS (measured, v27d reach dump + v27a/c24a `grip.held`):
    # the failers' TORSO is twisted.  grip['held'][0:3] (the last COMMANDED
    # 31-D vector, whose waist part is the waist measured when the last reach
    # built its command) is
    #     s4 [-0.4128,-0.0457,0.0193]  s6 [-0.5153,-0.0363,0.0447]   (fail)
    #     s0 [ 0.0216,-0.1293,0.0525]                               (pass)
    # and ALLOWED_UPPER[0] is waist_yaw (hmg_tools WAIST = waist_yaw,
    # waist_roll, waist_pitch).  A -0.41/-0.52 rad waist_yaw is a -24/-30 deg
    # torso yaw; with base yaw -3.7 deg the torso faces ~-30 deg while the
    # passers face ~+2.6 deg (-0.065+ 0.024 rad).  The pinch target is at the
    # edge of the 7-DOF arm's workspace at the live base/waist (v27d pinch1
    # ik_err 0.0576/0.0575, achieved grip site 9.6/6.6 cm +x of the target,
    # palm_res [-0.096,-0.025,-0.034]/[-0.066,-0.002,-0.032]); a 30 deg torso
    # twist easily costs that much reach.  This one mechanism explains every
    # prior negative at once: base_align fixed BASE yaw (v21) but not the
    # waist; arm_ramp fixed ARM joints (v26) but the torso was still twisted,
    # so the same joint vector pointed 30 deg elsewhere; and the Cartesian
    # pad servos could not move the pad because the arm was already pinned at
    # its reachable frontier (v27d: 4 mm of pad motion per 2.2 cm command).
    #
    # LEVER (tw, default 0 = exact v27 behaviour): before the pinch, ramp
    # ALLOWED_UPPER[0] (waist_yaw) to tw_yaw with the registered `arm_ramp`
    # skill, joints=[0].  arm_ramp starts from api.commanded() and leaves the
    # other 30 joints and the nav command untouched, and certifies the move
    # with api.upper().  tw_comp=1 adds -base_yaw to the target so the TORSO
    # faces world-forward instead of the waist joint reading 0.
    # DECIDING MEASUREMENT: read log['tw'] (achieved waist_yaw, max_err,
    # tracked) and the pinch0/pinch1 ik_err + palm_res + pad_rel.  If tw=1
    # leaves ik_err ~0.02-0.06 and pad_rel ~[0.015,0.08,0.29] unchanged, the
    # torso hypothesis is refuted; if the pinch residual collapses and
    # pad_rel moves toward the passer [-0.10,0.075,0.262], it is confirmed.
    # Passers 0/2 must keep final obj z 0.965/0.936, stop_at_lift 4, and
    # fire no branch/servo (both off in this run).
    #
    # --- v27 header below (kept verbatim) ---
    # 05_drill_lift v27 = v26 with the branch repair COMPOSED: after the v26
    # guard fires, optionally (br_on) run the registered `arm_ramp` joint
    # transport, then optionally (ps_on) run the NEW registered skill
    # `servo_pad_translate`, which closes the loop on the MEASURED pad in
    # CARTESIAN terms.  The two knobs separate the two hypotheses on the
    # failers, and both default to 1 so the composition is the default.
    #
    # WHY (measured, v26a seeds 0,2,4,6, seat_mode='branch'): the guard fired
    # only on 4/6 (dev max-axis 0.1184/0.0933 vs passers 0.0033/0.0038) and
    # arm_ramp reached the passers' joint branch (achieved max_err 0.111/
    # 0.0987 rad, tracked 0), but the object-relative pad did NOT arrive at
    # the passer seat: post pad_rel [0.0148,0.0352,0.2468] / [0.0069,0.0413,
    # 0.2450] against the seat [-0.103,0.078,0.263], i.e. still 11.5/11.0 cm
    # outboard in x, 4 cm low in y, 1.6 cm low in z.  The grasp then still
    # failed (final obj z 0.838, stop_at_lift None, all four lift steps
    # identical).  A JOINT vector cannot place a CARTESIAN object-frame pad:
    # the failers' base differs (s25sv_a: x 1.4014/1.3719 vs passers 1.3723/
    # 1.3719, yaw -3.70/-3.74 vs +1.38/+1.61 deg), and the same joints on a
    # base 3-6 cm forward and 5 deg yawed land the pad 10 cm elsewhere.  That
    # is exactly why the catalogued arm_ramp says it "does not fix unseen
    # ik_seed-2 seeds" -- v26 measures that boundary.
    #
    # The new servo's law is  cmd_grip_site = grip_site_live + clip(target_pad
    # - pad_live, cap)  with target_pad = live_obj + [seat_rx,seat_ry,seat_rz],
    # both quantities re-measured every step.  `servo_pad_to_point` (kept, its
    # hash is a measured negative) commanded pad_live + d THROUGH the grip
    # site, so each step added the ~8.4 cm pad-below-site offset and the pad
    # ran away downward (s25sv_a: 20-25 cm, final err 0.2207/0.2543).
    #
    # DECIDING MEASUREMENT: (a) ps_on=1,br_on=1 -- does the measured pad_rel
    # reach the seat window and do seeds 4/6 then lift (stop_at_lift fires,
    # final obj z > 0.92, gripper0 contacts > 0)?  (b) ps_on=1,br_on=0 on the
    # same seeds -- if the servo alone suffices, the joint transport is
    # unnecessary; if it aborts/flips branch, the ramp is required as a
    # precondition.  Passers 0/2 must send zero commands and keep final obj
    # z 0.965/0.936, stop_at_lift 4.
    #
    # --- v26 header below (kept verbatim) ---
    # 05_drill_lift v26 = v25 with ONE addition: seat_mode='branch' in block 3c.
    # It measures the LIVE palm pad relative to the LIVE drill root, and only
    # when that object-relative pad is OUTSIDE a window around the passer
    # reference (seat_rx/ry/rz) does it transport the 7 left-arm joints onto
    # the passers' own held-arm vector with the registered `arm_ramp` skill.
    # On a passer the measured pad is already inside the window, so the guard
    # is false and ZERO commands are sent -- seeds 0/2 must stay at the seat=0
    # numbers (final obj z 0.965/0.936, stop_at_lift 4).
    #
    # WHY (measured, s25sv_a): the passers' held left-arm vectors are
    #   s0 [-2.1222,-0.0534,2.6635,1.0611,0.7363,1.5245,1.5082]
    #   s2 [-2.0953,-0.0536,2.6568,1.1550,0.7315,1.5161,1.5084]
    # i.e. the `br_arm` target of the catalogued arm_ramp skill, and their
    # SECOND pinch is what lands the pad at the seat.  The failers' held arm
    #   s4 [-1.2348,0.3509,2.2787,0.9029,1.6191,1.1610,-0.0372]
    #   s6 [-1.1939,0.2583,2.0970,1.2650,1.7564,1.2403,-0.3601]
    # is a DIFFERENT IK branch (shoulder_yaw 2.28/2.10 vs 2.66; wrist_roll
    # 1.62/1.76 vs 0.73; wrist_yaw -0.04/-0.36 vs 1.51), and their pad_rel is
    # [+0.0154,0.0889,0.2942] / [-0.0097,0.0748,0.2907] against the passers'
    # [-0.1013,0.0747,0.2619] / [-0.1052,0.0818,0.2641]: 9.3-11.8 cm outboard
    # in x -- the site OVERSHOOTS the commanded pinch site along +x -- and
    # 2.8-3.1 cm high, y good.  v25's existing branch block sits behind
    # `gated`, which needs the LAST pinch's ik_seed == 0, but all four measured
    # seeds report ik_seed 2 there, so it never ran.  The discriminating
    # quantity is the measured PAD, not the ik_seed label (the passers' second
    # pinch also reports ik_seed 2 and yet lands in the passing branch).
    #
    # DECIDING MEASUREMENT: the LIVE pad_rel after the transport (target
    # x ~ -0.10, y ~ 0.075, z ~ 0.262), whether seeds 4/6 then lift the drill
    # (stop_at_lift fires, final obj z > 0.92), and the live gripper0 contacts.
    #
    # --- v23 history below (kept verbatim) ---
    # 05_drill_lift v23 = v22 with settle=0 (default) and ONE addition: a
    # READ-ONLY live grip probe at three instants -- pre-close, immediately
    # after close_grip, after lift 0.  It logs the left-hand geom world poses
    # (object-relative), the drill's LIVE body pose + AABB + per-geom boxes,
    # and every sim contact that involves the drill ('bottle') with its
    # penetration (dist<0), contact-frame normal and normal force.  NO command
    # changes anywhere: with settle=0 the motion is v21's byte-for-byte.
    #
    # WHY (measured, s22a = v22 settle=1 seeds 0,2,4,6): the passers' object
    # RISES out of the close (grip z 0.829/0.831 -> lift0 0.854 -> 0.941/0.927)
    # while the failers' object FALLS (0.825/0.815 -> lift0 0.838 and stays
    # pinned there through all four lifts).  The hand command is nearly
    # identical across the groups (s6 [-0.456,-2.207,0.296,-0.26,1.523,1.553,
    # 0.652] vs s0 [-0.443,-2.19,0.246,-0.273,1.537,1.549,0.653]) and
    # hand_contacts_side()['left'] reads 0 on passers AND failers because it
    # counts only object ROOT-body contacts (hmg_tools.py:419), so it cannot
    # see a multi-body drill.  Nothing measured so far reports the LIVE
    # hand/drill contact geometry at the close.  DECIDING MEASUREMENT: at grip
    # and after lift 0, does the drill touch the pad/fingers, with what
    # penetration, normal force and normal direction, and where is it relative
    # to the drill's live AABB -- and does that differ between passers 0/2 and
    # failers 4/6.  Seeds 0/2 must keep final obj z 0.941/0.927, stop 4.
    #
    # --- v22 history ---
    # 05_drill_lift v22 = v21 with ONE added composition step, gated by
    # off settle=1: after the second pinch reach, re-deliver that reach's OWN
    # 31-D `upper_target` through the registered `reach_settle` skill
    # (segmented smoothstep ramps from the measured vector, then a short dwell
    # at the target) and measure the LIVE pad before and after.  With settle=0
    # the composition is v21's byte-for-byte.
    #
    # WHY (measured, m21s = v21 gs=1 seeds 0,2,4,6): reach_box solves its
    # pinch in scratch MjData and writes the solution there; grasp_scratch at
    # that same pre-pinch pose reports the solver's pad at object-relative
    # x -0.064 on BOTH failers, with real left-hand/object contacts (s4
    # thumb_2 -0.0105, index_1 -0.0335, middle_1 -0.0282; s6 thumb_2 -0.0150,
    # index_1 -0.0081).  The LIVE pad after the same reaches sits at
    # +0.0154 (s4) / -0.0097 (s6) against -0.1013 / -0.1052 on the passers, and
    # the two identical pinch calls close only ~2 cm of that gap.  So the
    # target is graspable and the transport is what falls short.  reach_box
    # delivers with one 0.4 s ramp; settle re-delivers the SAME vector in
    # small tracked steps.  DECIDING MEASUREMENT: live pad_rel x after settle
    # (it must leave +0.015 and approach the solver's own -0.064, or the
    # passers' -0.10) and whether 4/6 then reach stop_at_lift; seeds 0/2 must
    # stay at final obj z 0.965/0.936.
    #
    # 05_drill_lift v21 = v20 with (a) align default 0 (the base-yaw trim is
    # DISPROVEN: align2 drove yaw -3.70/-3.45/-3.74 -> -0.52/-0.93/-0.71 deg,
    # well inside the passers' +1.38..+1.61 deg band, and the pad still sat at
    # object-relative x +0.026/+0.013/+0.034 with the object unmoved), (b) a
    # TRUTHFUL hand-object contact count, (c) the commanded-vs-measured left
    # arm vector logged at every pinch reach, and (d) a gated READ-ONLY
    # reach/grasp screen (off gs=1) at the exact pre-pinch state.  With gs=0 it
    # changes NO motion, so it is a pure measurement run whose default
    # composition is v19's.
    #
    # WHY v21 exists (measured, ddef1 defaults, seeds 0-6): reach_box returns
    # ik_err/ik_rot from its SCRATCH solution but pad and palm_residual from
    # the LIVE sim after _ramp, so the two can be compared directly:
    #   pinch0 passers s0/s2/s3 ik_err 0.013/0.016/0.017  ik_rot 0.018/0.033/0.045
    #   pinch0 failers s4/s5/s6 ik_err 0.021/0.008/0.008  ik_rot 0.171/0.131/0.138
    # The failers' solver also puts the site within 2 cm of the target, yet the
    # LIVE palm is 6.6-9.6 cm away (palm_residual x -0.096/-0.068/-0.066 at
    # pinch1) and the live pad sits at object-relative x +0.015/-0.004/-0.010
    # against the passers' -0.101/-0.105/-0.100.  The arm is therefore NOT
    # reaching the command it was given.  ik seed is not the cause (both groups
    # report 0/2) and neither is base x (1.365..1.401 in both groups).  v21
    # measures WHICH joint of the commanded vector the live arm fails to reach,
    # and screens read-only whether any graspable target exists from that pose.
    #
    # The v16 note below is HISTORY (what v16 changed); the composition itself
    # is v11's plus the v16-v20 gated correction.
    #
    # 05_drill_lift v16 = v15 VERBATIM (same walk, far/near reaches, two pinch
    # reaches, measured-height early stop in the four hand_hold lifts) with ONE
    # change: the gated correction is now a CLOSED-LOOP SERVO ON THE ACHIEVED
    # PALM PAD (new skill `pad_servo`, registered separately), instead of
    # another reach at a moved world target.
    #
    # WHY (measured, v11 baseline over seeds 0-3; pad = gripper0_left_g0_col
    # world pose from reach_box, reported object-relative):
    #   pinch1 (the reach that decides the grasp) on the three PASSING seeds
    #     s0 ik_seed 2  pad_rel [-0.1013, 0.0747, 0.2619]
    #     s2 ik_seed 2  pad_rel [-0.1052, 0.0818, 0.2641]
    #     s3 ik_seed 2  pad_rel [-0.0999, 0.1141, 0.2635]
    #   pinch1 on the FAILING seed 1
    #     s1 ik_seed 0  pad_rel [-0.1310, 0.0956, 0.2685]
    # The failing hand ends 2.6-3.1 cm further OUTBOARD (behind) the drill than
    # every passing seed; the palm sits 13.1 cm behind the drill instead of
    # 10.0-10.5 cm, so the ~10 cm finger reach falls short by ~3 cm and the
    # fingers close on air.  Confirmation: on seed 1 the drill is NOT touched at
    # all by pinch+grip (obj [1.764, 0.028, 0.799] unchanged), while every
    # passing seed first shoves the drill 9 cm in y / 2.4 cm up and then catches
    # it -- shoving is normal, not touching is the failure.
    # y and z agree within 2 cm / 1 cm across seeds, so x is the deciding axis,
    # and the absolute pad x does NOT separate them (1.620..1.665 across seeds,
    # seed 1 at 1.633 inside that range) -- the object-relative pose does.
    #
    # Rejected and not repeated: v5 banded site servo 0/4, v6 base servo 2/4,
    # v7 blind world-target correction 2/4, v8 scratch-screened correction 2/4,
    # v9 residual-gated settle 2/4, v10 read-only sweep (instrumentation),
    # v12 single low pinch 3/4, v13 dead gate, v14 five target-offset
    # corrections 3/4 (all re-took ik_seed 0; +2.5 cm target moved the pad only
    # +0.5 cm), v15 clone/gen/cloneaway/genaway 3/4 (clone reached the good
    # BRANCH label ik_seed 2 yet put the pad at rel x +0.012 and failed; gen
    # drifted the base 0.30 m back).
    #
    # Modes (inside the gate unless gate=0 or force=1):
    #   padservo  default: pad_servo(side left, rel p_rel*) then close+lifts
    #   padonly   pad_servo with gain=0 => measure + log only, no commanded
    #             reach (one inventory reading, zero motion; diagnostic)
    #   pc        reach again at the nominal pinch target with
    #             prefer_current=False (the full three-start search)
    #   branch    segmented joint-space transport onto the passers' mean held
    #             arm vector via the registered `arm_ramp` skill, then a
    #             settled pad certification (pad_hold_audit dq=0)
    #   gen/clone/...  v15's ramp modes, kept as references
    #   none      v11 exactly (no correction)
    #
    # v18 = v17 with ONLY the branch arm's inline segmented ramp replaced by
    # api.skill('arm_ramp', ...).  Evidence that this is the fix: the passers'
    # grip['held'] left-arm vectors agree to <5 mrad on j5 (+2.6191) while the
    # failing seed sits at +0.4555; v28 (v17) drove the branch in 10 x 0.30 s
    # segments and reached j5 +2.613, pad settled [-0.038,0.0471,0.25], drift
    # 0.0009, final obj z 0.938.  v29/v30 pass 4/4 with the same move.
    off = kw.get('off') or {}
    dx_far = off.get('dx_far', -0.431)
    dy_far = off.get('dy_far', 0.020)
    dx_near = off.get('dx_near', -0.411)
    dy_near = off.get('dy_near', 0.100)
    dx_pinch = off.get('dx_pinch', -0.091)
    dy_pinch = off.get('dy_pinch', 0.080)
    z_pinch = off.get('z_pinch', 1.14)
    dx_lift = off.get('dx_lift', -0.021)
    dy_lift = off.get('dy_lift', 0.050)
    n_pinch = off.get('n_pinch', 2)
    # v22: segmented re-delivery of the pinch reach's own solved 31-D vector.
    settle = off.get('settle', 0)
    settle_seg = off.get('settle_seg', 8)
    settle_dur = off.get('settle_dur', 0.30)
    settle_tol = off.get('settle_tol', 0.05)
    settle_dwell = off.get('settle_dwell', 2)
    # v24: pad_seat.  Closed-loop GEOMETRIC seating of the pinch hand on the
    # drill's grasp capsule.  Failure signature it addresses (measured, s23a):
    # at the pre-close instant the failers' palm pad sits 9-12 cm PAST the
    # commanded pinch target in object x and 3-3.5 cm too high, while the
    # passers land within 4 mm of a stable object-relative pose.  The skill
    # measures the LIVE pad (hand_geoms) relative to the LIVE drill root
    # (bodies match='bottle') and feeds the residual back into the TARGET,
    # capped per step.  It is a no-op on the passers by construction.
    seat = off.get('seat', 1)
    # v25: seat_mode.  'pad_seat' keeps v24's skill (a 6 cm capped IK
    # re-solve -- measured to flip the IK branch and destabilise the base on
    # s24a/c24a).  'servo' calls servo_pad_to_point: the SAME closed-loop idea
    # but a 2.2 cm per-step cap, so every step's target is solvable from the
    # CURRENT pose (reach_box accepts seed 0) and the branch is held.  The
    # target is the passer reference obj + [rx,ry,rz]; the pad error is what
    # is closed, on the live pad geom.
    # v26: 'branch' measures the live pad vs the same passer reference and,
    # ONLY when the max per-axis deviation exceeds br_guard, transports the
    # left arm to br_arm with the registered arm_ramp skill.  No-op on passers.
    seat_mode = off.get('seat_mode', 'pad_seat')
    sv_tol = off.get('sv_tol', 0.006)
    sv_step = off.get('sv_step', 0.022)
    sv_dur = off.get('sv_dur', 0.30)
    sv_it = off.get('sv_it', 220)
    sv_max = off.get('sv_max', 14)
    sv_rep = off.get('sv_rep', 1)
    seat_rx = off.get('seat_rx', -0.103)
    seat_ry = off.get('seat_ry', 0.078)
    seat_rz = off.get('seat_rz', 0.263)
    seat_it = off.get('seat_it', 4)
    seat_gain = off.get('seat_gain', 1.0)
    seat_cap = off.get('seat_cap', 0.06)
    seat_tol = off.get('seat_tol', 0.015)
    seat_dur = off.get('seat_dur', 0.5)
    seat_dwell = off.get('seat_dwell', 2)
    seat_box = off.get('seat_box', 0.15)
    far_dur = off.get('far_dur', 1.4)
    hold = off.get('hold', 1)
    gate_on = off.get('gate', 1)
    # v34: the legacy padservo correction is now its OWN knob.  v33c measured
    # that `gate=0` removes the pre-seat displacement but ALSO disables the
    # object_high early stop (the same flag gates both), so it was not a clean
    # single-knob test.  servo_on=0 removes ONLY the servo call; gate stays 1
    # so the stop and the read-only gated logging are unchanged from v33b.
    # Default 0 is the measured-better config (v33c: 11/13 lifted vs 8/13);
    # servo_on=1 reproduces v33b exactly.
    servo_on = off.get('servo_on', 0)
    force = off.get('force', 0)
    z_stop = off.get('z_stop', 0.92)
    mode = off.get('mode', 'padservo')
    corr_pc = off.get('corr_pc', 1)
    corr_dur = off.get('corr_dur', 0.4)
    corr_rep = off.get('corr_rep', 1)
    corr_dx = off.get('corr_dx', 0.0)
    corr_dy = off.get('corr_dy', 0.0)
    corr_dz = off.get('corr_dz', 0.0)
    gen_dur = off.get('gen_dur', 0.9)
    pre_x = off.get('pre_x', -0.20)
    pre_y = off.get('pre_y', 0.14)
    pre_z = off.get('pre_z', 1.13)
    pre_dur = off.get('pre_dur', 0.6)
    reanch = off.get('reanch', 0)
    low_dx = off.get('low_dx', 0.0)
    low_dy = off.get('low_dy', -0.030)
    low_dz = off.get('low_dz', -0.050)
    # v20 base_align knobs: close the loop on the base pose `walk_to` left
    # behind (its yaw tolerance is 0.08 rad = 4.58 deg and yaw is never zeroed).
    # Default target = the passers' mean pre-pinch yaw (+1.5 deg); passers sit
    # at +1.38/+1.61/+1.58 so the call is a no-op on them; failing seeds sit at
    # -3.45..-3.74 and get a 5.0-5.2 deg trim.  x/y are left uncontrolled
    # (None) so this is a ONE-axis change; base x does not separate pass/fail.
    # v21: DEFAULT OFF.  Measured: this trim reaches its yaw goal (n=40 capped,
    # final -0.52/-0.93/-0.71 deg) yet every failing seed still misses the
    # pinch, so base yaw is not the mechanism and this call must not be part of
    # the shipped composition.  Kept reachable for reference only.
    align = off.get('align', 0)
    align_yaw = off.get('align_yaw', 1.5)
    align_yawe = off.get('align_yawe', 0.30)
    align_maxp = off.get('align_maxp', 40)
    align_dur = off.get('align_dur', 0.12)
    align_gain = off.get('align_gain', 2.0)
    align_vw = off.get('align_vw', 0.30)
    # v21 read-only screen.  gs=1 calls api.grasp_scratch + api.ik_preview at
    # the exact pre-pinch state.  Both build their own MjData from the live
    # qpos and send NO command, so this cannot move the robot or change the
    # episode; it only reports whether a graspable target exists from the pose
    # the failing seeds actually reach.
    gs_on = off.get('gs', 0)
    # pad_servo knobs: target is OBJECT-RELATIVE (the measured mean of the
    # three passing seeds' pinch1 pad), gain/dur/max_iters/tol close the loop.
    p_relx = off.get('p_relx', -0.102)
    p_rely = off.get('p_rely', 0.090)
    p_relz = off.get('p_relz', 0.263)
    p_gain = off.get('p_gain', 1.8)
    p_dur = off.get('p_dur', 0.3)
    p_max = off.get('p_max', 8)
    p_tol = off.get('p_tol', 0.010)
    p_ornt = off.get('p_ornt', 'down')
    p_side = off.get('p_side', 'left')
    # pad_hold_audit knobs (settle curve, commanded joint step, return, for
    # both dwell anchors 'commanded' and 'measured')
    a_joint = off.get('a_joint', 3)
    a_joints = off.get('a_joints', None)
    a_dq = off.get('a_dq', 0.04)
    a_ramp = off.get('a_ramp', 0.40)
    a_n = off.get('a_n', 6)
    a_dt = off.get('a_dt', 0.20)
    a_target = off.get('a_target', 'commanded')
    # pad_place knobs (joint-space numeric-Jacobian pad servo)
    q_step = off.get('q_step', 0.018)
    q_pdur = off.get('q_pdur', 0.16)
    q_pset = off.get('q_pset', 0.12)
    q_dur = off.get('q_dur', 0.40)
    q_set = off.get('q_set', 0.28)
    q_max = off.get('q_max', 2)
    q_tol = off.get('q_tol', 0.008)
    q_lam = off.get('q_lam', 0.030)
    q_dqmax = off.get('q_dqmax', 0.12)
    q_objtol = off.get('q_objtol', 0.015)
    q_probe = off.get('q_probe', 1)
    # base_place knobs (BASE-space pad placement, closed on averaged settled
    # measurements).  rel = mean settled object-relative pad of the three
    # passing seeds; kx/ky < 1/1.3 because the pad translates ~1.3x the base.
    b_relx = off.get('b_relx', -0.038)
    b_rely = off.get('b_rely', 0.044)
    b_relz = off.get('b_relz', 0.243)
    b_tol = off.get('b_tol', 0.010)
    b_max = off.get('b_max', 5)
    b_step = off.get('b_step', 0.050)
    b_vx = off.get('b_vx', 0.20)
    b_vy = off.get('b_vy', 0.15)
    b_dur = off.get('b_dur', 0.15)
    b_n = off.get('b_n', 16)
    b_skip = off.get('b_skip', 5)
    b_dt = off.get('b_dt', 0.20)
    b_kx = off.get('b_kx', 0.55)
    b_ky = off.get('b_ky', 0.55)
    b_adv = off.get('b_adv', 0.090)
    b_min = off.get('b_min', 0.004)
    b_cstop = off.get('b_cstop', 0)
    # branch knobs (v17): drive the 7 left-arm joints onto the joint vector the
    # three PASSING seeds hold at grip, in SEGMENTED ramps, then certify the
    # resulting pad with pad_hold_audit(dq=0) (settled read).
    # WHY: measured `grip['held']` arm vectors --
    #   s0 [-2.1349,-0.0500, 2.6191, 0.9569, 0.7257, 1.5494, 1.5067]
    #   s2 [-2.0997,-0.0502, 2.6190, 1.0545, 0.7044, 1.5382, 1.5127]
    #   s3 [-2.0631,-0.0530, 2.6191, 1.1111, 0.6937, 1.5439, 1.5042]
    #   s1 [-2.0469, 0.1448, 0.4555, 2.0815,-1.3545, 1.3028, 1.5748]  FAIL
    # j5 is +2.619 on ALL THREE passers and +0.4555 on seed 1: the failing
    # seed sits in a DIFFERENT IK BRANCH, 2.16 rad away in j5 and ~1.4 rad in
    # j6/j7.  That is why every centimetre-scale pad correction failed: it was
    # asked to remove a 2 rad branch difference with millimetre moves.
    br_arm = off.get('br_arm', [-2.0992, -0.0511, 2.6191, 1.0408, 0.7079,
                                1.5438, 1.5079])
    br_seg = off.get('br_seg', 10)
    br_seg_dur = off.get('br_seg_dur', 0.30)
    # v26 guard: max per-axis |pad_rel - passer reference| that still counts as
    # "seated".  Measured separation: passers 0.0033, failers 0.0933/0.1184.
    br_guard = off.get('br_guard', 0.020)
    br_n = off.get('br_n', 26)
    br_dt = off.get('br_dt', 0.20)
    # v27: the two composition switches.  br_on runs arm_ramp (the v26 joint
    # transport); ps_on runs the NEW Cartesian pad servo afterwards.  Both 1
    # = the full composition; ps_on=1,br_on=0 isolates the servo.
    br_on = off.get('br_on', 1)
    # v32 routing switch.  br_gate=1 skips the br transport (and logs
    # br.gated_out=1) when the pre-pinch arm-branch gate |j5| exceeds
    # br_gate_thr, i.e. exactly on the seeds where pre_stance already fired.
    # br_gate=0 is exact v31.  Thr 1.0 is the same empty-band value as the
    # pre_stance gate; measured separation 0.7233 max (fire-set) vs 2.5492 min
    # (no-fire set) on all 13 seeds.
    br_gate = off.get('br_gate', 0)
    br_gate_thr = off.get('br_gate_thr', 1.0)
    # v35 (actor_7 scheme H).  MEASURED-PAD ROUTING of the same v31
    # transport.  br_gate above routes on the |j5| LABEL; this routes on the
    # MEASURED outcome of the pre_stance ramp.  pr_on=0 is byte-exact v34.
    # The box is the axis-aligned hull of the post-pre_stance pad_rel on the
    # seven protected seeds where pre_stance fires (4,5,6,7,8,10,12): the
    # poses measured to pass on that route.  A seed with pre_stance fire=0
    # never reaches this code (the transport already runs, unchanged).  pr_m
    # widens the hull symmetrically; default 0.0 keeps the measured hull.
    pr_on = off.get('pr_on', 0)
    pr_m = off.get('pr_m', 0.0)
    pr_x0 = off.get('pr_x0', -0.0636) - pr_m
    pr_x1 = off.get('pr_x1', 0.0395) + pr_m
    pr_y0 = off.get('pr_y0', 0.0474) - pr_m
    pr_y1 = off.get('pr_y1', 0.0842) + pr_m
    pr_z0 = off.get('pr_z0', 0.2344) - pr_m
    pr_z1 = off.get('pr_z1', 0.2510) + pr_m
    ps_on = off.get('ps_on', 1)
    ps_tol = off.get('ps_tol', 0.012)
    ps_cap = off.get('ps_cap', 0.022)
    ps_dur = off.get('ps_dur', 0.30)
    ps_it = off.get('ps_it', 220)
    ps_max = off.get('ps_max', 14)
    ps_objmove = off.get('ps_objmove', 0.05)
    # v28 torso-square knob (see header).  tw=0 is exact v27 behaviour.
    tw = off.get('tw', 0)
    tw_yaw = off.get('tw_yaw', 0.0)
    tw_comp = off.get('tw_comp', 0)
    tw_seg = off.get('tw_seg', 6)
    tw_seg_dur = off.get('tw_seg_dur', 0.30)
    tw_tol = off.get('tw_tol', 0.06)

    log = {'walk': [], 'reach': [], 'grip': None, 'lift': [], 'hold': hold,
           't': [], 'gate': gate_on, 'z_stop': z_stop, 'mode': mode}

    def stamp(tag):
        log['t'].append({'tag': tag, 't': np.round(api.t, 2).item()})

    st0 = api.observe()
    ob0 = (st0.get('object') or {}).get('pos')
    ox = ob0[0]
    oy = ob0[1]
    log['start_obj'] = ob0
    stamp('start')

    def walk_to(x_t, vx, tol, maxp, y_t=0.03, ytol=0.05, yawe=0.08,
                dur=0.14):
        # v11 verbatim
        i = 0
        while i < maxp:
            s = api.base_state()
            bx = s['pos'][0]
            by = s['pos'][1]
            yaw = s['yaw_deg'] * math.pi / 180.0
            ex = x_t - bx
            ey = y_t - by
            eyaw = -yaw
            if np.abs(ex) < tol and np.abs(ey) < ytol and np.abs(eyaw) < yawe:
                break
            cx = vx if ex > 0 else -vx
            if np.abs(ex) < tol:
                cx = 0.0
            cy = np.clip(1.2 * ey, -0.10, 0.10).item()
            cyaw = np.clip(1.5 * eyaw, -0.30, 0.30).item()
            api.move_base(cx, cy, cyaw, dur)
            i += 1
        s = api.base_state()
        log['walk'].append({'n': i, 'vx': vx, 'tol': tol,
                            'target': np.round(np.array([x_t, y_t]), 4).tolist(),
                            'base': np.round(np.array(s['pos']), 4).tolist(),
                            'yaw': np.round(s['yaw_deg'], 2).item()})
        return i

    def ncon():
        # v21: state()['hand_object_contacts'] reads 0 on EVERY seed measured
        # here, including passers whose 14 cm lift visibly succeeds, so it is
        # not a usable contact signal on this task.  hand_contacts_side()
        # counts contacts whose other body is a hand/gripper body and is the
        # truthful source (see contact_pairs() for the individual pairs).
        hs = api.hand_contacts_side() or {}
        return hs.get('left', -1)

    def pad_live():
        # v22: the LIVE pad geom world pose.  hand_geoms() runs mj_forward on
        # the live data (read-only) and never steps the env.  grasp_scratch
        # CANNOT be used here: it writes its own IK solution into scratch
        # qpos, so the pad it reports is the solver's, not the arm's.
        hg = api.hand_geoms()
        out = None
        for b in hg:
            for gg in hg[b]['geoms']:
                if gg['name'] == 'gripper0_left_g0_col':
                    out = gg['world_pos']
        return out

    def rel3(p, ob):
        return [np.round(p[0] - ob[0], 4).item(),
                np.round(p[1] - ob[1], 4).item(),
                np.round(p[2] - ob[2], 4).item()]

    def gprobe(tag, ob):
        # v23 READ-ONLY.  hand_geoms() and scene_objects() each run
        # mj_forward on the LIVE data (no env step, no command); contact_detail
        # only reads the contact list.  Nothing here changes qpos/qvel, so the
        # episode motion is untouched.
        hg = api.hand_geoms()
        ge = []
        for b in hg:
            if '_left_' not in b:
                continue
            for gg in hg[b]['geoms']:
                ge.append({'n': gg['name'],
                           'rel': rel3(gg['world_pos'], ob),
                           'f': np.round(np.array(gg['xmat'])[:, 0],
                                         3).tolist()})
        so = api.scene_objects(detail=True) or []
        obj = None
        for o in so:
            rb = o.get('root_body') or ''
            if 'bottle' not in rb and 'drill' not in rb:
                continue
            gs = []
            for gg in o.get('geoms') or []:
                gs.append({'n': gg['name'], 'c': gg.get('world_pos'),
                           'lo': gg.get('aabb_lo'), 'hi': gg.get('aabb_hi')})
            obj = {'b': rb, 'p': o.get('body_pos'), 'n': o.get('ngeom'),
                   'm': o.get('body_mass'), 'lo': o.get('aabb_lo'),
                   'hi': o.get('aabb_hi'), 'g': gs}
        cd = api.contact_detail(limit=40, match='bottle') or []
        ct = []
        nb = 0
        for c in cd:
            nb += 1
            fr = c.get('force')
            ct.append({'d': c.get('dist'), 'p': c.get('pos'),
                       'nv': c.get('normal'),
                       'b': (c.get('b1') or '') + '|' + (c.get('b2') or ''),
                       'g': (c.get('g1') or '') + '|' + (c.get('g2') or ''),
                       'f': None if fr is None else fr[0]})
        return {'tag': tag, 'obj': ob, 'hand': ge, 'drill': obj,
                'n_bc': nb, 'c': ct,
                'side': api.hand_contacts_side(),
                't': np.round(api.t, 2).item()}

    def rec(tag, r):
        pad = (r.get('pad') or {}).get('left')
        pl = None if pad is None else pad.get('pos')
        pn = None if pad is None else pad.get('normal')
        ob = ((r.get('state') or {}).get('object') or {}).get('pos')
        # v21: the commanded left-arm vector (7 joints, ALLOWED_UPPER[3:10]) and
        # the MEASURED one at the same instant.  d_arm is the per-joint tracking
        # error; a joint pinned at its model limit shows up as a large d_arm
        # whose sign matches a limit, which is the saturation hypothesis.
        arm = np.round(np.asarray(api.upper())[3:10], 4).tolist()
        tg = []
        if r.get('upper_target') is not None:
            tg = np.round(np.asarray(r['upper_target'])[3:10], 4).tolist()
        dq = None if tg == [] else np.round(
            np.asarray(tg) - np.asarray(arm), 4).tolist()
        return {'tag': tag, 'palm_res': r.get('palm_residual'),
                'ik_err': r.get('ik_err'), 'ik_seed': r.get('ik_seed'),
                'ik_rot': r.get('ik_rot'),
                'obj': ob, 'pad_abs': pl, 'pad_n': pn,
                'pad_rel': None if (pl is None or ob is None)
                else rel3(pl, ob),
                'arm': arm, 'tgt_arm': tg, 'd_arm': dq,
                'ncon': ncon(), 't': np.round(api.t, 2).item()}

    def inv(tag, ob):
        # read-only dump of every geoms of the given side's hand, in the
        # object frame, so the pad/finger geometry vs the drill is visible.
        hg = api.hand_geoms()
        out = {}
        for b in hg:
            if ('_%s_' % p_side) not in b:
                continue
            ents = []
            for gg in hg[b]['geoms']:
                ents.append({'n': gg['name'],
                             'rel': rel3(gg['world_pos'], ob),
                             'nx': np.round(np.array(gg['xmat'])[:, 0], 3).tolist()})
            out[b] = {'body_rel': rel3(hg[b]['body_pos'], ob), 'geoms': ents}
        return {'tag': tag, 'obj': ob, 'bodies': out}

    # 1. far stance, then 'down' preshape (v11 verbatim)
    walk_to(0.95 + (ox - 1.731), 0.25, 0.03, 30)
    r = api.reach_box(targets={'left': [ox + dx_far, oy + dy_far, 1.13],
                               'right': [ox - 0.681, -0.25, 1.15]},
                      orientation='down', dur=far_dur)
    log['reach'].append({'tag': 'far', 'palm_res': r.get('palm_residual'),
                         'ik_err': r.get('ik_err'), 'ik_seed': r.get('ik_seed')})
    stamp('far')
    r = api.reach_box(targets={'left': [ox + dx_near, oy + dy_near, 1.13]},
                      sides=('left',), orientation='down', dur=0.6)
    log['reach'].append({'tag': 'near', 'palm_res': r.get('palm_residual'),
                         'ik_err': r.get('ik_err'), 'ik_seed': r.get('ik_seed')})
    stamp('near')
    # 2. two-stage walk to the verified base placement (v11 verbatim)
    walk_to(ox - 0.431, 0.15, 0.02, 40)
    walk_to(ox - 0.351, 0.15, 0.02, 20)
    # 2b. NEW in v20: trim the base yaw residual left by walk_to (one axis).
    if align:
        log['base_align'] = api.skill('base_align', x_t=None, y_t=None,
                                      yaw_t_deg=align_yaw,
                                      yawe_deg=align_yawe,
                                      maxp=align_maxp, dur=align_dur,
                                      gain=align_gain, vw=align_vw)
    else:
        log['base_align'] = 'off'
    # 2b2. NEW in v28: square the torso (ALLOWED_UPPER[0] = waist_yaw) before
    # the pinch, via the registered arm_ramp skill (joints=[0]).  Off by
    # default; this block is the only behavioural difference from v27.
    if tw:
        bs_tw = api.base_state() or {}
        yt = tw_yaw
        if tw_comp:
            # v29: the program sandbox exposes no Python built-in namespace
            # (measured: NameError 'float' on the first v29 run of seeds
            # 0/2/4/6), so v28's tw lever could never have executed.
            # Identical arithmetic.
            yt = yt - np.pi * np.asarray(
                bs_tw.get('yaw_deg', 0.0)).item() / 180.0
        log['tw'] = api.skill('arm_ramp', side='left', target=[yt],
                              joints=[0], seg=tw_seg, seg_dur=tw_seg_dur,
                              tol=tw_tol)
    else:
        log['tw'] = 'off'
    log['pre_pinch_base'] = api.base_state()
    log['pre_pinch_upper'] = np.round(np.asarray(api.upper()), 4).tolist()
    # 2b3. NEW in v31 (off pre_stance=0 is exact v30).  PRE-PINCH ARM-BRANCH
    # PRE-POSITION.  MEASURED in v30a (seeds 0/2/4/6) on pre_pinch_upper, i.e.
    # BEFORE either pinch reach, the left arm already sits on two IK branches:
    #   passer s0 [-1.8957, 0.4534, 0.4488, 1.8999,-1.3045, 1.4401, 1.5566]
    #   passer s2 [-1.8917, 0.4040, 0.3362, 1.8726,-1.4252, 1.4418, 1.5554]
    #   failer s4 [-2.0129, 0.1314, 2.5771, 1.0242, 0.8078, 1.4764, 1.5435]
    #   failer s6 [-2.0023, 0.1312, 2.5710, 1.0502, 0.7966, 1.4736, 1.5447]
    # Contract index 5 (left shoulder_yaw) is 0.34-0.45 on the passers and
    # 2.57 on BOTH failers (2.13 rad apart); index 7 (wrist_roll) is
    # -1.30/-1.43 vs +0.81/+0.80.  The split is CREATED UPSTREAM in the far /
    # near warm-up reaches: the near solve reports ik_seed 2 with err
    # 0.0020/0.0042 on the passers and ik_seed 0 with err 0.0122/0.0058 on the
    # failers.  The BASE is NOT the pre-pinch cause: pre_pinch_base x is
    # 1.3723/1.3719 (passers) and 1.3719 (s6), identical, with s4 at 1.4014
    # only because its drill sits 1.7 cm forward.  The +6..+7 cm base drift to
    # grip.base 1.442/1.413 appears AFTER the pinch reaches, on the failer
    # branch, and a closed-loop x correction is MEASURED impossible (v27b: the
    # pad servo's ik_err was 0.0004-0.0008 on every step while the achieved
    # grip site crept 1.762 -> 1.769 cm against 2.05 cm backward commands, so
    # the balance response inverts the net motion).
    # Hence the one open lever: START the two pinch reaches from the passer
    # arm.  Gate on |measured shoulder_yaw| > pre_gate_thr, read-only and
    # pre-pinch (measured separation 0.34-0.45 vs 2.57, so 1.0 leaves a
    # 0.55 rad margin).  Target is a full 31-vector built from the MEASURED
    # pose with only [3:10] overwritten, so the waist command stays the
    # measured waist (the v29b confound control) and arm_ramp's
    # START-from-commanded cannot drag the torso.  Passers never fire.
    pre_stance = off.get('pre_stance', 0)
    pre_gate_j = off.get('pre_gate_j', 5)
    pre_gate_thr = off.get('pre_gate_thr', 1.0)
    pre_arm = off.get('pre_arm_vec', [-1.8957, 0.4534, 0.4488, 1.8999,
                                      -1.3045, 1.4401, 1.5566])
    pre_seg = off.get('pre_seg', 10)
    pre_seg_dur = off.get('pre_seg_dur', 0.30)
    pre_tol = off.get('pre_tol', 0.06)
    # v33: this block IS the named skill arm_branch_prestance (gate +
    # full-31 arm_ramp), extracted from the v31/v32 inline code with the same
    # reads and the same commands.  The skill is called even when
    # pre_stance=0 (enable=0), because the gate value is a READ that the br
    # routing below needs on every seed; enable=0 sends no command.
    pre_lg = api.skill('arm_branch_prestance', target=pre_arm,
                       enable=pre_stance, gate_j=pre_gate_j,
                       gate_thr=pre_gate_thr, seg=pre_seg,
                       seg_dur=pre_seg_dur, tol=pre_tol,
                       phase='v33_prestance') or {}
    pre_lg['on'] = pre_stance
    if pre_lg.get('fire'):
        stamp('prestance')
    log['pre_st'] = pre_lg
    # 2c. NEW in v21, read-only screen (off gs=1).  grasp_scratch and
    # ik_preview each copy the LIVE qpos into their own MjData and never send a
    # command, so this cannot move the robot or change the episode -- it only
    # answers: from the pose the failing seeds actually reach, (a) does the
    # commanded pinch target yield real hand-object contacts, and (b) what does
    # the reach map look like object-relative around the pinch?  ik_preview
    # targets are OBJECT-RELATIVE, so the composition's pinch (dx_pinch,
    # dy_pinch, z_pinch) maps to (xo=dx_pinch, gap=dy_pinch, zo=z_pinch-obj_z).
    if gs_on:
        T0 = [ox + dx_pinch, oy + dy_pinch, z_pinch]
        log['jrange'] = api.joint_ranges()
        log['screen'] = {
            't0': np.round(np.asarray(T0), 4).tolist(),
            'base': api.base_state(),
            'grasp': api.grasp_scratch(targets={'left': T0}, sides=('left',),
                                       orientation='down'),
            'preview': api.ik_preview(xs=(-0.16, -0.09, -0.02, 0.05),
                                      gaps=(0.08, 0.11),
                                      zs=(0.30, 0.341, 0.38),
                                      orients=('down',), sides=('left',))}
    # 3. two identical inboard placement reaches (v11 verbatim)
    last_seed = None
    last_tgt = None
    k = 0
    # ---- c6 family B: wrist azimuth at the pinch (R_body override).  ----
    # reach_box REPLACES the whole desired orientation when R_body is given:
    # R_des = Rb @ R_body @ R_site0.  v35's orientation 'down' is exactly
    # R_body = R_Y90, so pinch_rb_deg = 0 passes the SAME matrix and must be
    # bit-identical to the default path (plumbing control).  Non-zero degrees
    # rotate the pad frame about the world z axis at the pinch; hypothesis:
    # seed 7's close roll about world +x is a pad-frame alignment artefact.
    _rb_l = None
    if off.get('pinch_rb'):
        _bdeg = off.get('pinch_rb_deg', 0.0)
        _br = _bdeg * math.pi / 180.0
        _bc = math.cos(_br)
        _bs = math.sin(_br)
        _Rz = np.array([[_bc, -_bs, 0.0], [_bs, _bc, 0.0], [0.0, 0.0, 1.0]])
        _RY90 = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0],
                          [-1.0, 0.0, 0.0]])
        _rb_l = _Rz.dot(_RY90)
        log['pinch_rb'] = {'deg': _bdeg, 'R': np.round(_rb_l, 6).tolist()}
    while k < n_pinch:
        r = api.reach_box(targets={'left': [ox + dx_pinch, oy + dy_pinch,
                                            z_pinch]},
                          sides=('left',), orientation='down', dur=0.4,
                          R_body=_rb_l)
        ent = rec('pinch%d' % k, r)
        log['reach'].append(ent)
        last_seed = (r.get('ik_seed') or {}).get('left')
        last_tgt = r.get('upper_target')
        k += 1
    log['pinch_seed'] = last_seed
    stamp('pinch')
    # 3a2. NEW in v29 (off pin1_ramp=0 = exact v28).  Wrist-branch
    # pre-position on the MEASURED arm, gated so the passers never fire, plus
    # the base re-translation that removes the ramp's own kick.  See the
    # module header for the measured mechanism and the v26 confound budget.
    pin1_ramp = off.get('pin1_ramp', 0)
    pin1_target = off.get('pin1_target',
                          [-2.1222, -0.0534, 2.6635, 1.0611,
                           0.7363, 1.5245, 1.5082])
    pin1_lag_on = off.get('pin1_lag', 1)
    # v26 MEASURED lag = achieved - target, ramping to the commanded passer
    # mean from the failer pinch1 pose: [-0.0611,+0.0167,-0.0013,+0.1110,
    # +0.0200,+0.0037,-0.0150].  lag[j] is added to the target.
    pin1_lag = off.get('pin1_lag_vec',
                       [-0.0611, 0.0167, -0.0013, 0.1110,
                        0.0200, 0.0037, -0.0150])
    pin1_gate = off.get('pin1_gate', 0.15)
    pin1_seg = off.get('pin1_seg', 12)
    pin1_seg_dur = off.get('pin1_seg_dur', 0.30)
    pin1_tol = off.get('pin1_tol', 0.06)
    # v30: rebase default 0.  MEASURED in v29b (seeds 4,6): 40/40 iterations,
    # base yaw -1.9 -> -22.66 deg on s6, x converged only to 1.4229/1.4209
    # against a 1.396/1.380 target (the known +-1.8 cm base-x oscillation makes
    # a 6 mm tolerance unreachable), and it cost 5.6 s.  Opt-in only.
    pin1_rebase = off.get('pin1_rebase', 0)
    pin1_rx = off.get('pin1_rx', 0.351)
    pin1_ry = off.get('pin1_ry', -0.006)
    pin1_rb_tol = off.get('pin1_rb_tol', 0.006)
    pin1_rb_maxp = off.get('pin1_rb_maxp', 40)
    pin1_rb_dur = off.get('pin1_rb_dur', 0.14)
    pin1_rb_v = off.get('pin1_rb_v', 0.10)
    # v30: freeze the WAIST.  arm_ramp takes START = api.commanded(), so a
    # 7-vector target inherits the last COMMANDED waist: if that differs from
    # the measured waist the ramp drags the torso to it.  v29b measured exactly
    # that (grip.held waist -0.4053/-0.0192 vs v28a -0.08), which invalidated
    # the wrist test.  pin1_full=1 builds the ramp target from the MEASURED
    # 31-vector and overwrites only [3:10], so the target waist IS the measured
    # waist and the torso is not a free channel during the test.
    pin1_full = off.get('pin1_full', 1)
    # v33 ROUTING of the pin1 ramp (see the module header).  MEASURED in v32a
    # vs v32b (13 seeds each, otherwise identical off):
    #   pin1_ramp=0 -> s1 0.838 (fail), s4 0.936, s9 0.943, s11 0.927
    #   pin1_ramp=1 -> s1 0.944 (pass), s4 0.838, s9 0.838, s11 0.933
    # so the lever is SPLIT and cannot be a global knob.  Two measured route
    # tests, both logged in pin1.route / pin1.go / pin1.skip_why:
    #   pre_branch: skip when the pre-pinch branch gate (arm_branch_gate, the
    #               same value pre_stance used) fired, i.e. the episode is on
    #               the failer branch and takes the pre_stance route.  That is
    #               the s4..s7/s10/s12 class; v32a is the number to keep there.
    #   sy         : skip when the measured pinch1 arm |shoulder_yaw| is below
    #               pin1_sy_min.  MEASURED at this read site: s9 0.1929 and
    #               s11 0.2928 pass WITHOUT the ramp; s1 0.4716 needs it.
    #               WARNING: three observations only; this is a fit, not a law,
    #               and the margin between 0.2928 and 0.4716 is 0.18 rad.
    pin1_route = off.get('pin1_route', 1)
    pin1_sy_min = off.get('pin1_sy_min', 0.35)
    pin1_lg = {'ramp_on': pin1_ramp, 'fire': 0, 'lag_on': pin1_lag_on,
               'rebase_on': pin1_rebase, 'full': pin1_full}
    if pin1_ramp:
        u0 = np.asarray(api.upper(), dtype='float64')
        a0 = u0[3:10]
        c0 = api.commanded()
        tv = np.asarray(pin1_target, dtype='float64')
        if pin1_lag_on:
            tv = tv + np.asarray(pin1_lag, dtype='float64')
        dev = np.abs(a0 - np.asarray(pin1_target, dtype='float64'))
        b0 = api.base_state()
        ob_a = ((api.state() or {}).get('object') or {}).get('pos')
        pa = pad_live()
        pin1_lg['target_cmd'] = np.round(tv, 4).tolist()
        pin1_lg['dev'] = np.round(dev, 4).tolist()
        pin1_lg['dev_max'] = np.round(np.max(dev), 4).item()
        pin1_lg['gate'] = pin1_gate
        pin1_lg['pre_arm'] = np.round(a0, 4).tolist()
        pin1_lg['pre_upper10'] = np.round(u0[0:10], 4).tolist()
        pin1_lg['pre_cmd10'] = None if c0 is None else np.round(
            np.asarray(c0, dtype='float64')[0:10], 4).tolist()
        pin1_lg['pre_base'] = b0
        pin1_lg['pre_pad'] = None if pa is None else np.round(
            np.asarray(pa), 4).tolist()
        pin1_lg['pre_pad_rel'] = None if (pa is None or ob_a is None) \
            else rel3(pa, ob_a)
        pin1_lg['pre_obj'] = None if ob_a is None else np.round(
            np.asarray(ob_a), 4).tolist()
        # v33 ROUTE: dev_max alone decides fire (the split lever is real); the
        # measured branch context decides whether the ramp is ALLOWED to run.
        pre_fire = (log.get('pre_st') or {}).get('fire')
        sy = np.abs(a0[2]).item()
        go = 1
        why = []
        if pin1_route:
            if pre_fire == 1:
                go = 0
                why.append('pre_branch')
            if sy < pin1_sy_min:
                go = 0
                why.append('sy')
        pin1_lg['route'] = pin1_route
        pin1_lg['pre_fire'] = pre_fire
        pin1_lg['sy'] = np.round(sy, 4).item()
        pin1_lg['sy_min'] = pin1_sy_min
        pin1_lg['go'] = go
        pin1_lg['skip_why'] = ','.join(why)
        pin1_lg['fire'] = 1 if pin1_lg['dev_max'] > pin1_gate else 0
        pin1_lg['ran'] = 1 if (pin1_lg['fire'] and go) else 0
        if pin1_lg['ran']:
            if pin1_full:
                tfull = u0.copy()
                tfull[3:10] = tv
                targ = tfull.tolist()
            else:
                targ = tv.tolist()
            pin1_lg['target_waist'] = np.round(
                np.asarray(targ, dtype='float64')[0:3], 4).tolist()
            rp = api.skill('arm_ramp', side='left', target=targ,
                           joints=[3, 4, 5, 6, 7, 8, 9], seg=pin1_seg,
                           seg_dur=pin1_seg_dur, tol=pin1_tol,
                           phase='v30_pin1')
            pin1_lg['ramp'] = {
                'ok': rp.get('ok'), 'error': rp.get('error'),
                'start_label': rp.get('start_label'),
                'start': rp.get('start'), 'target': rp.get('target'),
                'achieved': rp.get('achieved'), 'err': rp.get('err'),
                'max_err': rp.get('max_err'), 'max_delta': rp.get('max_delta'),
                'seg': rp.get('seg'), 'dur_total': rp.get('dur_total'),
                'tracked': rp.get('tracked'),
                'pad_before': rp.get('pad_before'),
                'pad_after': rp.get('pad_after'),
                'dpad': rp.get('dpad'), 'drift': rp.get('drift'),
                't': rp.get('t')}
            b1 = api.base_state()
            pin1_lg['base_after_ramp'] = b1
            pin1_lg['ramp_upper10'] = np.round(
                np.asarray(api.upper(), dtype='float64')[0:10], 4).tolist()
            pin1_lg['base_drift_ramp'] = np.round(
                np.asarray(b1['pos']) - np.asarray(b0['pos']), 4).tolist()
            if pin1_rebase:
                tx = np.asarray(ox).item() - pin1_rx
                ty = np.asarray(oy).item() - pin1_ry
                nr = 0
                while nr < pin1_rb_maxp:
                    sb = api.base_state()
                    ex = tx - sb['pos'][0]
                    ey = ty - sb['pos'][1]
                    if np.abs(ex) < pin1_rb_tol and np.abs(ey) < pin1_rb_tol:
                        break
                    cx = np.clip(1.6 * ex, -pin1_rb_v, pin1_rb_v).item()
                    cy = np.clip(1.6 * ey, -pin1_rb_v, pin1_rb_v).item()
                    api.move_base(cx, cy, 0.0, pin1_rb_dur)
                    nr += 1
                pin1_lg['rebase'] = {'n': nr, 'target': [tx, ty],
                                     'base': api.base_state()}
                pin1_lg['rb_upper10'] = np.round(
                    np.asarray(api.upper(), dtype='float64')[0:10], 4).tolist()
                stamp('pin1rb')
            a1 = np.asarray(api.upper()[3:10], dtype='float64')
            ob_b = ((api.state() or {}).get('object') or {}).get('pos')
            pb = pad_live()
            pin1_lg['post_arm'] = np.round(a1, 4).tolist()
            pin1_lg['d_arm'] = np.round(a1 - a0, 4).tolist()
            pin1_lg['arm_vs_passer'] = np.round(
                a1 - np.asarray(pin1_target, dtype='float64'), 4).tolist()
            pin1_lg['post_base'] = api.base_state()
            pin1_lg['post_obj'] = None if ob_b is None else np.round(
                np.asarray(ob_b), 4).tolist()
            pin1_lg['post_pad'] = None if pb is None else np.round(
                np.asarray(pb), 4).tolist()
            pin1_lg['post_pad_rel'] = None if (pb is None or ob_b is None) \
                else rel3(pb, ob_b)
            pin1_lg['t'] = np.round(api.t, 2).item()
            stamp('pin1')
        else:
            pin1_lg['post_arm'] = pin1_lg['pre_arm']
            pin1_lg['post_base'] = b0
            pin1_lg['post_pad_rel'] = pin1_lg['pre_pad_rel']
            pin1_lg['arm_vs_passer'] = np.round(dev, 4).tolist()
    log['pin1'] = pin1_lg
    # 3a. NEW in v22 (off settle=1): re-deliver the pinch reach's own solved
    # 31-D vector in segmented smoothstep ramps, then read the LIVE pad.  No
    # IK, no target change, no base or height command -- the reach_settle
    # skill owns the transport and its own contract.
    if settle:
        if last_tgt is None:
            log['settle'] = {'ok': 0, 'why': 'no upper_target'}
        else:
            ob1 = ((api.state() or {}).get('object') or {}).get('pos')
            pad0 = pad_live()
            stt = api.skill('reach_settle', upper=last_tgt, seg=settle_seg,
                            seg_dur=settle_dur, tol=settle_tol,
                            dwell=settle_dwell)
            pad1 = pad_live()
            ob2 = ((api.state() or {}).get('object') or {}).get('pos')
            log['settle'] = {'ok': stt.get('ok'),
                             'max_err_arm': stt.get('max_err_arm'),
                             'max_err_all': stt.get('max_err_all'),
                             'trace': stt.get('trace'),
                             'pad0': pad0,
                             'pad0_rel': None if (pad0 is None or ob1 is None)
                             else rel3(pad0, ob1),
                             'pad1': pad1,
                             'pad1_rel': None if (pad1 is None or ob2 is None)
                             else rel3(pad1, ob2),
                             'obj': ob2,
                             'd_arm': stt.get('d')[3:10]
                             if stt.get('d') is not None else None,
                             'ncon': ncon(),
                             't': np.round(api.t, 2).item()}
        stamp('settle')
    # 3b. GATED correction.  force=1 runs it on EVERY seed (template test).
    gated = False
    if last_seed is not None and (force or (gate_on and last_seed == 0)):
        gated = True
    log['gated'] = gated
    servo_it = (mode == 'padservo' or mode == 'padonly') and servo_on
    place_it = (mode == 'padplace')
    audit_it = (mode == 'audit')
    base_it = (mode == 'baseplace')
    branch_it = (mode == 'branch')
    ramp_it = (mode == 'gen' or mode == 'genlow' or mode == 'genlong'
               or mode == 'clone' or mode == 'clonelow'
               or mode == 'cloneaway' or mode == 'genaway')
    ispc = (mode == 'pc' or mode == 'pclow')
    away_it = (mode == 'cloneaway' or mode == 'genaway')
    lowit = (mode == 'genlow' or mode == 'clonelow' or mode == 'low'
             or mode == 'pclow')
    active = ramp_it or ispc or mode == 'low'
    log['corr'] = 'none'
    if gated and (servo_it or place_it or audit_it or base_it or branch_it):
        ob = ((api.state() or {}).get('object') or {}).get('pos')
        log['servo_inv_pre'] = inv('pre', ob)
        arm_b = np.round(api.upper()[3:10], 3).tolist()
        if audit_it:
            sv = api.skill('pad_hold_audit', side=p_side, joint=a_joint,
                           joints=a_joints, dq=a_dq, ramp=a_ramp, n_read=a_n,
                           dt=a_dt, target=a_target, phase=mode)
        elif place_it:
            sv = api.skill('pad_place', side=p_side,
                           rel=(p_relx, p_rely, p_relz), step=q_step,
                           probe_dur=q_pdur, probe_settle=q_pset, dur=q_dur,
                           settle=q_set, max_iters=q_max, tol=q_tol, lam=q_lam,
                           dq_max=q_dqmax, obj_tol=q_objtol, probe=q_probe,
                           phase=mode)
        elif base_it:
            # BASE-space placement: hold the arm/waist/hand joints exactly
            # where the pinch left them and translate the base until the
            # AVERAGED SETTLED pad reaches the passers' object-relative pose.
            sv = api.skill('base_place', side=p_side,
                           rel=(b_relx, b_rely, b_relz), tol=b_tol,
                           max_iters=b_max, step_m=b_step, vx_max=b_vx,
                           vy_max=b_vy, dur=b_dur, n_read=b_n, skip=b_skip,
                           dt=b_dt, kx=b_kx, ky=b_ky, advance_max=b_adv,
                           min_step=b_min, contact_stop=b_cstop, phase=mode)
        elif branch_it:
            # BRANCH move: on the failing seed the arm settles in a DIFFERENT
            # IK branch of the same pad target (j5 +0.4555 vs +2.619 on all
            # three passers).  v18 composes this out of the registered
            # `arm_ramp` skill (segmented joint-space transport with tracking
            # certification) instead of the v17 inline loop: arm_ramp owns the
            # start/target scatter, the segmentation and the achieved/tracked
            # read, and reports max_err so the caller can see a v15-style lag
            # instead of inferring it.  Then CERTIFY the resulting pad with a
            # settled read (pad_hold_audit, dq=0: ramp to the anchor, dwell,
            # report rel/settled/drift).
            svr = api.skill('arm_ramp', side=p_side, target=br_arm,
                            joints=None, seg=br_seg, seg_dur=br_seg_dur,
                            tol=0.06, phase=mode)
            log['branch'] = {'from': svr.get('start'),
                             'to': svr.get('target'),
                             'seg': svr.get('seg'),
                             'seg_dur': svr.get('seg_dur'),
                             'reached': svr.get('achieved'),
                             'max_err': svr.get('max_err'),
                             'tracked': svr.get('tracked'),
                             'start_label': svr.get('start_label'),
                             'ok': svr.get('ok')}
            sv = api.skill('pad_hold_audit', side=p_side, dq=0,
                           ramp=a_ramp, n_read=br_n, dt=br_dt,
                           target='commanded', phase=mode)
        else:
            sg = 0.0 if mode == 'padonly' else p_gain
            sv = api.skill('pad_servo', side=p_side,
                           rel=(p_relx, p_rely, p_relz), gain=sg, dur=p_dur,
                           max_iters=p_max, tol=p_tol, orientation=p_ornt,
                           phase=mode)
        sv['arm_before'] = arm_b
        sv['arm_after'] = np.round(api.upper()[3:10], 3).tolist()
        if base_it:
            sv['target_rel'] = [b_relx, b_rely, b_relz]
        elif branch_it:
            sv['target_rel'] = 'none'
        else:
            sv['target_rel'] = [p_relx, p_rely, p_relz]
        log['servo'] = sv
        ob2 = ((api.state() or {}).get('object') or {}).get('pos')
        log['servo_inv_post'] = inv('post', ob2)
        if audit_it:
            log['corr'] = {'mode': mode, 'pad_rel': sv.get('rel'),
                           'anchor': sv.get('anchor'),
                           'settled': sv.get('settled'),
                           'drift': sv.get('drift'),
                           'step_n': sv.get('step_n'),
                           'return_err': sv.get('return_err'),
                           'steps': sv.get('steps')}
        elif base_it:
            log['corr'] = {'mode': mode, 'stop': sv.get('stop'),
                           'converged': sv.get('converged'),
                           'iters': sv.get('iters'),
                           'err_xy': sv.get('err_xy'),
                           'err_n': sv.get('err_n'),
                           'rel_mean': sv.get('rel_mean'),
                           'base_delta': sv.get('base_delta'),
                           'pad_delta': sv.get('pad_delta'),
                           'advance_n': sv.get('advance_n'),
                           'moves': sv.get('moves'),
                           'transfer': sv.get('transfer'),
                           'contacted': sv.get('contacted')}
        elif branch_it:
            log['corr'] = {'mode': mode, 'pad_rel': sv.get('rel'),
                           'settled': sv.get('settled'),
                           'drift': sv.get('drift'),
                           'anchor': sv.get('anchor'),
                           'anchor_vec': sv.get('anchor_vec'),
                           'branch': log.get('branch')}
        else:
            log['corr'] = {'mode': mode,
                           'ik_seed': (sv.get('trace') or [{}])[-1].get('ik_seed'),
                           'pad_rel': sv.get('rel'), 'converged': sv.get('converged'),
                           'iters': sv.get('iters')}
    elif gated and active:
        tgtl = [ox + dx_pinch + corr_dx, oy + dy_pinch + corr_dy,
                z_pinch + corr_dz]
        if lowit:
            tgtl = [ox + dx_pinch + low_dx + corr_dx,
                    oy + dy_pinch + low_dy + corr_dy,
                    z_pinch + low_dz + corr_dz]
        if away_it:
            tgtl_away = [ox + pre_x, oy + pre_y, pre_z]
            ra = api.reach_box(targets={'left': tgtl_away}, sides=('left',),
                               orientation='down', dur=pre_dur)
            ea = rec('away', ra)
            ea['target'] = np.round(np.array(tgtl_away), 4).tolist()
            log['away'] = ea
        if ramp_it:
            cur = api.upper()
            if mode == 'clone' or mode == 'clonelow' or mode == 'cloneaway':
                cur[3:10] = np.array([-2.094, -0.054, 2.660, 1.139, 0.730,
                                      1.519, 1.506])
            else:
                cur[3:10] = np.array([-0.23, 0.19, -0.22, 0.71, -0.48, 1.04,
                                      0.18])
            log['pre_ramp_arm'] = np.round(api.upper()[3:10], 3).tolist()
            api._ramp(cur, gen_dur)
            log['post_ramp_arm'] = np.round(api.upper()[3:10], 3).tolist()
        pcv = 0 if ispc else corr_pc
        rep = 0
        while rep < corr_rep:
            corr = api.reach_box(targets={'left': tgtl}, sides=('left',),
                                 orientation='down', dur=corr_dur,
                                 prefer_current=pcv)
            e = rec('%s%d' % (mode, rep), corr)
            e['target'] = np.round(np.array(tgtl), 4).tolist()
            e['ramp_dur'] = gen_dur
            e['pc'] = pcv
            log['corr%d' % rep] = e
            rep += 1
        log['corr'] = log['corr%d' % (corr_rep - 1)]
        log['corr_rep'] = corr_rep
    if gated:
        jr = api.joint_ranges()
        ks = []
        for kk in jr:
            ks.append(kk)
        arm = {}
        for kk in ks[3:10]:
            arm[kk] = jr[kk]
        log['arm_limits'] = arm
        log['cpairs_corr'] = api.contact_pairs(limit=20)
    stamp('corr')
    # 3c. NEW in v24 (off seat=1).  Hand the pinch target to the pad_seat
    # skill, which owns the seating loop: it measures the live pad vs the live
    # drill root and moves the command until the measured pad matches the
    # passer reference (or iters is spent).  On the passers the first
    # measurement is already inside tol, so no command is sent and the
    # episode is unchanged.  tgt here is the same world point the pinch loop
    # commanded (ox/oy are the START object pose, as in the pinch block).
    seat_tgt = [ox + dx_pinch, oy + dy_pinch, z_pinch]
    if seat:
        ob_sv = ((api.state() or {}).get('object') or {}).get('pos')
        pad0 = pad_live()
        log['seat_pre'] = {
            'mode': seat_mode, 'obj': ob_sv, 'pad': pad0,
            'pad_rel': None if (pad0 is None or ob_sv is None)
            else np.round(np.array(pad0) - np.array(ob_sv), 4).tolist()}
        if seat_mode == 'servo' and ob_sv is not None:
            tgt_sv = (np.array(ob_sv)
                      + np.array([seat_rx, seat_ry, seat_rz])).tolist()
            log['sv'] = []
            nrep = 0
            while nrep < sv_rep:
                rsv = api.skill('servo_pad_to_point', target=tgt_sv,
                                side='left', tol=sv_tol, step=sv_step,
                                dur=sv_dur, iters=sv_it, max_steps=sv_max)
                log['sv'].append(rsv)
                nrep += 1
        elif seat_mode == 'branch':
            # v26.  Guard on the MEASURED object-relative pad, not on the
            # ik_seed label: the passers' second pinch also reports ik_seed 2.
            dev = None
            fire = 0
            if pad0 is not None and ob_sv is not None:
                dev = np.array(pad0) - (np.array(ob_sv)
                                        + np.array([seat_rx, seat_ry,
                                                    seat_rz]))
                if np.max(np.abs(dev)) > br_guard:
                    fire = 1
            log['br'] = {'guard': br_guard,
                         'dev': None if dev is None
                         else np.round(dev, 4).tolist(), 'fired': fire}
            # v36 R2-free (pre-registered, prereg_R2free.py).  The 2 mm hull
            # test R3 must not be gated by the 2 cm guard R2 on the nominal
            # seat point.  Entry is widened ONLY where the label veto
            # (gate_val > thr) would otherwise skip the transport AND the
            # measured pad is outside the hull; the existing veto block then
            # runs unchanged and its hull override clears the veto.  Every
            # other seed takes the unmodified path.
            _route = 0
            if fire == 0 and br_gate and pr_on and dev is not None:
                _gv0 = pre_lg.get('gate_val')
                if _gv0 is not None and _gv0 > br_gate_thr:
                    _pr0 = (np.asarray(dev)
                            + np.asarray([seat_rx, seat_ry, seat_rz]))
                    if (_pr0[0] < pr_x0 or _pr0[0] > pr_x1
                            or _pr0[1] < pr_y0 or _pr0[1] > pr_y1
                            or _pr0[2] < pr_z0 or _pr0[2] > pr_z1):
                        _route = 1
            if fire or _route:
                br_gout = 0
                br_gv = pre_lg.get('gate_val')
                if br_gate and br_gv is not None and br_gv > br_gate_thr:
                    br_gout = 1
                    log['br']['gated_out'] = 1
                    log['br']['gate_val'] = br_gv
                    log['br']['gate_thr'] = br_gate_thr
                    # v35 H: override the label veto with the measured pad.
                    # pad_rel = dev + the seat offset (same arithmetic as
                    # seat_pre.pad_rel).  Outside the hull => run the v31
                    # transport that every non-pre_stance seed already gets.
                    if pr_on and dev is not None:
                        _pr = (np.asarray(dev)
                               + np.asarray([seat_rx, seat_ry, seat_rz]))
                        _po = 0
                        if (_pr[0] < pr_x0 or _pr[0] > pr_x1
                                or _pr[1] < pr_y0 or _pr[1] > pr_y1
                                or _pr[2] < pr_z0 or _pr[2] > pr_z1):
                            _po = 1
                        log['br']['pr'] = np.round(_pr, 4).tolist()
                        log['br']['pr_out'] = _po
                        log['br']['pr_box'] = np.round(
                            np.asarray([pr_x0, pr_x1, pr_y0, pr_y1,
                                        pr_z0, pr_z1]), 4).tolist()
                        if _po:
                            br_gout = 0
                            log['br']['gated_out'] = 0
                            log['br']['route_pad'] = 1
                if br_on and not br_gout:
                    svr = api.skill('arm_ramp', side='left', target=br_arm,
                                    joints=None, seg=br_seg,
                                    seg_dur=br_seg_dur, tol=0.06,
                                    phase='v27_branch')
                    log['br']['ramp'] = {
                        'ok': svr.get('ok'), 'error': svr.get('error'),
                        'start_label': svr.get('start_label'),
                        'start': svr.get('start'),
                        'target': svr.get('target'),
                        'achieved': svr.get('achieved'),
                        'err': svr.get('err'),
                        'max_err': svr.get('max_err'),
                        'max_delta': svr.get('max_delta'),
                        'seg': svr.get('seg'),
                        'dur_total': svr.get('dur_total'),
                        'tracked': svr.get('tracked'), 't': svr.get('t')}
                    pb = pad_live()
                    ob_b = ((api.state() or {}).get('object')
                            or {}).get('pos')
                    log['br']['post'] = {
                        'pad': pb, 'obj': ob_b,
                        'pad_rel': None if (pb is None or ob_b is None)
                        else np.round(np.array(pb) - np.array(ob_b),
                                      4).tolist()}
                    stamp('branch')
                if ps_on and not br_gout:
                    psv = api.skill('servo_pad_translate', side='left',
                                    rel=[seat_rx, seat_ry, seat_rz],
                                    tol=ps_tol, cap=ps_cap, dur=ps_dur,
                                    iters=ps_it, max_steps=ps_max,
                                    obj_move_max=ps_objmove)
                    log['br']['sv'] = psv
                    log['seat_sv'] = {
                        'br_on': br_on, 'ok': psv.get('ok'),
                        'reason': psv.get('reason'),
                        'aborted': psv.get('aborted'),
                        'steps': psv.get('steps'),
                        'err': psv.get('err'), 'errs': psv.get('errs'),
                        'pad': psv.get('pad'),
                        'pad_rel': psv.get('pad_rel'),
                        'obj': psv.get('obj')}
                    stamp('servo')
            stamp('branch')
        else:
            log['seat'] = api.skill('pad_seat', tgt=seat_tgt,
                                    rel_ref=[seat_rx, seat_ry, seat_rz],
                                    side='left', iters=seat_it,
                                    gain=seat_gain, cap=seat_cap,
                                    tol=seat_tol, dur=seat_dur,
                                    dwell=seat_dwell, box=seat_box)
        pad1 = pad_live()
        ob1 = ((api.state() or {}).get('object') or {}).get('pos')
        log['seat_post'] = {
            'obj': ob1, 'pad': pad1,
            'pad_rel': None if (pad1 is None or ob1 is None)
            else np.round(np.array(pad1) - np.array(ob1), 4).tolist()}
        stamp('seat')
    # ---- c6 family D: settle HOLD before the close.  step_goal(upper=None)
    # sends the MEASURED 31-D pose n times: a pure hold, no ramp, no target
    # change.  Hypothesis: the pad oscillates on a ~1.2 s, 1-5 cm cycle, so
    # the close starts at an arbitrary phase; a dwell should move seed 7 off
    # the phase that rolls the drill.  (Confound, unresolved in task_memory
    # item 5: an api.upper() anchor abandons a pending IK target.)
    if off.get('preclose_hold'):
        _h0 = np.asarray(api._upper_vec(), dtype='float64').copy()
        _hd = api.step_goal(upper=None, dur=off.get('preclose_hold'))
        log['preclose_hold'] = {
            'dur': off.get('preclose_hold'),
            'upper0': np.round(_h0, 4).tolist(),
            'upper1': np.round(np.asarray(api._upper_vec()),
                               4).tolist(),
            't': _hd}
        stamp('hold')
    # actor_7 scheme D: terminate the settle on a MEASURED convergence of the
    # left pad instead of after a fixed duration.  Each step is the same pure
    # hold step_goal(upper=None) that the fixed dwell issues; only how many of
    # them are issued, and what decides to stop, changes.  pad_live() is
    # hand_geoms() (mj_forward) -- read-only, no command.  Top level on
    # purpose: it must NOT require preclose_hold.
    if off.get('close_key') == 'settle':
        _p0 = pad_live()
        _sprev = _p0
        _stol = off.get('settle_tol', 0.002)
        _smax = off.get('settle_max', 6)
        _sstep = off.get('settle_step', 0.2)
        _mvs = []
        _d = None
        _k = 0
        while _k < _smax:
            _sh = api.step_goal(upper=None, dur=_sstep)
            _pn = pad_live()
            _d = None
            if _sprev is not None and _pn is not None:
                # no numpy scalar in the log (the queue cannot serialise
                # one): manual max over the three axes
                _d = 0.0
                for _i in (0, 1, 2):
                    _a = math.fabs(_pn[_i] - _sprev[_i])
                    if _a > _d:
                        _d = _a
            _mvs.append({'k': _k, 'pad': _pn, 'mv': _d,
                         't': _sh.get('t')})
            _k += 1
            _sprev = _pn
            if _d is not None and _d <= _stol:
                break
        log['settle'] = {'pad0': _p0, 'steps': _mvs,
                         'tol': _stol, 'n': _k}
    # ---- c6 family C: right-hand BRACE on the object during the close.
    # Hypothesis: the drill ROLLS 36.36 deg about world +x because nothing
    # resists it; one external contact from the other hand should turn the
    # roll into a slide/stop.  The right arm is restored to its MEASURED
    # pre-brace vector after the close (absolute _ramp), so no reverse IK is
    # needed and the left-grip target is never touched.
    if off.get('brace'):
        _rest0_save = np.asarray(api._upper_vec(), dtype='float64')[10:17].copy()
        _ob_b = ((api.state() or {}).get('object') or {}).get('pos')
        _rz = off.get('brace_z', 1.16)
        _rbr = api.reach_box(
            targets={'right': [ox + off.get('brace_dx', 0.0),
                               oy + off.get('brace_dy', -0.13),
                               _rz]},
            sides=('right',), orientation='down',
            dur=off.get('brace_dur', 0.6))
        _ob_a = ((api.state() or {}).get('object') or {}).get('pos')
        log['brace_pre'] = {
            'obj0': _ob_b, 'obj1': _ob_a,
            'ik_err': (_rbr.get('ik_err') or {}).get('right'),
            'ik_rot': (_rbr.get('ik_rot') or {}).get('right'),
            'tgt': [ox + off.get('brace_dx', 0.0),
                    oy + off.get('brace_dy', -0.13), _rz],
            'upper_target': _rbr.get('upper_target'),
            'park': np.round(np.asarray(api._upper_vec())[10:17],
                             4).tolist()}
        stamp('brace')
    # ---- family A selector is at the close itself (below). ----
    # 4. squeeze, then freeze that exact 31-D target (v11 verbatim)
    # v23: READ-ONLY live grip probe #0, the exact pre-close instant.
    ob_pc = ((api.state() or {}).get('object') or {}).get('pos')
    if ob_pc is not None:
        log['gprobe_preclose'] = gprobe('preclose', ob_pc)
        log['gv_preclose'] = api.skill('grasp_verify')
    if off.get('close_gate_nf') is not None:
        # v42 adaptive close (drill_c7): branch on the pre-close probe that was
        # just issued above.  A condition on state the robot already observes
        # -- no seed or task literal -- and NO extra read: the probe is in the
        # log for every composition.  The sandbox exposes only np and math
        # to this program, so the block stays with plain arithmetic.
        # Both branches are 3-stage plans of exactly 25 control steps, so the
        # read count/order and the step count are identical between them;
        # only the index partition and the per-stage durations differ.
        # _gate_hi = True  -> thumb-first (measured TRUE on s7, FALSE on s2):
        #                     identical to the A3c plan at angle 1.4.
        # _gate_hi = False -> 25-step magnitude (measured TRUE on s2 and s3,
        #                     FALSE on s7): identical to the G3 plan at 1.415.
        # Ablation without editing: thr <= 0 forces thumb, thr > 1 forces mag2.
        _gvp = log.get('gv_preclose') or {}
        _nf = _gvp.get('n_fing')
        if _nf is None:
            _nf = 0
        _thr = off.get('close_gate_nf')
        _gate_hi = _nf >= _thr
        # actor_7 scheme A: the branch variable becomes the measured pre-close
        # thumb PLACEMENT (world y relative to the drill root) as an ADDITIONAL
        # way to select thumb-first.  The contact count keeps its old
        # authority, so this block is a no-op whenever n_fing already fires.
        # Close plan, angle and step count for each branch are unchanged.
        _ck = off.get('close_key', 'nf')
        _ty = None
        _g2 = _gvp.get('thumb2')
        if _g2 is not None:
            _ty = _g2[1]
        log['close_key'] = {'key': _ck, 'thumb2_y': _ty,
                            'nf': _nf, 'nf_hi': _gate_hi}
        if _ck == 'geom' and _ty is not None:
            _gty = off.get('geom_thumb2_y', -0.055)
            _gfire = 0
            if _ty <= _gty:
                _gfire = 1
            if _gfire:
                _gate_hi = True
            log['close_key']['thr_y'] = _gty
            log['close_key']['fire'] = _gfire
            log['close_key']['branch'] = 'thumb' if _gate_hi else 'mag2'
        if _gate_hi:
            off['close_mode'] = 'thumbfirst'
            off['close_angle'] = 1.4
            off['close_plan'] = [[[21, 22, 23], 1.00, 0.18],
                                 [[17, 18, 19, 20], 1.00, 0.24],
                                 [[24, 25, 26, 27, 28, 29, 30], 1.00, 0.08]]
        else:
            off['close_mode'] = 'mag2'
            off['close_angle'] = 1.415
            off['close_plan'] = [[[17, 18, 19, 20, 21, 22, 23], 0.45, 0.20],
                                 [[17, 18, 19, 20, 21, 22, 23], 0.85, 0.15],
                                 [[17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
                                   27, 28, 29, 30], 1.00, 0.16]]
        log['close_gate'] = {'n_fing': _nf, 'thr': _thr,
                             'branch': 'thumb' if _gate_hi else 'mag2'}
    if off.get('close_mode'):
        # family A: same FINAL 31-D target as close_grip, delivered over a
        # plan of sub-ramps.  'one' is the null control (single stage, 0.50 s
        # = close_grip's duration; with no limit binding at angle 1.4 the
        # target is bit-identical), 'mag2' stages the MAGNITUDE 45/85/100 %,
        # 'fingfirst'/'thumbfirst' stage the JOINT ORDER.
        _cm = off.get('close_mode')
        _cp = off.get('close_plan')
        if _cp is None:
            if _cm == 'one':
                _cp = [[[17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
                         29, 30], 1.00, 0.50]]
            elif _cm == 'mag2':
                _cp = [[[17, 18, 19, 20, 21, 22, 23], 0.45, 0.20],
                       [[17, 18, 19, 20, 21, 22, 23], 0.85, 0.15],
                       [[17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
                         29, 30], 1.00, 0.15]]
            elif _cm == 'fingfirst':
                _cp = [[[17, 18, 19, 20], 1.00, 0.25],
                       [[21, 22, 23, 24, 25, 26, 27, 28, 29, 30], 1.00, 0.25]]
            elif _cm == 'thumbfirst':
                _cp = [[[21, 22, 23], 1.00, 0.15],
                       [[17, 18, 19, 20], 1.00, 0.25],
                       [[24, 25, 26, 27, 28, 29, 30], 1.00, 0.10]]
        g = api.skill('staged_close', angle=off.get('close_angle', 1.4),
                      plan=_cp)
        log['close_stages'] = g
        log['close_mode'] = _cm
    else:
        g = api.close_grip(1.4, 0.5)
    gv = api.commanded()
    if gv is None:
        gv = np.array(g.get('upper_target'), dtype='float64')
    s = api.observe()
    log['grip'] = {'ncon': ncon(), 'obj': (s.get('object') or {}).get('pos'),
                   'base': s.get('base'),
                   'held': np.round(gv, 4).tolist(),
                   't': np.round(api.t, 2).item()}
    # v23: READ-ONLY live grip probe #1, immediately after the close.
    ob_g = (s.get('object') or {}).get('pos')
    if ob_g is not None:
        log['gprobe_grip'] = gprobe('grip', ob_g)
        log['gv_grip'] = api.skill('grasp_verify')
    if gated:
        log['cpairs_grip'] = api.contact_pairs(limit=20)
    stamp('grip')
    if off.get('brace') and not off.get('brace_hold'):
        # absolute ramp from the CURRENT measured pose with only the right
        # arm joints overwritten back to their pre-brace vector: no IK, and
        # the left-hand hold target is untouched.
        _rst = np.asarray(api._upper_vec(), dtype='float64').copy()
        _rest0 = np.round(_rest0_save, 4).tolist()
        _rst[10:17] = np.array(_rest0_save, dtype='float64')
        api._ramp(_rst, off.get('brace_park_dur', 0.30))
        _ob_p = ((api.state() or {}).get('object') or {}).get('pos')
        log['brace_post'] = {
            'from': _rest0,
            'to': np.round(_rst, 4).tolist(),
            'obj': _ob_p,
            't': np.round(api.t, 2).item()}
        stamp('park')
    lx = ox + dx_lift
    ly = oy + dy_lift
    if gated and reanch:
        op = (log['grip'].get('obj') or None)
        if op is not None:
            lx = op[0] + dx_lift
            ly = op[1] + dy_lift
        log['reanch'] = {'obj': op, 'lx': np.round(lx, 4).item(),
                         'ly': np.round(ly, 4).item()}
    else:
        log['reanch'] = 'none'
    # 5. four small arm lifts, hand joints HELD at the closed target, with the
    #    v11 measured-height early stop.
    j = 0
    while j < 4:
        z = 1.12 + 0.03 * (j + 1)
        if hold:
            rr = api.reach_box(targets={'left': [lx, ly, z]},
                               sides=('left',), orientation='down', dur=0.5,
                               hand_hold=gv)
        else:
            rr = api.reach_box(targets={'left': [lx, ly, z]},
                               sides=('left',), orientation='down', dur=0.5)
        st = rr.get('state') or {}
        obz = (st.get('object') or {}).get('pos')
        uu = api.upper()
        log['lift'].append({'i': j, 'z': np.round(z, 3).item(),
                            'obj': obz,
                            'ncon': ncon(),
                            'palm_res': rr.get('palm_residual'),
                            'ik_seed': rr.get('ik_seed'),
                            'arm': np.round(uu[3:17], 3).tolist(),
                            't': np.round(api.t, 2).item()})
        j += 1
        if j == 1 and obz is not None:
            # v23: READ-ONLY live grip probe #2, right after the first lift.
            log['gprobe_lift0'] = gprobe('lift0', obz)
            log['gv_lift0'] = api.skill('grasp_verify')
        if gate_on and obz is not None and obz[2] >= z_stop:
            log['lift'][-1]['stop'] = 'object_high'
            log['stop_at_lift'] = j
            break
        # actor_7 scheme B: after lift stage 0, if the ARM lift did not move
        # the object (measured dz below lift_gain_min), switch ACTUATOR: raise
        # the base height with the closed 31-D grip target frozen.  step_goal
        # with an explicit upper vector never re-reads the measured fingers,
        # so the grasp is not relaxed (set_height/lift would relax it).  Clip
        # 0.90 is the engine's own limit for this path; no read or command is
        # added unless the measured dz test fires, so passers are bit-stable.
        if j == 1 and off.get('lift_stand'):
            _z0 = (log['grip'].get('obj') or [None, None, None])[2]
            _z1 = None
            if obz is not None:
                _z1 = obz[2]
            _dz = 0.0
            if _z0 is not None and _z1 is not None:
                _dz = _z1 - _z0
            if _dz < off.get('lift_gain_min', 0.005):
                _h0 = api.height
                _cmd = np.asarray(gv, dtype='float64').copy()
                _sl = []
                _reach = 0
                _k = 0
                _sn = off.get('lift_stand_n', 4)
                while _k < _sn:                     # no range() in the sandbox
                    _ht = _h0 + 0.04 * (_k + 1)
                    if _ht > 0.90:
                        _ht = 0.90
                    api.step_goal(upper=_cmd, nav=[0.0, 0.0, 0.0],
                                  height=_ht,
                                  dur=off.get('lift_stand_dur', 0.6))
                    _ob = ((api.state() or {}).get('object') or {}).get('pos')
                    _oz = None
                    if _ob is not None:
                        _oz = _ob[2]
                    _sl.append({'h': np.round(_ht, 3).item(),
                                'obj': _ob,
                                't': np.round(api.t, 2).item()})
                    _k += 1
                    if _oz is not None and _oz >= z_stop:
                        _sl[-1]['reach'] = 1
                        _reach = 1
                        break
                log['lift_stand'] = {'h0': _h0, 'dz_arm': _dz,
                                     'steps': _sl,
                                     'cmd_grip': np.round(_cmd, 4).tolist()}
                stamp('stand')
                if _reach:
                    log['stop_at_lift'] = 'stand'
                    break
    st = api.observe()
    log['final_obj'] = (st.get('object') or {}).get('pos')
    log['final_ncon'] = ncon()
    log['base'] = st.get('base')
    stamp('end')
    return log
