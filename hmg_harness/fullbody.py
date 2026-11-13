"""43-DoF full-body control contract for the HMG harness.

The ONLY interface candidate programs see.  Everything else (env internals,
reward, success predicate, safety limits) stays in the trusted adapter.

State  : 43 joint positions + velocities + base pose/velocity + task obs.
Action : 43 joint POSITION targets in the canonical order below, plus an
         optional per-step mode.  The adapter maps them to the official HMG
         action vector (``{"q": ...}``) after checking limits.

Canonical order (by name, resolved at runtime -- never by index):
  29 body joints  (legs, waist, arms)  +  14 hand joints (index/middle/thumb)
"""
import numpy as np

BODY_JOINTS = [
    'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee',
    'left_ankle_pitch', 'left_ankle_roll',
    'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee',
    'right_ankle_pitch', 'right_ankle_roll',
    'waist_yaw', 'waist_roll', 'waist_pitch',
    'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
    'left_elbow', 'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
    'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
    'right_elbow', 'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
]
HAND_JOINTS = [
    'left_hand_index_0', 'left_hand_index_1',
    'left_hand_middle_0', 'left_hand_middle_1',
    'left_hand_thumb_0', 'left_hand_thumb_1', 'left_hand_thumb_2',
    'right_hand_index_0', 'right_hand_index_1',
    'right_hand_middle_0', 'right_hand_middle_1',
    'right_hand_thumb_0', 'right_hand_thumb_1', 'right_hand_thumb_2',
]
JOINTS = [j + '_joint' for j in BODY_JOINTS + HAND_JOINTS]
N = 43
assert len(JOINTS) == N and len(set(JOINTS)) == N


class FullBodyState:
    """One-time-aligned snapshot handed to the program."""
    __slots__ = ('q', 'qd', 'base_pos', 'base_quat', 'base_vel', 'time',
                 'task_obs')

    def __init__(self, q, qd, base_pos, base_quat, base_vel, t, task_obs):
        self.q = np.asarray(q, float)
        self.qd = np.asarray(qd, float)
        self.base_pos = np.asarray(base_pos, float)
        self.base_quat = np.asarray(base_quat, float)
        self.base_vel = np.asarray(base_vel, float)
        self.time = float(t)
        self.task_obs = task_obs


class JointMap:
    """Resolve the canonical 43 names against a live HMG env (by name)."""

    def __init__(self, env):
        self.names = JOINTS
        self.qpos_idx = np.zeros(N, int)
        self.qvel_idx = np.zeros(N, int)
        self.lo = np.zeros(N)
        self.hi = np.zeros(N)
        self.nu_idx = np.zeros(N, int)   # actuator index per joint (-1 = none)
        self.missing = []
        sim = env.sim
        for i, name in enumerate(JOINTS):
            # HMG's G1 puts the hands on their own gripper prefix:
            #   robot0_<body_joint>  /  gripper0_{left,right}_<hand_joint>
            cands = []
            if name.startswith('left_'):
                cands.append('gripper0_left_' + name)
            if name.startswith('right_'):
                cands.append('gripper0_right_' + name)
            cands += ['robot0_' + name, name]
            jid = None
            for cand in cands:
                jid = mujoco_name2id(sim.model, cand)
                if jid is not None:
                    break
            if jid is None:
                self.missing.append(name)
                continue
            self.qpos_idx[i] = sim.model.jnt_qposadr[jid]
            self.qvel_idx[i] = sim.model.jnt_dofadr[jid]
            self.lo[i], self.hi[i] = sim.model.jnt_range[jid]
            self.nu_idx[i] = -1
        if self.missing:
            raise RuntimeError('joints missing in the model: %s' % self.missing)
        # the env's action vector is the ACTUATOR order; map each canonical
        # joint to its slot there (robosuite native backend)
        self.act_slot = np.full(N, -1, int)
        for a in range(sim.model.nu):
            j = int(sim.model.actuator_trnid[a, 0])
            for i, name in enumerate(JOINTS):
                cands = []
                if name.startswith('left_'):
                    cands.append('gripper0_left_' + name)
                if name.startswith('right_'):
                    cands.append('gripper0_right_' + name)
                cands += ['robot0_' + name, name]
                if any(sim.model.joint(j).name == c for c in cands):
                    self.act_slot[i] = a
                    break

    def read_q(self, env):
        return env.sim.data.qpos[self.qpos_idx].copy()

    def read_qd(self, env):
        return env.sim.data.qvel[self.qvel_idx].copy()

    def clip(self, q):
        return np.clip(np.asarray(q, float), self.lo, self.hi)


def mujoco_name2id(model, name):
    try:
        return int(model.joint_name2id(name))
    except Exception:
        return None


def to_env_action(jm, q43):
    """Canonical 43 -> the env's actuator order (native backend)."""
    a = np.zeros(int(jm.act_slot.max()) + 1)
    for i, slot in enumerate(jm.act_slot):
        if slot >= 0:
            a[slot] = q43[i]
    return a


def build_state(env, jm, task_obs=None, t=0.0):
    d = env.sim.data
    return FullBodyState(
        q=jm.read_q(env), qd=jm.read_qd(env),
        base_pos=d.qpos[0:3].copy(), base_quat=d.qpos[3:7].copy(),
        base_vel=d.qvel[0:6].copy(), t=t, task_obs=task_obs)


# ---------------------------------------------------------------------------
# HMG's 35-D WBC goal action (what the released policy emits):
#     35 = 31 upper body + 3 navigation (vx, vy, wz) + 1 base height
#     31 = waist 3 + arms 14 + hands 14
# The 43-DoF number is the robot's JOINT count (12 legs + 3 waist + 14 arms
# + 14 hands); the two count different things and are not alternatives.
UPPER_BODY_JOINTS = JOINTS[12:]          # 31: waist + arms + hands
NAV_DIM, HEIGHT_DIM = 3, 1
WBC_GOAL_DIM = len(UPPER_BODY_JOINTS) + NAV_DIM + HEIGHT_DIM   # 35
assert WBC_GOAL_DIM == 35 and len(UPPER_BODY_JOINTS) == 31


class WBCGoal:
    """35-D high-level goal for the HMG WBC backend."""
    __slots__ = ('upper', 'nav', 'height')

    def __init__(self, upper=None, nav=None, height=0.0):
        self.upper = np.zeros(31) if upper is None else np.asarray(upper, float)
        self.nav = np.zeros(3) if nav is None else np.asarray(nav, float)
        self.height = float(height)

    def to_vec(self):
        return np.r_[self.upper, self.nav, self.height]

    @classmethod
    def from_vec(cls, v):
        v = np.asarray(v, float)
        assert v.shape == (35,), v.shape
        return cls(v[:31], v[31:34], v[34])


def wbc_goal_to_joint_targets(goal, q_current, dheight=0.0):
    """Default mapping when no WBC backend is attached: the 31 upper-body
    targets go straight to their joints, the legs hold, and the nav command
    is returned separately for a walking backend to consume."""
    q = np.asarray(q_current, float).copy()
    q[12:] = goal.upper
    return q, np.asarray(goal.nav, float), float(goal.height)
