"""Isaac Lab env — Pendulum target tracking via PVT (ImplicitActuator).

Designed to match the real X6 driven in PVT Mode 5:
    τ = stiffness · (target_pd − pos) + damping · (target_vel − vel)

Mirror of pendulum_pd_env_cfg.py with key differences:
  - **Actuator**: ImplicitActuatorCfg(stiffness=200, damping=12, effort_limit=20)
    instead of NoisyDCMotor. Matches PVT formula exactly (no DCMotor back-EMF
    or current-loop dynamics — drive's current loop is invisible to master in
    real PVT and runs much faster than physics dt anyway).
  - **Physics rate**: dt=0.001 (1 kHz) — same as EtherCAT cycle on real.
    decimation=20 → policy at 50 Hz.
  - **Friction DR**: (0.5, 2.5) Nm — covers the real X6 stiction we measured
    on 2026-05-13 (~2 Nm break-away). Wider range than CAN 0xA4 (0.1, 0.8).
  - **Velocity threshold**: 5 rad/s (was 8 in pd env) — user constraint.
  - **No master-side target slewing**: the commanded target jumps directly to
    a new random value on each resample. The policy MUST learn to interpolate
    by ramping its action output over multiple steps — the high_velocity and
    action_rate rewards punish abrupt target_pd jumps. This is the whole point
    of training: the deployed ONNX must publish smooth position trajectories
    on its own, without any external rate-limiter.
  - **Action scale = 2π/3** (vs π/3 in pd env): max offset = ±2π so the policy
    can fully cancel any jump in [0, 2π] on step 0 by setting action = ∓3,
    then ramp back toward 0 over subsequent steps.
  - **No shared noise**: ImplicitActuator runs PD on clean physics state.
    Observation noise is added separately via UniformNoiseCfg on each term —
    mirrors the real wiring where drive uses its own encoder (clean) and
    /joint_states (noisy) feeds the policy.
"""

from __future__ import annotations

import math
import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.buffers import DelayBuffer
from isaaclab.utils.noise import UniformNoiseCfg

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


VELOCITY_LIMIT = 16.0          # rad/s, for obs normalization
V_MAX = 5.5                    # rad/s — user constraint, soft-enforced via reward
ERR_SCALE = math.pi            # err/π keeps obs in ≈[−2, +2]


# =============================================================================
# Variable target — raw step jumps (no master-side slewing).
#
# Policy MUST learn to interpolate on its own: at each resample the commanded
# target jumps to a new random value in [0, 2π], and the high_velocity /
# action_rate rewards punish the policy if it lets motor velocity exceed
# V_MAX. The policy's optimal strategy is to RAMP its action output (offset
# from commanded target) over multiple policy steps, producing a smooth
# target_pd trajectory that the actuator can track at ≤ V_MAX without
# overshoot.
# =============================================================================

_target_user: torch.Tensor | None = None


def _resample_targets(env: "ManagerBasedRLEnv", env_ids: torch.Tensor | None = None):
    global _target_user
    num_envs = env.num_envs
    device = env.device
    two_pi = 2.0 * math.pi
    if _target_user is None or _target_user.shape[0] != num_envs:
        _target_user = torch.zeros(num_envs, 1, device=device)
    if env_ids is None:
        env_ids = torch.arange(num_envs, device=device)
    new = torch.rand(len(env_ids), device=device) * two_pi
    _target_user[env_ids] = new.unsqueeze(1)


def target_position(env: "ManagerBasedRLEnv") -> torch.Tensor:
    """Return the commanded target — NO slewing. Policy sees raw step jumps
    and must learn to interpolate via its action output."""
    global _target_user
    if _target_user is None:
        _resample_targets(env)
    return _target_user


# =============================================================================
# Action — target-offset with per-env latency (same as pd env)
# =============================================================================

class TargetOffsetJointPositionAction(JointPositionAction):
    """target_pd = scale * action + commanded_target (per policy step)."""

    def __init__(self, cfg: "TargetOffsetJointPositionActionCfg", env):
        super().__init__(cfg, env)
        self._max_delay = cfg.max_delay_steps
        self._buffer = DelayBuffer(
            history_length=self._max_delay + 1,
            batch_size=env.num_envs,
            device=env.device,
        )

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if env_ids is None:
            env_ids = torch.arange(self._env.num_envs, device=self._env.device)
        lags = torch.randint(
            0, self._max_delay + 1, (len(env_ids),),
            dtype=torch.int, device=self._env.device,
        )
        self._buffer.set_time_lag(lags, env_ids)
        self._buffer.reset(env_ids)

    def apply_actions(self):
        delayed = self._buffer.compute(self.processed_actions)
        target = target_position(self._env) + delayed
        self._asset.set_joint_position_target(target, joint_ids=self._joint_ids)


@configclass
class TargetOffsetJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = TargetOffsetJointPositionAction
    max_delay_steps: int = 12


# =============================================================================
# Joint indexing — same as pd env (handles compliance Weight DOF)
# =============================================================================

_PJ_IDS: slice | None = None


def _pj_ids(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg) -> slice:
    global _PJ_IDS
    if _PJ_IDS is None:
        ids, _ = env.scene[asset_cfg.name].find_joints("pendulum_joint")
        i = int(list(ids)[0])
        _PJ_IDS = slice(i, i + 1)
    return _PJ_IDS


def _as_torch(x):
    if isinstance(x, torch.Tensor):
        return x
    import warp as wp
    return wp.to_torch(x)


def _joint_state(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg):
    """Read clean (pos, vel) from physics for pendulum_joint only.
    ImplicitActuator runs PD on this same state internally — no shared-noise
    indirection needed; obs noise is added separately via UniformNoiseCfg."""
    asset = env.scene[asset_cfg.name]
    pj = _pj_ids(env, asset_cfg)
    return _as_torch(asset.data.joint_pos[:, pj]), _as_torch(asset.data.joint_vel[:, pj])


# =============================================================================
# Observation functions
# =============================================================================

def linear_err(env: "ManagerBasedRLEnv",
               asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    pos, _ = _joint_state(env, asset_cfg)
    return (target_position(env) - pos) / ERR_SCALE


def normalized_vel(env: "ManagerBasedRLEnv",
                   asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    _, vel = _joint_state(env, asset_cfg)
    return vel / VELOCITY_LIMIT


def pos_sin(env: "ManagerBasedRLEnv",
            asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    pos, _ = _joint_state(env, asset_cfg)
    return torch.sin(pos)


def pos_cos(env: "ManagerBasedRLEnv",
            asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    pos, _ = _joint_state(env, asset_cfg)
    return torch.cos(pos)


# =============================================================================
# Reward functions
# =============================================================================

def _raw_err(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg) -> torch.Tensor:
    theta = _as_torch(env.scene[asset_cfg.name].data.joint_pos[:, _pj_ids(env, asset_cfg)])
    return target_position(env) - theta


def tracking_reward(env: "ManagerBasedRLEnv",
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    err = _raw_err(env, asset_cfg)
    return torch.exp(-0.2 * (err ** 2)).sum(dim=-1)


def velocity_penalty(env: "ManagerBasedRLEnv",
                     asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    vel = _as_torch(env.scene[asset_cfg.name].data.joint_vel[:, _pj_ids(env, asset_cfg)])
    return (vel ** 2).sum(dim=-1)


def action_smoothness_penalty(env: "ManagerBasedRLEnv") -> torch.Tensor:
    a = env.action_manager.action
    a_prev = env.action_manager.prev_action
    return ((a - a_prev) ** 2).sum(dim=-1)


def action_rate_penalty(env: "ManagerBasedRLEnv") -> torch.Tensor:
    """Penalize action-rate exceeding V_MAX (hinge, quadratic above).

    Action diff (Δa) per policy step → effective contribution to target_pd
    change is scale·Δa rad/step. Threshold = V_MAX·dt/scale (the max Δa allowed
    if no commanded_target change). With scale=2π/3, dt=0.02, V_MAX=5: ≈0.0477.
    """
    a = env.action_manager.action
    a_prev = env.action_manager.prev_action
    threshold = (V_MAX * 0.02) / (2.0 * math.pi / 3.0)
    excess = torch.clamp(torch.abs(a - a_prev) - threshold, min=0.0)
    return (excess ** 2).sum(dim=-1)


# Cache for previous commanded target — needed to compute target_pd rate of change
_target_user_prev: torch.Tensor | None = None
_last_rate_step: int = -1


def target_pd_rate_penalty(env: "ManagerBasedRLEnv") -> torch.Tensor:
    """Penalize |Δtarget_pd|/dt exceeding V_MAX. QUADRATIC with excess cap.

    Quadratic + cap=20 gives steep penalty on abrupt jumps (excess 25→cap 20
    → penalty 400 × weight) while keeping moderate slewing (excess 1-2) cheap.
    This is the gradient shape needed to penalize SUDDEN starts more than
    SUSTAINED slewing. Earlier linear penalty (weight -8) was too uniform —
    didn't discriminate between abrupt and gradual rate violations.

    Cap=20 prevents single-step catastrophic penalty from collapsing policy
    std (had collapse issues earlier with uncapped quadratic at weight -5).
    """
    global _target_user_prev, _last_rate_step
    scale = 2.0 * math.pi / 3.0
    a = env.action_manager.action
    a_prev = env.action_manager.prev_action

    cmd_curr = _target_user
    if _target_user_prev is None or _target_user_prev.shape != cmd_curr.shape:
        _target_user_prev = cmd_curr.clone()

    target_pd_curr = scale * a + cmd_curr
    target_pd_prev = scale * a_prev + _target_user_prev

    dt = float(env.step_dt) if hasattr(env, "step_dt") else 0.02
    rate = (target_pd_curr - target_pd_prev) / dt
    excess = torch.clamp(torch.abs(rate) - V_MAX, min=0.0, max=20.0)

    step = int(env.common_step_counter)
    if step != _last_rate_step:
        _last_rate_step = step
        _target_user_prev = cmd_curr.clone()

    return (excess ** 2).sum(dim=-1)   # QUADRATIC with cap


def tracking_bonus(env: "ManagerBasedRLEnv",
                   asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
                   threshold_rad: float = 0.3) -> torch.Tensor:
    err = _raw_err(env, asset_cfg)
    return (torch.abs(err) < threshold_rad).float().sum(dim=-1)


def precision_bonus(env: "ManagerBasedRLEnv",
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
                    threshold_rad: float = 0.035) -> torch.Tensor:
    err = _raw_err(env, asset_cfg)
    return (torch.abs(err) < threshold_rad).float().sum(dim=-1)


def high_velocity_penalty(env: "ManagerBasedRLEnv",
                          asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
                          threshold: float = V_MAX) -> torch.Tensor:
    """Hinge penalty: zero below v_max, quadratic above. Drives joint vel to ≤5 rad/s."""
    vel = _as_torch(env.scene[asset_cfg.name].data.joint_vel[:, _pj_ids(env, asset_cfg)])
    excess = torch.clamp(torch.abs(vel) - threshold, min=0.0)
    return (excess ** 2).sum(dim=-1)


def settled_velocity_penalty(env: "ManagerBasedRLEnv",
                             asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    vel = _as_torch(env.scene[asset_cfg.name].data.joint_vel[:, _pj_ids(env, asset_cfg)])
    err = _raw_err(env, asset_cfg)
    near = torch.exp(-6.0 * err ** 2)
    return (near * vel ** 2).sum(dim=-1)


def acceleration_penalty(env: "ManagerBasedRLEnv",
                         asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    accel = _as_torch(env.scene[asset_cfg.name].data.joint_acc[:, _pj_ids(env, asset_cfg)])
    return (accel ** 2).sum(dim=-1)


def settled_action_penalty(env: "ManagerBasedRLEnv",
                           asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    a = env.action_manager.action
    a_prev = env.action_manager.prev_action
    err = _raw_err(env, asset_cfg)
    near = torch.exp(-6.0 * err ** 2)
    return (near * (a - a_prev) ** 2).sum(dim=-1)


# =============================================================================
# Scene
# =============================================================================

import os as _os
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
# URDF lives in training/assets/; its mesh refs resolve into
# src/pendulum_description/meshes/ (relative within the v2 workspace).
PENDULUM_URDF = _os.path.normpath(
    _os.path.join(_THIS_DIR, "..", "assets", "pendulum_isaac_swingup_compliant.urdf"))


@configclass
class PendulumPvtSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0, dynamic_friction=1.0, restitution=0.0,
        ),
    )

    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Pendulum",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=PENDULUM_URDF,
            fix_base=True,
            merge_fixed_joints=False,
            joint_drive=None,
            self_collision=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=8,
            ),
        ),
        actuators={
            # PVT-equivalent: τ = K·(target_pd − pos) + D·(target_vel − vel).
            # Defaults match the real-drive yaml (kp=200, kd=12, max=20 Nm).
            "x6": ImplicitActuatorCfg(
                joint_names_expr=["pendulum_joint"],
                stiffness=200.0,
                damping=12.0,
                effort_limit=20.0,
                velocity_limit=VELOCITY_LIMIT,
                friction=0.0,        # set per-episode via EventsCfg.randomize_friction
                armature=0.022,      # rotor inertia reflected to joint axis
            ),
            # Passive bar/mount compliance — same as pd env, k=1500, c=0.005.
            "compliance": ImplicitActuatorCfg(
                joint_names_expr=["Weight"],
                stiffness=1500.0,
                damping=0.005,
                effort_limit=100.0,
                velocity_limit=50.0,
            ),
        },
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.6),
            joint_pos={"pendulum_joint": 0.0, "Weight": 0.0},
        ),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=3000.0),
    )


# =============================================================================
# Actions — same offset architecture as pd env, scale tuned for lower stiffness
# =============================================================================

@configclass
class ActionsCfg:
    # scale=2π/3 with clip=3 → target_pd ∈ [commanded − 2π, commanded + 2π].
    #
    # Wider span than pd env (which was π/3): without master-side slewing, the
    # commanded target jumps by up to 2π at every resample. The policy needs
    # max offset = ±2π in its action space to fully cancel the jump in step 0
    # (output action=-3 → target_pd = pos, no instantaneous motor jerk), then
    # ramp action back toward 0 over subsequent steps to follow v_max=5 rad/s.
    #
    # Trade-off: action precision near steady state is coarser (Δa=0.01 →
    # Δtarget_pd ≈ 0.021 rad). settled_action reward + action_smooth keep this
    # in check during steady tracking.
    joint_pos = TargetOffsetJointPositionActionCfg(
        asset_name="robot",
        joint_names=["pendulum_joint"],
        scale=2.0 * math.pi / 3.0,
        use_default_offset=False,
        max_delay_steps=12,
    )


# =============================================================================
# Observations — 8-dim, same structure as pd env, but noise added explicitly
# (no NoisyDCMotor shared-noise) to mirror /joint_states wiring on real.
# =============================================================================

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        # [0] (target − pos) / π — encoder σ ≈ 5e-5 rad, /π ≈ 1.6e-5. Negligible.
        err_obs = ObservationTermCfg(
            func=linear_err,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=UniformNoiseCfg(n_min=-0.0005, n_max=0.0005),
        )
        # [1] vel / VELOCITY_LIMIT — real X6 vel ripple ~0.5 rad/s, /16 ≈ 0.03.
        vel_norm = ObservationTermCfg(
            func=normalized_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=UniformNoiseCfg(n_min=-0.05, n_max=0.05),
        )
        # [2] sin(pos), [3] cos(pos) — encoder noise too small to matter on trig.
        sin_pos = ObservationTermCfg(
            func=pos_sin,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        cos_pos = ObservationTermCfg(
            func=pos_cos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        # [4..7] prev_action history — oscillation detection
        prev_action = ObservationTermCfg(
            func=mdp.last_action,
            history_length=4,
            flatten_history_dim=True,
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# =============================================================================
# Events — DR + reset
# =============================================================================

@configclass
class EventsCfg:
    randomize_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="urdf_weight"),
            "mass_distribution_params": (0.29, 1.93),
            "operation": "scale",
        },
    )

    randomize_com = EventTermCfg(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="urdf_weight"),
            "com_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (-0.05, 0.05)},
        },
    )

    # Friction DR — covers the real X6 stiction we measured on 2026-05-13.
    # Empirical break-away ≈ 1.8–2.4 Nm (kp=60→200 sweep). Range (0.5, 2.5)
    # ensures policy sees and learns to compensate for stiction-dominated
    # regimes typical of the 19.612:1 gear.
    randomize_friction = EventTermCfg(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names="pendulum_joint"),
            "friction_distribution_params": (0.5, 2.5),
            "operation": "abs",
        },
    )

    # Gain DR ±25% around the deploy nominal (K=200, D=12).
    randomize_actuator_gains = EventTermCfg(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names="pendulum_joint"),
            "stiffness_distribution_params": (150.0, 250.0),
            "damping_distribution_params":   (8.0, 16.0),
            "operation": "abs",
        },
    )

    # pos uniform in [0, 2π] (matches target range).
    reset_joints = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names="pendulum_joint"),
            "position_range": (0.0, 2.0 * math.pi),
            "velocity_range": (-0.0025, 0.0025),
        },
    )

    reset_target = EventTermCfg(
        func=_resample_targets,
        mode="reset",
    )

    resample_target_interval = EventTermCfg(
        func=_resample_targets,
        mode="interval",
        interval_range_s=(3.0, 6.0),
        is_global_time=False,
    )

    # Disturbance pushes — smaller than pd env (±5 Nm vs ±10) since PVT is
    # softer than CAN-0xA4 PD; ±5 Nm tests recovery without overwhelming.
    disturbance_push = EventTermCfg(
        func=mdp.apply_external_force_torque,
        mode="interval",
        interval_range_s=(4.0, 8.0),
        is_global_time=False,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="urdf_profile_700"),
            "force_range":  (0.0, 0.0),
            "torque_range": (-5.0, 5.0),
        },
    )


# =============================================================================
# Rewards
# =============================================================================

@configclass
class RewardsCfg:
    """Pure physics-signal rewards — no synthetic rate penalties.

    Previous attempts with action_rate / target_pd_rate hinge penalties
    collapsed policy std to 0.04 within 30-150 iterations: their excess²
    gradient is too sharp on early random actions, the adaptive-KL scheduler
    then crushes exploration. This config relies on `high_velocity` and
    `acceleration` (direct physics consequences) to shape smooth output —
    they only activate when the motor actually misbehaves, so early-training
    variance is bounded by physics, not by mathematical excess functions.
    """
    tracking         = RewardTermCfg(func=tracking_reward, weight=1.0)
    velocity         = RewardTermCfg(func=velocity_penalty, weight=-0.05)
    action_smooth    = RewardTermCfg(func=action_smoothness_penalty, weight=-0.1)
    # Quadratic + cap=20 target_pd_rate. Weight reduced -3 → -0.5 after warm-
    # start from model_2800 showed policy regressing: at original -3 weight,
    # target_pd_rate penalty was -9.8/episode (12× more than linear -8 produced
    # for same behavior), dominating reward and degrading tracking 0.90 → 0.51.
    # At -0.5: abrupt step (excess 20) = -200, slow ramp (excess 1-2) ≈ -1, so
    # only ABRUPT starts get bite while normal slewing stays cheap.
    target_pd_rate   = RewardTermCfg(func=target_pd_rate_penalty, weight=-0.5)
    tracking_b       = RewardTermCfg(
        func=tracking_bonus, weight=5.0,
        params={"threshold_rad": 0.3},
    )
    precision        = RewardTermCfg(
        func=precision_bonus, weight=8.0,
        params={"threshold_rad": 0.035},
    )
    # Threshold close to V_MAX=5 — let the policy use the full velocity budget
    # for fast tracking, only penalize getting close to / exceeding the cap.
    # Earlier threshold=1.5 made the policy too slow overall (capped peak vel
    # at 1.5 → 5 rad target took >3s). With threshold=4 + acceleration penalty,
    # policy can travel fast (peak ~4 rad/s) but with smooth acceleration.
    high_velocity    = RewardTermCfg(
        func=high_velocity_penalty, weight=-10.0,
        params={"threshold": 4.5},
    )
    settled_velocity = RewardTermCfg(func=settled_velocity_penalty, weight=-1.0)
    settled_action   = RewardTermCfg(func=settled_action_penalty, weight=-50.0)
    # Strong jerk-suppression — peak acc ≈ 180 rad/s² (drive saturation /
    # rotor inertia), so weight -5e-3 gives ~162 per-step penalty at the peak.
    # Forces smooth acceleration ramps at the start of each move instead of
    # bang-bang torque application. Combined with high_velocity (cap motion)
    # and target_pd_rate (cap setpoint rate), produces S-curve-like motion.
    acceleration     = RewardTermCfg(func=acceleration_penalty, weight=-5.0e-3)


@configclass
class TerminationsCfg:
    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    pass


# =============================================================================
# Environment config
# =============================================================================

@configclass
class PendulumPvtEnvCfg(ManagerBasedRLEnvCfg):
    """Pendulum PVT target tracking — ImplicitActuator (K=200, D=12), 1 kHz physics."""

    scene: PendulumPvtSceneCfg = PendulumPvtSceneCfg(num_envs=4096, env_spacing=2.0)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventsCfg = EventsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    seed = 123

    def __post_init__(self):
        # 1 kHz physics matches the EtherCAT cycle on real. decimation=20 →
        # policy at 50 Hz. ImplicitActuator handles the high physics rate
        # cheaply (no Python overhead per step).
        self.decimation = 20             # 1000 Hz physics → 50 Hz policy
        self.episode_length_s = 20.0     # ~3–6 target transitions per rollout
        self.sim.dt = 0.001
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material


@configclass
class PendulumPvtPlayEnvCfg(PendulumPvtEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        # Slower target switching for visual eval — see policy converge per target.
        self.events.resample_target_interval.interval_range_s = (15.0, 25.0)
