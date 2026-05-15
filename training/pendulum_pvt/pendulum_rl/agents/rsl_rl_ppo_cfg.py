"""RSL-RL PPO config for Pendulum PVT target tracking.

Tuned for the lower stiffness (K=200) and 1 kHz physics of the PVT env.
Compared to pd config (K=1800), the softer actuator settles slower per
action change → policy can explore with higher init_std without KL spikes,
and learning_rate can stay higher for longer.
"""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlMLPModelCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class PendulumPvtPPOCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    # Peak in 2026-05-14_13-48-08 run was iter 219 (+89 reward). Beyond ~400
    # the entropy bonus inflated std back to 0.5 and reward dove to -150.
    # 500 iter gives margin; save_interval=100 catches the convergence point.
    max_iterations = 5000
    save_interval = 100
    experiment_name = "pendulum_pvt"
    empirical_normalization = True
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    clip_actions = 3.0
    actor = RslRlMLPModelCfg(
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=True,
        # Reverted 0.8 → 0.5: previous attempt had std collapse from 0.78 → 0.04
        # in 150 iter — too aggressive start with strong penalties triggered the
        # adaptive KL scheduler to crush noise. Combined with reduced reward
        # weights, 0.5 should stay stable.
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=True,
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        # 0.005 matches the PD env config that converged stably. The 0.02
        # value in earlier PVT attempts inflated std back to 0.5 by iter 700
        # and the policy lost peak performance. 0.005 lets std collapse for
        # convergence; we accept early-iter local minimum risk in exchange
        # for stable plateau after convergence.
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        # Back to 0.02 (PD env value). Combined with entropy_coef=0.005 and
        # the physics-only reward stack, the adaptive scheduler keeps LR
        # at a stable level instead of bouncing between policy collapse and
        # exploration spikes.
        desired_kl=0.02,
        max_grad_norm=1.0,
    )
