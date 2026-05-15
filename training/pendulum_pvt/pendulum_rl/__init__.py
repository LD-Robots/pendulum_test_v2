"""Pendulum PVT (Mode 5) target-tracking — RL task for Isaac Lab.

Registers the PVT training/play gym tasks. Imported by the train/play
scripts under training/scripts/.
"""

import gymnasium as gym

from .env_cfg import PendulumPvtEnvCfg, PendulumPvtPlayEnvCfg  # noqa: F401

# PVT-targeted policy — ImplicitActuator (K=200, D=12), 1 kHz physics,
# v_max soft-enforced via reward, friction DR for sim2real.
gym.register(
    id="Isaac-Pendulum-PVT-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:PendulumPvtEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:PendulumPvtPPOCfg",
    },
)

gym.register(
    id="Isaac-Pendulum-PVT-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:PendulumPvtPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:PendulumPvtPPOCfg",
    },
)
