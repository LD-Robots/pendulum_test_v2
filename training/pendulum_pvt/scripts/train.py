#!/usr/bin/env python3
"""Train pendulum target-tracking policy for PVT deployment.

Mirror of train_pd.py but targeted at PVT mode (Mode 5):
  - ImplicitActuator with stiffness=200, damping=12 (matches real-drive PVT)
  - 1 kHz physics, 50 Hz policy (decimation=20)
  - Friction DR (0.5, 2.5) Nm covers measured X6 stiction
  - v_max=5 rad/s soft-enforced via reward (hinge penalty above threshold)

Usage:
    cd ~/IsaacLab
    ./isaaclab.sh -p ~/Documents/GitHub/pendulum_test/training/pendulum_pvt/scripts/train.py \\
        --num_envs 4096 --headless
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train pendulum policy for PVT deployment.")
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--checkpoint", type=str, default=None)
parser.add_argument("--seed", type=int, default=42)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import time
from datetime import datetime

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

_this_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_this_dir, ".."))
import pendulum_rl  # noqa: F401

from pendulum_rl.env_cfg import PendulumPvtEnvCfg
from pendulum_rl.agents.rsl_rl_ppo_cfg import PendulumPvtPPOCfg as PendulumPPOCfg

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def main():
    env_cfg = PendulumPvtEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    agent_cfg = PendulumPPOCfg()
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    agent_cfg.seed = args_cli.seed
    agent_cfg.experiment_name = "pendulum_pvt"

    log_root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "logs", agent_cfg.experiment_name)
    log_dir = os.path.join(log_root, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    print(f"[INFO] Logging to: {log_dir}")

    env = gym.make("Isaac-Pendulum-PVT-v0", cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    cfg_dict = agent_cfg.to_dict()
    for section in ("actor", "critic"):
        if section in cfg_dict and isinstance(cfg_dict[section], dict):
            for k in ("stochastic", "init_noise_std", "noise_std_type", "state_dependent_std"):
                cfg_dict[section].pop(k, None)
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device=args_cli.device)

    if args_cli.checkpoint:
        runner.load(args_cli.checkpoint)

    start = time.time()
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
    elapsed = time.time() - start
    print(f"\nTraining complete! {elapsed:.1f}s")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
