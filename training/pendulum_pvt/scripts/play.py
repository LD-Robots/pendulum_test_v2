#!/usr/bin/env python3
"""Play trained pendulum PVT policy (8-dim obs, 1-dim action).

Same flow as play_pd.py but uses the PVT env (ImplicitActuator K=200, D=12,
1 kHz physics). Exports ONNX with --export_onnx.

Usage:
    cd ~/IsaacLab
    ./isaaclab.sh -p .../pendulum_pvt/scripts/play.py --checkpoint /path/to/model.pt --export_onnx
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play pendulum PVT policy.")
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--export_onnx", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.importer.urdf")

import os
import torch
import gymnasium as gym
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

_this_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_this_dir, ".."))
import pendulum_rl  # noqa: F401

from pendulum_rl.env_cfg import PendulumPvtPlayEnvCfg
from pendulum_rl.agents.rsl_rl_ppo_cfg import PendulumPvtPPOCfg as PendulumPPOCfg


def main():
    env_cfg = PendulumPvtPlayEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs

    agent_cfg = PendulumPPOCfg()
    agent_cfg.experiment_name = "pendulum_pvt"

    env = gym.make("Isaac-Pendulum-PVT-Play-v0", cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    log_dir = os.path.dirname(args_cli.checkpoint)
    cfg_dict = agent_cfg.to_dict()
    for section in ("actor", "critic"):
        if section in cfg_dict and isinstance(cfg_dict[section], dict):
            for k in ("stochastic", "init_noise_std", "noise_std_type", "state_dependent_std"):
                cfg_dict[section].pop(k, None)
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device="cuda:0")
    runner.load(args_cli.checkpoint)

    obs_size = env.unwrapped.observation_manager.group_obs_dim["policy"][0]
    print(f"[INFO] Obs size: {obs_size} "
          f"(expected 8 — err/π, vel/16, sin(pos), cos(pos), 4×prev_action)")

    # Per-env DR snapshot
    robot = env.unwrapped.scene["robot"]
    actuator = robot.actuators["x6"]
    kp_per_env = actuator.stiffness[:, 0].cpu().numpy()
    kd_per_env = actuator.damping[:, 0].cpu().numpy()

    print(f"\nKp:  min={kp_per_env.min():.3f}  max={kp_per_env.max():.3f}  "
          f"mean={kp_per_env.mean():.3f}  std={kp_per_env.std():.3f}")
    print(f"Kd:  min={kd_per_env.min():.3f}  max={kd_per_env.max():.3f}  "
          f"mean={kd_per_env.mean():.3f}  std={kd_per_env.std():.3f}\n")

    if args_cli.export_onnx:
        onnx_path = os.path.join(log_dir, "policy.onnx")
        onnx_wrapper = runner.alg.actor.as_onnx(verbose=False).cpu().eval()
        dummy = torch.zeros(1, obs_size)
        torch.onnx.export(
            onnx_wrapper, dummy, onnx_path,
            input_names=["obs"], output_names=["actions"],
            opset_version=11,
            dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        )
        import onnx
        m = onnx.load(onnx_path)
        if m.ir_version > 9:
            m.ir_version = 9
            onnx.save(m, onnx_path)
        print(f"[INFO] Exported ONNX to: {onnx_path}")

    policy = runner.get_inference_policy(device="cuda:0")
    print(f"[INFO] Playing with {args_cli.num_envs} envs. Ctrl+C to stop.")

    obs, *_ = env.get_observations()
    step = 0

    from pendulum_rl.env_cfg import VELOCITY_LIMIT, ERR_SCALE
    import warp as _wp

    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
        clipped_actions = torch.clamp(actions, -agent_cfg.clip_actions, agent_cfg.clip_actions)

        if step % 50 == 0:
            o = obs["policy"].detach().cpu()
            if o.dim() > 1:
                o = o[0]
            o = o.numpy()
            a = actions.detach().cpu()
            a_clip = clipped_actions.detach().cpu()
            if a.dim() > 1:
                a = a[0]
            if a_clip.dim() > 1:
                a_clip = a_clip[0]
            a = a.numpy().flatten()
            a_clip = a_clip.numpy().flatten()

            # Live commanded target (no master-side slewing — policy slews itself)
            from pendulum_rl.env_cfg import _target_user

            _jp = env.unwrapped.scene["robot"].data.joint_pos
            _jv = env.unwrapped.scene["robot"].data.joint_vel
            if not isinstance(_jp, torch.Tensor):
                _jp = _wp.to_torch(_jp)
                _jv = _wp.to_torch(_jv)
            raw_pos = float(_jp[0, 0].cpu())
            raw_vel = float(_jv[0, 0].cpu())
            raw_target = (float(_target_user[0, 0].cpu())
                          if _target_user is not None else float("nan"))

            linear_err = float(o[0]) * ERR_SCALE
            vel_obs = float(o[1]) * VELOCITY_LIMIT
            pos_obs = raw_target - linear_err
            prev_action_newest = float(o[7])

            print(f"[step {step}] "
                  f"pos={pos_obs:+.3f}({raw_pos:+.3f}) "
                  f"target={raw_target:+.3f} "
                  f"vel={vel_obs:+.3f}({raw_vel:+.3f}) "
                  f"prev_a={prev_action_newest:+.3f} "
                  f"action_raw={a[0]:+.3f} action_clip={a_clip[0]:+.3f}")

        obs, rewards, dones, infos = env.step(clipped_actions)
        step += 1

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
