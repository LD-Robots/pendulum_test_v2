# Pendulum PVT — RL Training

Isaac Lab training project for the X6 pendulum PVT (Mode 5) target-tracking
policy. **Not a ROS 2 package** — runs from a separate Isaac Lab install,
which is why `training/` lives at the workspace root (outside `src/`, so
`colcon` ignores it).

## Folder-per-training convention

Each training campaign is fully self-contained in its own folder under
`training/`:

```
training/
├── pendulum_pvt/          ← this one (PVT Mode 5 policy)
└── pendulum_<next>/       ← future trainings get their own folder
```

A training folder owns its env config, agent config, scripts, assets and
run logs — nothing is shared, so trainings never collide.

## Layout

```
training/pendulum_pvt/
├── pendulum_rl/
│   ├── __init__.py                  gym.register PVT tasks
│   ├── env_cfg.py                  env: ImplicitActuator (K=200, D=12),
│   │                                1 kHz physics, friction DR, v_max reward
│   └── agents/rsl_rl_ppo_cfg.py PPO hyperparameters
├── scripts/
│   ├── train.py                 training entry point
│   └── play.py                  playback + ONNX export
├── assets/
│   └── pendulum_isaac_swingup_compliant.urdf
│       (mesh refs → ../../../src/pendulum_description/meshes/)
└── logs/                            run outputs (gitignored)
```

## Architecture

- Policy at 50 Hz, physics at 1 kHz (decimation 20) — matches the EtherCAT
  cycle of the real drive.
- `ImplicitActuatorCfg(stiffness=200, damping=12)` replicates the firmware
  PVT formula `τ = Kp·err + Kd·verr` exactly.
- Action = target-offset: `target_pd = (2π/3)·action + commanded_target`.
- Observation (8-dim): `[err/π, vel/16, sin(pos), cos(pos), 4×prev_action]`.
- The exported `policy.onnx` bakes in the empirical normalizer — deploy-side
  code passes raw observations straight in.

## Train

```bash
cd ~/IsaacLab
./isaaclab.sh -p <pendulum_test_v2>/training/pendulum_pvt/scripts/train.py \
    --num_envs 4096 --headless
```

Logs → `training/pendulum_pvt/logs/pendulum_pvt/<timestamp>/`.

Warm-start from a checkpoint:

```bash
./isaaclab.sh -p .../train.py --num_envs 4096 --headless \
    --checkpoint .../logs/pendulum_pvt/<run>/model_NNNN.pt
```

## Play + export ONNX

```bash
./isaaclab.sh -p <pendulum_test_v2>/training/pendulum_pvt/scripts/play.py \
    --checkpoint .../logs/pendulum_pvt/<run>/model_NNNN.pt \
    --export_onnx
```

The `policy.onnx` lands next to the checkpoint. Deploy it with the
`pendulum_pvt_policy` ROS package.

## Adding a new training

1. `cp -r training/pendulum_pvt training/pendulum_<name>`
2. Rename the env/agent configs and update `pendulum_rl/__init__.py` task IDs.
3. Adjust the env config for the new task.
4. Run scripts from the new folder — paths are all relative, so it works
   without further edits.
