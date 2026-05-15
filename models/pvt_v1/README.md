# pvt_v1 — PVT target-tracking policy

ONNX policy for the X6 pendulum in PVT (Mode 5) deployment. Trained in
Isaac Lab with the `pendulum_pvt` task.

## Provenance

| | |
|---|---|
| Source run | `pendulum_test/training/logs/pendulum_pvt/2026-05-14_18-48-34/` |
| Checkpoint | `model_43900.pt` (iteration 43900) |
| Exported   | 2026-05-15 via `play_pvt.py --export_onnx` |
| ONNX IR    | version 9 (compatible with bundled ONNX Runtime 1.17.0) |

## ONNX interface

| Tensor | Shape | Meaning |
|---|---|---|
| input `obs` | `[batch, 8]` | observation vector (see below) |
| output `actions` | `[batch, 1]` | raw policy action |

The empirical observation normalizer is **baked into the ONNX graph** —
pass raw observations straight in, do not normalize a second time.

## Observation (8-dim)

```
[0]    (target − pos) / π      linear error, NOT wrapped
[1]    vel / 16.0              normalized angular velocity
[2]    sin(pos)                gravity context
[3]    cos(pos)                gravity context
[4..7] prev_action history     last 4 raw actions, oldest at [4], newest at [7]
```

## Action mapping

```
raw_action ∈ [−3, 3]   (clipped to PPO clip_actions = 3.0)
target_pd = (2π/3) · raw_action + commanded_target
```

The policy outputs an offset around the commanded target; the deploy node
adds it to the user target and publishes the resulting `target_pd`.

## Training config (snapshot)

- Actuator: `ImplicitActuatorCfg(stiffness=200, damping=12, effort_limit=20)`
  — replicates the firmware PVT law `τ = Kp·err + Kd·verr`.
- Physics 1 kHz, policy 50 Hz (decimation 20).
- `V_MAX = 5.0` rad/s soft-enforced via the `high_velocity` / `target_pd_rate`
  rewards (the env later moved to 5.5 — this model is the 5.0 generation).
- Friction domain randomization `(0.5, 2.5)` Nm — covers the measured X6
  stiction (~2 Nm break-away).
- Gain DR ±25 % around nominal: stiffness `(150, 250)`, damping `(8, 16)`.
- Target range `[0, 2π]`, resampled every 3–6 s; no master-side slewing in
  training — the policy learns to interpolate on its own.

## Deploy

```bash
ros2 launch pendulum_pvt_policy pvt_policy.launch.py \
    onnx_path:=<pendulum_test_v2>/models/pvt_v1/policy.onnx
```

On the drive, set the PVT gains to match training before running:

```bash
ethercat download -p 0 0x2000 0 200000 --type int32   # PVT_KP = 200
ethercat download -p 0 0x2001 0 12000  --type int32   # PVT_KD = 12
```
