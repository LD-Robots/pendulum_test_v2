# pendulum_pvt_policy

ONNX inference node for the X6 pendulum PVT (Mode 5) RL policy.

Runs the trained policy and feeds the `PendulumPVTController` (from
`pendulum_pvt_control`) via its `~/setpoint` topic. The drive firmware does
the 1 kHz PD; this node only decides the position target.

## Architecture

```
/pendulum/target  (user goal, Float64)
        │
        ▼
pendulum_pvt_policy node
  ├── inference timer  50 Hz : ONNX → target_pd goal
  └── output timer    200 Hz : slew toward goal, publish
        │
        ▼
/pendulum_pvt_controller/setpoint  (JointTrajectoryPoint)
        │
        ▼
PendulumPVTController → drive (1 kHz PVT torque)
```

- **Inference 50 Hz** matches the training cadence.
- **Output 200 Hz** decouples publishing from inference so the drive sees
  small frequent setpoint steps instead of 50 Hz pulses (audible noise).
- **Slew limiter** caps `|Δtarget_pd|/dt` at `target_pd_slew_rate` (5.5 rad/s).

## Observation / action

8-dim obs `[err/π, vel/16, sin(pos), cos(pos), 4×prev_action]`;
1-dim action; `target_pd = (2π/3)·action + user_target`. See
`models/pvt_v1/README.md` for the full spec.

## Gains config

`config/pvt_gains_policy.yaml` holds the drive Kp/Kd **matched to the trained
policy** (the gains are a property of the model, not the controller — see
`models/pvt_vN/README.md`). Feed-forward is off — the policy compensates
gravity/inertia implicitly.

`policy.launch.py` leaves `pvt.launch.py` untouched: the controller comes up
with its own `pvt_gains.yaml`, then this launch runs `ros2 param load
/pendulum_pvt_controller pvt_gains_policy.yaml` once the controller is up.
The controller re-reads its parameters every `update()`, so the new gains
take effect on the next cycle — no restart needed.

## Dependencies

ONNX Runtime — fetched into `<workspace>/third_party/onnxruntime` by
`scripts/init_onnxruntime.sh`. Run that before building this package.

## Build

```bash
cd <workspace>
./scripts/init_onnxruntime.sh
colcon build --packages-select pendulum_pvt_policy --symlink-install
```

## Run

`policy.launch.py` is a one-shot bring-up — it includes the PVT controller
stack (with the policy-matched gains) AND the inference node:

```bash
# real hardware
ros2 launch pendulum_pvt_policy policy.launch.py \
    onnx_path:=$PWD/models/pvt_v1/policy.onnx

# Gazebo sim
ros2 launch pendulum_pvt_policy policy.launch.py \
    onnx_path:=$PWD/models/pvt_v1/policy.onnx use_sim:=true

# send a target to track
ros2 topic pub --once /pendulum/target std_msgs/msg/Float64 "{data: 1.5}"
```

## Live params

```bash
ros2 param set /pendulum_pvt_policy policy_enabled false       # bypass ONNX
ros2 param set /pendulum_pvt_policy target_pd_slew_rate 3.0    # gentler slew
```

## Debug

```bash
ros2 topic echo /pendulum/target_pd          # published setpoint stream (50→200 Hz)
ros2 topic echo /pendulum/fall_latched       # fall e-stop flag
```
