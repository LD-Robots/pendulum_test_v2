# pendulum_pvt_control

PVT / MIT-mode (CiA-402 **mode 5**) impedance controller for the LDR pendulum
testbed (one myActuator X6 drive over EtherCAT).

Where `pendulum_pd_control` boots the drive into **CST (mode 10)** and runs the
PD law in software, this package boots it into **PVT (mode 5)** and the **drive**
runs the law

```
tau = Kp*(q_d - q) + Kd*(qd_d - qd) + tau_ff
```

internally at 1 kHz. The host only streams the payload — target position,
target velocity, the torque feedforward `tau_ff`, and the `Kp`/`Kd` gains — every
cycle over RxPDO `0x1601`. This mirrors Isaac Lab's `ImplicitActuatorCfg`. See
`ldr-harambe-docs/hardware/ethercat_control_guide.html` §23 / §28 for the drive
side.

A single controller, `pendulum_pvt_control/PendulumPVTController`, covers both
targets via the `drive_side_pd` parameter:

| `drive_side_pd` | Target | Command interfaces claimed | PD law runs in |
|-----------------|--------|----------------------------|----------------|
| `true` (default) | real hardware | `position, velocity, effort, kp, kd` | the X6 drive (firmware, 1 kHz) |
| `false` | Gazebo sim | `effort` only | the controller (software) |

In both cases the host adds gravity / inertia / viscous feedforward through the
torque-feedforward channel (`effort`), gated by the `ff_*` flags.

---

## Build

```bash
cd ~/Documents/GitHub/pendulum_test_v2
colcon build --packages-select pendulum_pvt_control pendulum_description pendulum_gazebo pendulum_pd_control
source install/setup.bash
```

`pendulum_description` carries the `pvt_mode` xacro block that exposes the
`position`/`velocity`/`kp`/`kd` command interfaces; `pendulum_gazebo` provides
the reusable sim bringup (`sim_bringup.launch.py`); `pendulum_pd_control`
provides the `DriveStatusBroadcaster` reused for drive telemetry.

---

## Launching

`pvt.launch.py` takes `use_sim`:

- `use_sim:=false` (default) — real hardware: parses `pendulum_ethercat.urdf.xacro`
  with `mode_of_operation:=5 pvt_mode:=true`, starts `ros2_control_node` with the
  EtherCAT driver and the `icube_x6_drive_pvt.yaml` slave config. Requires the
  EtherCAT pre-flight from `docs/ETHERCAT.md` (RT permissions, `ec_master`
  loaded, drive in PREOP).
- `use_sim:=true` — Gazebo: parses `pendulum.urdf.xacro`, includes
  `pendulum_gazebo/sim_bringup.launch.py`, layers `pvt_gains_sim.yaml` to flip
  `drive_side_pd` to false.

```bash
ros2 launch pendulum_pvt_control pvt.launch.py use_sim:=true
ros2 launch pendulum_pvt_control pvt.launch.py            # real
```

Both paths spawn `joint_state_broadcaster` first, then
`pendulum_pvt_controller`. On real hardware (`use_sim:=false`) a
`drive_status_broadcaster` is also spawned — see [Drive telemetry](#drive-telemetry).

---

## Using the controller

Controller name: `pendulum_pvt_controller`. Modes: **FREE** (neutral output —
zero drive torque) and **PVT** (track the streaming setpoint). Starts in FREE so
a stale setpoint can't kick the joint.

### Send a setpoint

The controller takes a **streaming** setpoint on `~/setpoint` — one
`trajectory_msgs/JointTrajectoryPoint` at a time (index `[0]`, single joint).
Publishing any setpoint automatically switches the controller into PVT mode.

```bash
# Step to 1.0 rad (no velocity profile — abrupt, expect overshoot)
ros2 topic pub --once /pendulum_pvt_controller/setpoint \
  trajectory_msgs/msg/JointTrajectoryPoint \
  '{positions: [1.0], velocities: [0.0], accelerations: [0.0]}'
```

The controller keeps the last setpoint in its buffer, so a single `--once`
publish is held until the next one arrives.

#### What the three fields do

| Field | Symbol | Role | Sent to drive? |
|-------|--------|------|----------------|
| `positions[0]` | `q_d` | proportional term `Kp*(q_d - q)` | yes — `0x607A` target_position |
| `velocities[0]` | `qd_d` | derivative term `Kd*(qd_d - qd)` | yes — `0x60FF` target_velocity |
| `accelerations[0]` | `qdd_d` | inertia feedforward `J*qdd_d` only | no — folded into `tau_ff` |

- **`positions`** — where you want the joint to be. The dominant term.
- **`velocities`** — the reference rate for the `Kd` term. For a **static hold**
  set it to `0` (then `Kd*(0 - qd)` is pure damping that resists motion). When
  **tracking a moving trajectory** set it to the trajectory's actual velocity,
  or the `Kd` term fights the motion you intend.
- **`accelerations`** — feeds *only* the inertia feedforward
  (`tau_J = J * qdd_d`, gated by `ff_inertia`); it is not a PD term and is not a
  separate drive channel. Leave it `0` for a static hold.

Omitting a field is allowed: empty `positions` falls back to `hold_position`;
empty `velocities` / `accelerations` default to `0.0`.

### Smooth point-to-point moves — `pvt_goto`

For an actual move from A to B over a time `T`, stream a smooth profile rather
than stepping. `pvt_goto.py` generates a **quintic** trajectory (zero velocity
and acceleration at both endpoints) and streams mutually-consistent
position/velocity/acceleration points:

```bash
# move to 1.5 rad over 2.0 s (start = current position, default 200 Hz stream)
ros2 run pendulum_pvt_control pvt_goto.py 1.5 2.0

# slower move, finer stream rate
ros2 run pendulum_pvt_control pvt_goto.py 0.0 4.0 500
```

`ros2 run pendulum_pvt_control pvt_goto.py <goal_rad> <duration_s> [rate_hz]`

- Point A is read once from `/joint_states` — the move starts wherever the joint
  currently is. Point B and `T` are the arguments.
- When the stream ends the controller holds the final point (B, zero velocity)
  automatically — no need to keep publishing.
- `rate_hz` (default 200) only sets stream granularity; the controller still
  runs its loop at 1 kHz and holds each setpoint between updates. 500–1000 Hz is
  smoother.

### Offline simulation & validation — `pvt_sim_gui`

Before touching hardware, `pvt_sim_gui.py` lets you *see* what a given set of
trajectory parameters and gains produces. It is a self-contained desktop app
(numpy + matplotlib + tkinter, **no ROS, no hardware**): it embeds the same
quintic generator as `pvt_goto`, runs a forward physics simulation of the
pendulum under the drive's law `tau = Kp*(q_d-q) + Kd*(qd_d-qd) + tau_ff`, and
plots the commanded setpoint against the predicted response.

```bash
python3 src/pendulum_pvt_control/scripts/pvt_sim_gui.py   # standalone
ros2 run pendulum_pvt_control pvt_sim_gui.py              # after colcon build
```

Edit the trajectory (`q0`, `goal`, `duration`, stream `rate`), the gains
(`Kp`, `Kd`, `tau_limit`), the feedforward model (`mgl`, `J`, `Fv` + the
`ff_*` toggles) and the plant physics, then hit **Run**. The four stacked plots
show position, velocity, the torque breakdown (`Kp`/`Kd`/`tau_ff`/total, with
the `tau_limit` and `0x6072` ceiling lines) and tracking error; the metrics
panel reports peak speed, steady-state error, overshoot, settling time, peak
torque and whether the ceiling / FF clamp were hit. The gains are pre-seeded
from `config/pvt_gains.yaml`; the drive loop is modelled at 1 kHz with the
streamed setpoint zero-order-held, so a low stream rate visibly tracks worse.

Things to try: turn off `ff_gravity` and watch the steady-state droop appear;
on a fast move (`duration` ~0.3 s) drop `Kd` to ~0.2 and watch the
overshoot/ringing; run a fast move at `rate` 10 Hz then 1000 Hz (tick *keep
previous run as ghost*) to compare the stair-step tracking. The gentle 2 s
default move is deliberately well-damped — push `duration` down to excite the
gain/rate effects.

### Mode services

```bash
# Snapshot current position and hold it (enters PVT mode)
ros2 service call /pendulum_pvt_controller/hold std_srvs/srv/Trigger

# Drop to zero drive torque (enters FREE mode)
ros2 service call /pendulum_pvt_controller/free std_srvs/srv/Trigger
```

### Tune gains / toggle feedforward at runtime

`Kp`, `Kd` and the FF flags are re-read every `update()`, so `ros2 param set`
takes effect immediately — no re-spawn needed. On real hardware the new `Kp`/`Kd`
are streamed to the drive's `PVT_KP`/`PVT_KD` objects on the next cycle.

```bash
ros2 param set /pendulum_pvt_controller Kp 30.0
ros2 param set /pendulum_pvt_controller Kd 2.5

# Compare "with gravity FF" vs "without" — watch the steady-state error grow
ros2 param set /pendulum_pvt_controller ff_gravity false
```

### Parameters (`config/pvt_gains.yaml`)

| Param | Meaning | Default |
|-------|---------|---------|
| `joint` | joint name | `pendulum_joint` |
| `drive_side_pd` | `true` = drive runs the PD law (real); `false` = software (sim) | `true` |
| `Kp` | stiffness (N·m/rad) — streamed to `PVT_KP` | `22.95` |
| `Kd` | damping (N·m·s/rad) — streamed to `PVT_KD` | `2.14` |
| `tau_limit` | symmetric clamp on the host feedforward (N·m); `<=0` disables | `20.0` |
| `mgl` | gravity torque amplitude (N·m) — **placeholder, remeasure per rig** | `4.10` |
| `J` | rotating inertia (kg·m²) | `0.102` |
| `Fv` | viscous friction coeff (N·m·s/rad) | `0.05` |
| `comp_sign` | ±1 motor-mounting flip applied to feedforward (sim-only — see Caveats) | `1.0` |
| `ff_gravity` / `ff_inertia` / `ff_viscous` | feedforward term toggles | `true`/`true`/`false` |
| `hold_position` | default setpoint before any command arrives | `0.0` |

Changing `drive_side_pd` at runtime has no effect — the claimed command-interface
set is fixed when the controller is configured. Re-spawn to switch.

---

## Drive telemetry

On real hardware the launch also spawns a `drive_status_broadcaster`
(`pendulum_pd_control/DriveStatusBroadcaster`, reused) that republishes the
drive's TxPDO `0x1A02` telemetry. **Real hardware only** — the Gazebo URDF has
no telemetry interfaces, so the spawner is skipped when `use_sim:=true`.

| Topic | Type | Source | Notes |
|-------|------|--------|-------|
| `/drive_status_broadcaster/error_code` | `std_msgs/Float64` | CiA-402 `0x603F` | `0` = no fault; non-zero = fault code (e.g. `0x4110` = temperature) |
| `/drive_status_broadcaster/bus_voltage` | `std_msgs/Float64` | `0x200A` | V — graded in `/diagnostics` |
| `/drive_status_broadcaster/motor_temperature` | `std_msgs/Float64` | `0x2009` | °C — graded in `/diagnostics` |
| `/drive_status_broadcaster/drive_temperature` | `std_msgs/Float64` | `0x200C` | °C — graded in `/diagnostics` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | all four | thresholded health bundle (`error_code` passed through ungraded) |

```bash
# watch the fault code while reproducing a cut-out
ros2 topic echo /drive_status_broadcaster/error_code
ros2 topic echo /drive_status_broadcaster/bus_voltage
ros2 topic echo /diagnostics
```

`error_code` is mapped to a named state interface only on the PVT path —
[icube_x6_drive_pvt.yaml](config/ethercat/icube_x6_drive_pvt.yaml) maps `0x603F`
to `error_code` and [pendulum_ethercat.urdf.xacro] declares it under
`pvt_mode:=true`. The CiA-402 plugin still reads `0x603F` internally for its own
fault state machine — naming it is purely additive. Thresholds and
`publish_rate` (50 Hz) live in
[config/drive_status_broadcaster.yaml](config/drive_status_broadcaster.yaml).

---

## Verifying

```bash
ros2 control list_controllers          # pendulum_pvt_controller is active
ros2 param get /pendulum_pvt_controller drive_side_pd   # false in sim, true on real
ros2 control list_hardware_interfaces  # sim: only .../effort claimed
                                       # real: position/velocity/effort/kp/kd claimed
ros2 topic hz /joint_states            # ~1000 Hz real; handled by gz in sim
ros2 topic echo /joint_states          # position converges to your setpoint
```

Suggested bring-up order (lowest risk first):

1. **Sim** — confirm the software PD law tracks, gain ramping works, and the
   mode services behave. Toggle `ff_gravity:=false` and watch the steady-state
   error grow (confirms the gravity FF path runs).
2. **Real, near-zero gains** — run the EtherCAT pre-flight from
   `docs/ETHERCAT.md`, launch with `use_sim:=false`, confirm the drive accepts
   mode 5 and reaches OP. With `Kp`/`Kd` low the joint should be nearly limp;
   raise `Kp` slightly and confirm it starts to hold.
3. **Real, production gains** — `~/hold` at the current position, then command
   small steps (±0.1 rad). Ramp `Kd` first, then `Kp`, watching for oscillation.
   `~/free` drops to zero drive torque if anything misbehaves.

If the drive refuses PREOP→SAFEOP with AL status `0x001E` ("Invalid input
configuration"), the RxPDO `0x1601` layout does not match the firmware ESI —
recheck the channel order / byte count in `config/ethercat/icube_x6_drive_pvt.yaml`
against `docs/MT-Device_260424.xml`.

---

## Caveats

- **`tau_limit` clamps the feedforward only on real hardware.** In
  `drive_side_pd:true` the drive adds `Kp*(q_d - q) + Kd*(qd_d - qd)` on top of
  the streamed `tau_ff`, and that term cannot be bounded from the host. The hard
  torque ceiling is the **`0x6072` max_torque SDO** in
  `config/ethercat/icube_x6_drive_pvt.yaml` (default 800 = 80 % rated current).
- **`comp_sign` is effective in sim only.** In `drive_side_pd:true` it negates
  the host feedforward but *not* the drive's internal `Kp`/`Kd` term, which runs
  in the encoder's own sign. A flipped motor mounting must be corrected with
  **negative EtherCAT factors** in the slave config (`docs/motor_specs.md`), not
  `comp_sign`. The controller logs a warning if `comp_sign != 1.0` while
  `drive_side_pd:true`.
- **`PVT_KP` / `PVT_KD` scaling.** The drive applies the written gain value
  `× 0.001`. `icube_x6_drive_pvt.yaml` bakes the `×1000` into the channel
  `factor`, so `Kp`/`Kd` are commanded in engineering units (N·m/rad,
  N·m·s/rad) — do not pre-scale them.
- **`mgl` is a placeholder.** The default `4.10 N·m` comes from the doc's bench
  rig. Measure `mgl` and `J` on this rig before trusting the feedforward.
- **EtherCAT effort factors are rig-specific.** `icube_x6_drive_pvt.yaml` uses
  the X6 drive #0 calibration from `docs/motor_specs.md`. Recalibrate
  `I_rated` / `K_t` for a different drive.
