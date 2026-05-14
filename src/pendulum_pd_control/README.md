# pendulum_pd_control

Host-side PD loop controller for the LDR pendulum testbed (one myActuator X6
drive over EtherCAT).

The drive is booted into **CST — Cyclic Synchronous Torque (mode 10)** and never
switched at runtime. The PD law

```
tau = comp_sign * (tau_gravity + tau_inertia + tau_viscous) + Kp*(q_d - q) + Kd*(qd_d - qd)
```

runs in **software** at controller-manager rate and writes the `effort` command
interface. See `ldr-harambe-docs/tuning/pd_tuning_guide.html` §11 and §14 for the
control theory.

The package ships **three controller variants** so you can benchmark them
side-by-side, each with a sim and a real launch.

| Variant | Launch file | Controller | PD math runs in | Gravity FF |
|---------|-------------|------------|-----------------|------------|
| 1 — custom plugin | `pd_custom.launch.py` | `pendulum_pd_control/PendulumPDController` | C++ plugin, 1 kHz | yes |
| 2 — stock JTC | `pd_jtc.launch.py` | `joint_trajectory_controller` (effort + gains) | JTC internals, 1 kHz | no (inertia FF only) |
| 3 — setpoint node | `pd_effort_fwd.launch.py` | `JointGroupEffortController` + `pd_setpoint_node.py` | Python node, 200 Hz | yes |

On real hardware, every launch also spawns a `drive_status_broadcaster`
(`pendulum_pd_control/DriveStatusBroadcaster`) that publishes the drive's
temperature / voltage / encoder telemetry — see [Drive telemetry](#drive-telemetry).

---

## Build

```bash
cd ~/Documents/GitHub/pendulum_test_v2
colcon build --packages-select pendulum_pd_control pendulum_description
source install/setup.bash
```

`pendulum_description` is included because the real-hardware URDF
(`pendulum_ethercat.urdf.xacro`) carries the telemetry `<state_interface>`
entries the `drive_status_broadcaster` claims.

---

## Launching

Every launch file takes `use_sim`:

- `use_sim:=false` (default) — real hardware: parses `pendulum_ethercat.urdf.xacro`,
  starts `ros2_control_node` with the EtherCAT driver. Requires the EtherCAT
  pre-flight from `docs/ETHERCAT.md` (RT permissions, `ec_master` loaded, drive
  in PREOP).
- `use_sim:=true` — Gazebo: parses `pendulum.urdf.xacro`, starts `gz_sim` and the
  URDF-embedded `gz_ros2_control` plugin.

```bash
# Variant 1 — custom PD plugin
ros2 launch pendulum_pd_control pd_custom.launch.py use_sim:=true
ros2 launch pendulum_pd_control pd_custom.launch.py            # real

# Variant 2 — stock JTC
ros2 launch pendulum_pd_control pd_jtc.launch.py use_sim:=true
ros2 launch pendulum_pd_control pd_jtc.launch.py               # real

# Variant 3 — JointGroupEffortController + Python setpoint node
ros2 launch pendulum_pd_control pd_effort_fwd.launch.py use_sim:=true
ros2 launch pendulum_pd_control pd_effort_fwd.launch.py        # real
```

Each launch spawns `joint_state_broadcaster` first, then the variant's
controller (variant 3 additionally starts `pd_setpoint_node`). On real
hardware (`use_sim:=false`) the `drive_status_broadcaster` is also spawned
alongside; it is skipped in sim since Gazebo has no telemetry interfaces.

---

## Variant 1 — custom PD plugin

Controller name: `pendulum_pd_controller`. Modes: **FREE** (zero effort),
**TUNE** (feedforward only, no PD), **PD** (feedforward + PD, production).
Starts in FREE so a stale setpoint can't kick the joint.

### Send a setpoint

Publishing a setpoint automatically switches the controller into PD mode.

```bash
# Hold at 1.0 rad
ros2 topic pub --once /pendulum_pd_controller/setpoint \
  trajectory_msgs/msg/JointTrajectoryPoint \
  '{positions: [1.0], velocities: [0.0], accelerations: [0.0]}'

# Track a moving setpoint (position + velocity + acceleration feedforward)
ros2 topic pub -r 100 /pendulum_pd_controller/setpoint \
  trajectory_msgs/msg/JointTrajectoryPoint \
  '{positions: [1.5], velocities: [0.5], accelerations: [0.0]}'
```

### Mode services

```bash
# Snapshot current position and hold it (enters PD mode)
ros2 service call /pendulum_pd_controller/hold std_srvs/srv/Trigger

# Cut effort to zero (enters FREE mode)
ros2 service call /pendulum_pd_controller/free std_srvs/srv/Trigger
```

### Tune gains / toggle feedforward at runtime

Gains and FF flags are re-read every `update()`, so `ros2 param set` takes
effect immediately — no re-spawn needed.

```bash
ros2 param set /pendulum_pd_controller Kp 30.0
ros2 param set /pendulum_pd_controller Kd 2.5

# Compare "PD + gravity FF" vs "pure PD" — watch the steady-state error grow
ros2 param set /pendulum_pd_controller ff_gravity false
ros2 param set /pendulum_pd_controller ff_inertia false
ros2 param set /pendulum_pd_controller ff_viscous false
```

### Parameters (`config/pd_gains.yaml`)

| Param | Meaning | Default |
|-------|---------|---------|
| `joint` | joint name | `pendulum_joint` |
| `Kp` | proportional gain (N·m/rad) | `22.95` |
| `Kd` | derivative gain (N·m·s/rad) | `2.14` |
| `tau_limit` | symmetric effort clamp (N·m); `<=0` disables | `20.0` |
| `mgl` | gravity torque amplitude (N·m) — **placeholder, remeasure per rig** | `4.10` |
| `J` | rotating inertia (kg·m²) | `0.102` |
| `Fv` | viscous friction coeff (N·m·s/rad) | `0.05` |
| `comp_sign` | ±1 motor-mounting flip applied to feedforward | `1.0` |
| `ff_gravity` / `ff_inertia` / `ff_viscous` | feedforward term toggles | `true`/`true`/`false` |
| `hold_position` | default setpoint before any command arrives | `0.0` |

---

## Variant 2 — stock JTC

Controller name: `pendulum_jtc`. Uses the standard JTC interfaces — no custom
topics or services. Gains live in `config/controllers_jtc.yaml` under
`gains.pendulum_joint` (`p`, `d`, `i`, `ff_velocity_scale`,
`ff_acceleration_scale`). **No gravity feedforward** — `ff_acceleration_scale`
covers inertia only, so expect steady-state error under gravity.

```bash
# Move to 1.0 rad over 2 s
ros2 topic pub --once /pendulum_jtc/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [pendulum_joint],
    points: [{positions: [1.0], time_from_start: {sec: 2}}]}'

# Or via the action
ros2 action send_goal /pendulum_jtc/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [pendulum_joint],
     points: [{positions: [1.0], time_from_start: {sec: 2}}]}}'
```

Hold pose = send a single-point trajectory at the current position.

---

## Variant 3 — JointGroupEffortController + setpoint node

Controller name: `pendulum_effort_controller` (forwards raw torques). Node:
`pd_setpoint_node` computes the PD law in user-space at 200 Hz and publishes
to `/pendulum_effort_controller/commands`. Same topic/service surface as
variant 1.

```bash
ros2 topic pub --once /pd_setpoint_node/setpoint \
  trajectory_msgs/msg/JointTrajectoryPoint \
  '{positions: [1.0], velocities: [0.0], accelerations: [0.0]}'

ros2 service call /pd_setpoint_node/hold std_srvs/srv/Trigger
ros2 service call /pd_setpoint_node/free std_srvs/srv/Trigger

ros2 param set /pd_setpoint_node Kp 30.0
ros2 param set /pd_setpoint_node ff_gravity false
```

Parameters mirror variant 1 (loaded from the `pd_setpoint_node` block of
`config/pd_gains.yaml`), plus `command_topic` and `rate_hz`.

Not realtime-safe — use variant 1 if you need 1 kHz determinism.

---

## Drive telemetry

The 2026-04-24 myActuator firmware exposes a set of read-only telemetry
objects. `config/ethercat/icube_x6_drive.yaml` maps them all into a custom
TxPDO 0x1A02, and the `drive_status_broadcaster` (a ros2_control broadcaster
in this package) republishes them. It is spawned automatically alongside the
PD controller in **every** launch — but **real hardware only** (the Gazebo
URDF does not expose these interfaces, so the spawner is skipped when
`use_sim:=true`).

| Signal | Object | Unit | Notes |
|--------|--------|------|-------|
| `motor_temperature` | 0x2009 | °C | graded WARN/ERROR |
| `drive_temperature` | 0x200C | °C | MOSFET temp, graded WARN/ERROR |
| `bus_voltage` | 0x200A | V | graded ERROR if out of range |
| `first_encoder` | 0x200B | pulses | absolute, raw counts |
| `filtered_velocity` | 0x200D | rad/s | firmware-filtered |
| `filtered_torque` | 0x200F | Nm | firmware-filtered (rig-specific factor) |
| `position_demand` | 0x6062 | rad | CiA-402 demand value |
| `following_error` | 0x60F4 | rad | CiA-402 following error |

### Topics

```bash
# Thresholded health bundle — one DiagnosticStatus with all 8 as key/values
ros2 topic echo /diagnostics

# Per-signal std_msgs/Float64 — easy to rqt_plot / ros2 bag
ros2 topic echo /drive_status_broadcaster/bus_voltage
ros2 topic echo /drive_status_broadcaster/motor_temperature
ros2 topic echo /drive_status_broadcaster/following_error
# ... one topic per signal name above
```

### Thresholds

Graded in `/diagnostics` as OK / WARN / ERROR. Defaults in
`config/drive_status_broadcaster.yaml` (X6 is 48 V nominal — tune against the
real rig):

| Param | Default | Param | Default |
|-------|---------|-------|---------|
| `motor_temp_warn` | 70 °C | `motor_temp_error` | 90 °C |
| `drive_temp_warn` | 70 °C | `drive_temp_error` | 85 °C |
| `bus_voltage_min` | 40 V | `bus_voltage_max` | 54 V |

`publish_rate` (default 20 Hz) and the `signals` list are also configurable
there. Adjust live with `ros2 param set /drive_status_broadcaster <param>`.

### Requirements & fallback

- Requires the **2026-04-24 firmware** (ESI `docs/MT-Device_260424.xml`).
  Older firmware lacks objects 0x2009–0x200F.
- If the drive fails to reach EtherCAT OP with an IgH mapping error
  (`0x06040041` — "object cannot be mapped into the PDO"), the firmware
  rejects the custom TxPDO. Fall back by reverting the `tpdo:` block in
  `config/ethercat/icube_x6_drive.yaml` to the ESI's predefined `0x1A00`
  (status only) and removing the 8 telemetry `<state_interface>` lines from
  `pendulum_ethercat.urdf.xacro`.

---

## Verifying

```bash
ros2 control list_controllers          # controller is active
ros2 topic hz /joint_states            # ~1000 Hz real, ~handled by gz in sim
ros2 topic echo /joint_states          # position converges to your setpoint
```

Suggested bring-up order (lowest risk first):

1. **Sim, variant 2** — no plugin code, no hardware.
2. **Sim, variant 3** — exercise `hold` / `setpoint`.
3. **Sim, variant 1** — same, then toggle `ff_gravity:=false` and watch
   steady-state error grow (confirms the gravity FF path runs).
4. **Real, variant 1** — only after sim passes. Run the EtherCAT pre-flight
   from `docs/ETHERCAT.md`, launch with `use_sim:=false`, hold at 0, then
   command small steps (±0.1 rad). Confirm `drive_status_broadcaster` is
   active and `/diagnostics` shows a `pendulum_drive: telemetry` status at
   level OK (≈48 V, plausible °C).

---

## Caveats

- **`mgl` is a placeholder.** The default `4.10 N·m` comes from the doc's bench
  rig. This pendulum has a different mass distribution (1.839 kg rotating, with
  the 1.037 kg weight at the tip of a 700 mm profile). Measure `mgl` and `J` on
  this rig per `pd_tuning_guide.html` §02 / §09 before trusting the feedforward.
- **EtherCAT effort factors are rig-specific.** `config/ethercat/icube_x6_drive.yaml`
  uses the X6 drive #0 calibration from `docs/motor_specs.md`. Recalibrate
  `I_rated` / `K_t` for a different drive.
