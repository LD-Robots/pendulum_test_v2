# pendulum_safety

Centralised safety enforcement for the LDR pendulum testbed. Caps **position**,
**velocity**, **effort** and **temperature**, and drives a configurable,
manually-reset **emergency stop** — shared by all three control paths
(`pendulum_pd_control`, `pendulum_pvt_control`, `pendulum_pvt_policy`).

Before this package each controller carried its own `clamp_symmetric` +
`tau_limit`, `DriveStatusBroadcaster` knew temperature thresholds but only
published `/diagnostics` and never acted, and the policy node had an ad-hoc fall
e-stop. Limits were scattered and inconsistent. `pendulum_safety` makes them one
library, one config file and one supervisor.

The package has two halves:

| Half | What it is | Runs where |
|------|-----------|------------|
| **Limiter library** | `libpendulum_safety.so` — realtime-safe clamp / rate-limit / thermal primitives | linked into the PD, PVT and policy processes |
| **Supervisor node** | `pendulum_safety_supervisor` — all breach detection + the e-stop | a standalone node, one per system |

---

## Build

```bash
cd ~/Documents/GitHub/pendulum_test_v2
colcon build --packages-select pendulum_safety
source install/setup.bash
```

`pendulum_safety` has no dependency on the control packages, but they depend on
it — `colcon build` topologically sorts it first. It builds standalone (no
hardware, no Gazebo).

---

## The limiter library

A realtime-safe C++ library — every function reachable from a controller
`update()` is allocation-free, lock-free and log-free.

| Header | Provides |
|--------|----------|
| [`safety_limits.hpp`](include/pendulum_safety/safety_limits.hpp) | `SafetyLimits` POD struct, `BreachReason` / `EstopAction` enums |
| [`clamp.hpp`](include/pendulum_safety/clamp.hpp) | `clampPosition` / `clampVelocity` / `clampEffort` (header-only `inline`) |
| [`rate_limiter.hpp`](include/pendulum_safety/rate_limiter.hpp) | `RateLimiter` — stateful slew-rate **and** acceleration limiter |
| [`sustained_effort_monitor.hpp`](include/pendulum_safety/sustained_effort_monitor.hpp) | `SustainedEffortMonitor` — leaky-bucket time-above-threshold detector |
| [`thermal.hpp`](include/pendulum_safety/thermal.hpp) | `kpScaleForTemp` / `kpScaleCombined` — thermal `Kp`-derating curve |
| [`breach.hpp`](include/pendulum_safety/breach.hpp) | `checkPosition` / `checkVelocity` / `checkMotorTemp` / … breach predicates |
| [`param_loader.hpp`](include/pendulum_safety/param_loader.hpp) | `loadSafetyLimits()` — declare + read `SafetyLimits` from ROS params |
| [`estop_subscriber.hpp`](include/pendulum_safety/estop_subscriber.hpp) | `EstopSubscriber` — consumer-side helper for the e-stop signal |

`clampSymmetric` keeps the `limit <= 0 → disabled` semantics of the old
`clamp_symmetric` helpers, so swapping them in is behaviour-preserving.

---

## The supervisor node

`pendulum_safety_supervisor` — a plain `rclcpp::Node`. A watchdog timer
(`watchdog_rate_hz`, default 200 Hz) runs every breach predicate on the latest
telemetry, latches the **first** breach, and drives the e-stop.

### Subscribes

| Topic | Type | Used for |
|-------|------|----------|
| `/joint_states` | `sensor_msgs/JointState` | position + velocity, **and** effort — the drive's actual torque (EtherCAT `0x6077`) on real hardware, the gz effort in sim — for the sustained-effort monitor |
| `/drive_status_broadcaster/motor_temperature` | `std_msgs/Float64` | motor over-temp (real only) |
| `/drive_status_broadcaster/drive_temperature` | `std_msgs/Float64` | drive over-temp (real only) |
| `/drive_status_broadcaster/bus_voltage` | `std_msgs/Float64` | bus-voltage window (real only) |

### Publishes

| Topic | Type | Notes |
|-------|------|-------|
| `/pendulum/safety/estop_state` | `std_msgs/Int8` | **latched** — `0` clear / `1` e-stop+FREE / `2` e-stop+HOLD |
| `/pendulum/safety/kp_scale` | `std_msgs/Float64` | **latched** — thermal `Kp` multiplier `[kp_scale_floor, 1.0]` |
| `/pendulum/safety/breach_reason` | `std_msgs/String` | **latched** — operator-facing latched breach name |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | live telemetry + state, ~10 Hz |

### Services

```bash
# Manual e-stop — latches BreachReason::MANUAL immediately
ros2 service call /pendulum_safety_supervisor/estop std_srvs/srv/Trigger

# Manual reset — REFUSED (success=false) while any breach is still active
ros2 service call /pendulum_safety_supervisor/reset std_srvs/srv/Trigger
```

The latch never auto-clears — once tripped it stays tripped until `~/reset`
succeeds, and reset succeeds only when every predicate is back in range. This is
deliberate: the drive cannot silently re-energise without an operator.

### Per-trigger FREE / HOLD

Each breach category resolves to an action — **FREE** (zero torque, the joint
coasts) or **HOLD** (lock at the position latched when the e-stop fired) — set
in [`config/safety_limits.yaml`](config/safety_limits.yaml):

```yaml
    action.temperature: free        # over-temp / bus voltage
    action.overspeed: free
    action.position: hold           # out-of-range position
    action.manual: free
    action.stale_joint_state: hold  # lost /joint_states
    action.sustained_effort: free
```

---

## The e-stop signalling contract

The supervisor → controller link is two **latched** `std_msgs` topics — no
custom messages. Controllers consume them via `EstopSubscriber`, which stores
the values in lock-free atomics so `update()` reads them with no blocking.

- `/pendulum/safety/estop_state` (`Int8`): `0` = clear, `1` = e-stop + FREE,
  `2` = e-stop + HOLD.
- `/pendulum/safety/kp_scale` (`Float64`): `Kp` is multiplied by this every
  cycle — `1.0` when cool or when no supervisor is running.

**Reaction time < 10 ms:** a controller reacts inside its own 1 kHz `update()`
loop — the only latency is topic delivery plus one control cycle. There is no
service round-trip in the e-stop path.

**No supervisor running?** The latched topics have no publisher, the atomics
keep their defaults (no e-stop, `kp_scale = 1.0`), and every control path
behaves exactly as it did before this package existed.

---

## How the control paths consume it

All three paths load `SafetyLimits` from the `safety.*` parameters
(`loadSafetyLimits`) and apply the library every cycle:

| Path | Position | Velocity | Effort | Slew / accel | Kp derate | E-stop reaction |
|------|----------|----------|--------|--------------|-----------|-----------------|
| **PD controller** | `clampPosition` | `clampVelocity` | `clampEffort` | — | yes | FREE → 0 τ · HOLD → active PD hold |
| **PVT controller** | `clampPosition` | `clampVelocity` | `clampEffort` | `RateLimiter` | yes | FREE → `write_free_outputs()` · HOLD → static PVT hold |
| **policy node** | `clampPosition` on the published setpoint | overspeed → fall-latch | — | `RateLimiter` | — | e-stop latches the fall-hold |

An e-stop **HOLD** overrides the controller's own FREE mode — safety must hold
the joint regardless of what the operator asked for.

---

## Configuration — one source of truth

[`config/safety_limits.yaml`](config/safety_limits.yaml) is the **only** place
limit numbers are edited. Its `/**:` wildcard block means the same file applies
to whatever node it is passed to — the supervisor, all three controllers (via an
extra `--param-file` on the spawner) and the policy node all read the same
`safety.*` values.

| Param (`safety.*`) | Meaning | Default |
|--------------------|---------|---------|
| `position_limits_enable` | enable the position clamp | `true` |
| `position_min` / `position_max` | joint bounds (rad) — URDF `[0, 2π]` | `0.0` / `6.2832` |
| `effort_limit` | symmetric torque cap (N·m); `<=0` disables | `20.0` |
| `sustained_effort_threshold` | "high effort" level (N·m) | `16.0` |
| `sustained_effort_window_sec` | above-threshold time that trips an e-stop | `1.0` |
| `velocity_limit` | symmetric speed cap (rad/s) | `16.02` |
| `slew_rate_limit` | position-command slew (rad/s) | `5.5` |
| `acceleration_limit` | position-command acceleration cap (rad/s²) | `60.0` |
| `motor_temp_warn` / `motor_temp_error` | derate-start / e-stop temps (°C) | `70` / `90` |
| `drive_temp_warn` / `drive_temp_error` | derate-start / e-stop temps (°C) | `70` / `85` |
| `bus_voltage_min` / `bus_voltage_max` | bus-voltage window (V) | `40` / `54` |
| `kp_scale_floor` | `Kp` multiplier at/above the error temp | `0.3` |
| `joint_state_timeout_sec` | `/joint_states` freshness watchdog (s) | `0.05` |

A second, node-name-targeted block holds the supervisor's own topic names,
`watchdog_rate_hz` and the per-trigger `action.*` map.

---

## Launching

```bash
ros2 launch pendulum_safety safety.launch.py             # real hardware
ros2 launch pendulum_safety safety.launch.py use_sim:=true   # simulation
```

`safety.launch.py` is also pulled in automatically by `pvt.launch.py` and
`pd_custom.launch.py` (and, transitively, `policy.launch.py`) — so the
supervisor comes up with the control stack and the controllers receive
`safety_limits.yaml` on their spawners.

In **simulation** (`use_sim:=true`) there are no temperature interfaces, so the
launch sets `monitor_temperature:=false` (the supervisor skips the temperature
subscriptions and checks) and points the effort source at `/joint_states`. The
supervisor still monitors position, velocity, sustained effort and joint-state
freshness.

---

## Verifying

```bash
# supervisor up, initial state clear
ros2 topic echo --once /pendulum/safety/estop_state    # data: 0

# manual e-stop -> state 1, breach MANUAL
ros2 service call /pendulum_safety_supervisor/estop std_srvs/srv/Trigger
ros2 topic echo --once /pendulum/safety/breach_reason  # data: MANUAL

# overspeed: publish joint states past velocity_limit -> state latches, OVERSPEED
ros2 topic pub -r 50 /joint_states sensor_msgs/msg/JointState \
  "{name: [pendulum_joint], velocity: [100.0]}"

# reset is refused while the breach is still active, succeeds once back in range
ros2 service call /pendulum_safety_supervisor/reset std_srvs/srv/Trigger
```

---

## Caveats & extension points

- **The clamp is a software layer, not the last line of defence.** On real
  hardware the drive's `0x6072` max_torque SDO remains the independent hardware
  torque ceiling. `clampEffort` sits above it.
- **Temperature monitoring is real-hardware only.** `DriveStatusBroadcaster`
  (the temperature source) does not run in Gazebo. In sim the supervisor runs
  with `monitor_temperature:=false`; thermal derating and over-temp e-stops are
  inactive there.
- **`DriveStatusBroadcaster` keeps its own thresholds.** It still grades
  `/diagnostics` for reporting; its threshold numbers are intentionally kept in
  sync with `safety_limits.yaml`. The supervisor is the component that *acts*.
- **A NaN reading never latches an e-stop.** Missing telemetry (an unread
  interface, a silent sensor) is treated as "no data", not as a breach.
