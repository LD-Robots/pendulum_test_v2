#ifndef PENDULUM_SAFETY__SAFETY_LIMITS_HPP_
#define PENDULUM_SAFETY__SAFETY_LIMITS_HPP_

#include <cstdint>

namespace pendulum_safety
{

/// Why the supervisor latched an e-stop. NONE means no active breach.
enum class BreachReason : uint8_t
{
  NONE = 0,
  POSITION_LOW,
  POSITION_HIGH,
  OVERSPEED,
  MOTOR_OVERTEMP,
  DRIVE_OVERTEMP,
  BUS_VOLTAGE_LOW,
  BUS_VOLTAGE_HIGH,
  SUSTAINED_EFFORT,
  STALE_JOINT_STATE,
  MANUAL,
  STARTUP,    ///< boot-time latch: drives limp until the operator calls ~/reset
};

/// What a control path should do while an e-stop is latched.
enum class EstopAction : uint8_t
{
  FREE = 0,   ///< zero all torque — the joint coasts
  HOLD = 1,   ///< hold the position latched when the e-stop fired
};

/// Plain-data limit set shared by every safety consumer. POD: trivially
/// copyable into a realtime buffer, no heap. Every field has a default that
/// matches the URDF joint limits and the DriveStatusBroadcaster thresholds, so
/// a partially-specified YAML still yields a valid, sane struct.
struct SafetyLimits
{
  // 3.1 position
  bool   position_limits_enable = true;
  double position_min = 0.0;                  ///< rad — URDF joint lower
  double position_max = 6.283185307179586;    ///< rad — URDF joint upper (2*pi)

  // 3.2 effort
  double effort_limit = 20.0;                  ///< Nm, symmetric — URDF effort
  double sustained_effort_threshold = 16.0;    ///< Nm — "high effort" level
  double sustained_effort_window_sec = 1.0;    ///< s above threshold => breach

  // 3.3 velocity
  double velocity_limit = 16.02;               ///< rad/s — URDF velocity limit
  double slew_rate_limit = 5.5;                ///< rad/s — position-command slew
  double acceleration_limit = 60.0;            ///< rad/s^2 — command accel cap

  // 3.4 temperature / bus voltage
  double motor_temp_warn  = 70.0;              ///< degC — start derating Kp
  double motor_temp_error = 90.0;              ///< degC — e-stop
  double drive_temp_warn  = 70.0;              ///< degC
  double drive_temp_error = 85.0;              ///< degC
  double bus_voltage_min  = 40.0;              ///< V
  double bus_voltage_max  = 54.0;              ///< V
  double kp_scale_floor   = 0.3;               ///< Kp multiplier at/above error

  // watchdog
  double joint_state_timeout_sec = 0.25;       ///< s — /joint_states freshness
};

/// Human-readable name for a breach reason (string literal, no allocation).
const char * to_string(BreachReason reason);

/// Human-readable name for an e-stop action (string literal, no allocation).
const char * to_string(EstopAction action);

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__SAFETY_LIMITS_HPP_
