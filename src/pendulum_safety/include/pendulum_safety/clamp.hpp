#ifndef PENDULUM_SAFETY__CLAMP_HPP_
#define PENDULUM_SAFETY__CLAMP_HPP_

#include "pendulum_safety/safety_limits.hpp"

namespace pendulum_safety
{

/// Symmetric clamp to [-limit, +limit]. A non-positive limit disables the
/// clamp (the value passes through) — this preserves the semantics of the
/// clamp_symmetric() helpers previously inlined in the PD and PVT controllers.
/// Realtime-safe: no allocation, no locking, no logging.
inline double clampSymmetric(double value, double limit)
{
  if (limit <= 0.0) {
    return value;
  }
  if (value < -limit) {
    return -limit;
  }
  if (value > limit) {
    return limit;
  }
  return value;
}

/// Clamp a torque command to the symmetric effort limit.
inline double clampEffort(double effort, const SafetyLimits & limits)
{
  return clampSymmetric(effort, limits.effort_limit);
}

/// Clamp a velocity command to the symmetric velocity limit.
inline double clampVelocity(double velocity, const SafetyLimits & limits)
{
  return clampSymmetric(velocity, limits.velocity_limit);
}

/// Clamp a position command to [position_min, position_max]. When position
/// limiting is disabled the value passes through unchanged.
inline double clampPosition(double position, const SafetyLimits & limits)
{
  if (!limits.position_limits_enable) {
    return position;
  }
  if (position < limits.position_min) {
    return limits.position_min;
  }
  if (position > limits.position_max) {
    return limits.position_max;
  }
  return position;
}

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__CLAMP_HPP_
