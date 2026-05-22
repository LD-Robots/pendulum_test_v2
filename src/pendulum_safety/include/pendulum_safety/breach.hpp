#ifndef PENDULUM_SAFETY__BREACH_HPP_
#define PENDULUM_SAFETY__BREACH_HPP_

#include "pendulum_safety/safety_limits.hpp"

namespace pendulum_safety
{

/// Breach predicates used by the supervisor. Each returns the matching
/// BreachReason, or BreachReason::NONE when within limits. A NaN input yields
/// NONE — missing telemetry must never by itself latch an e-stop.

BreachReason checkPosition(double position, const SafetyLimits & limits);
BreachReason checkVelocity(double velocity, const SafetyLimits & limits);
BreachReason checkMotorTemp(double temp, const SafetyLimits & limits);
BreachReason checkDriveTemp(double temp, const SafetyLimits & limits);
BreachReason checkBusVoltage(double voltage, const SafetyLimits & limits);

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__BREACH_HPP_
