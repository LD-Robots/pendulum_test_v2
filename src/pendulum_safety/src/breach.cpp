#include "pendulum_safety/breach.hpp"

namespace pendulum_safety
{

// TODO(phase-b): implement the breach predicates. Each compares the input
// against the relevant SafetyLimits field, returns the matching BreachReason
// or NONE, and treats a NaN input as NONE (missing telemetry never latches).
// Scaffolding stubs: never report a breach.

BreachReason checkPosition(double position, const SafetyLimits & limits)
{
  (void)position;
  (void)limits;
  return BreachReason::NONE;
}

BreachReason checkVelocity(double velocity, const SafetyLimits & limits)
{
  (void)velocity;
  (void)limits;
  return BreachReason::NONE;
}

BreachReason checkMotorTemp(double temp, const SafetyLimits & limits)
{
  (void)temp;
  (void)limits;
  return BreachReason::NONE;
}

BreachReason checkDriveTemp(double temp, const SafetyLimits & limits)
{
  (void)temp;
  (void)limits;
  return BreachReason::NONE;
}

BreachReason checkBusVoltage(double voltage, const SafetyLimits & limits)
{
  (void)voltage;
  (void)limits;
  return BreachReason::NONE;
}

}  // namespace pendulum_safety
