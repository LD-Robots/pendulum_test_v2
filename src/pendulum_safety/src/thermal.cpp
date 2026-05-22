#include "pendulum_safety/thermal.hpp"

namespace pendulum_safety
{

double kpScaleForTemp(double temp, double warn, double error, double floor)
{
  // TODO(phase-b): 1.0 below `warn`, linear ramp down to `floor` between
  // `warn` and `error`, `floor` at/above `error`; NaN temp -> 1.0.
  // Scaffolding stub: no derating.
  (void)temp;
  (void)warn;
  (void)error;
  (void)floor;
  return 1.0;
}

double kpScaleCombined(double motor_temp, double drive_temp,
  const SafetyLimits & limits)
{
  // TODO(phase-b): min of the motor and drive derating scales.
  // Scaffolding stub: no derating.
  (void)motor_temp;
  (void)drive_temp;
  (void)limits;
  return 1.0;
}

}  // namespace pendulum_safety
