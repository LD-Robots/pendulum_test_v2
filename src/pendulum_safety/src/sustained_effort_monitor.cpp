#include "pendulum_safety/sustained_effort_monitor.hpp"

namespace pendulum_safety
{

bool SustainedEffortMonitor::update(double abs_effort, double dt,
  double threshold, double window_sec)
{
  // TODO(phase-b): leaky-bucket accumulate above-threshold time, drain on
  // dips, trip once the accumulator exceeds `window_sec`.
  // Scaffolding stub: never trips.
  (void)abs_effort;
  (void)dt;
  (void)threshold;
  (void)window_sec;
  return tripped_;
}

void SustainedEffortMonitor::reset()
{
  accum_sec_ = 0.0;
  tripped_ = false;
}

}  // namespace pendulum_safety
