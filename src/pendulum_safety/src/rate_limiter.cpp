#include "pendulum_safety/rate_limiter.hpp"

namespace pendulum_safety
{

double RateLimiter::limit(double desired, double dt, double slew_rate,
  double accel_limit)
{
  // TODO(phase-b): bound the command velocity by `accel_limit`, then by
  // `slew_rate`, and integrate to the emitted command.
  // Scaffolding stub: pass the target through and seed the state.
  (void)dt;
  (void)slew_rate;
  (void)accel_limit;
  prev_cmd_ = desired;
  prev_vel_ = 0.0;
  seeded_ = true;
  return desired;
}

void RateLimiter::reset()
{
  seeded_ = false;
  prev_cmd_ = 0.0;
  prev_vel_ = 0.0;
}

void RateLimiter::seed(double position)
{
  prev_cmd_ = position;
  prev_vel_ = 0.0;
  seeded_ = true;
}

}  // namespace pendulum_safety
