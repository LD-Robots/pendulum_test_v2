#include "pendulum_safety/rate_limiter.hpp"

#include <algorithm>

namespace pendulum_safety
{

double RateLimiter::limit(double desired, double dt, double slew_rate,
  double accel_limit)
{
  if (!seeded_) {
    // First call after construction / reset() — seed to avoid a startup jump.
    prev_cmd_ = desired;
    prev_vel_ = 0.0;
    seeded_ = true;
    return desired;
  }
  if (dt <= 0.0) {
    // No time elapsed — cannot integrate; hold the last command.
    return prev_cmd_;
  }

  // Command velocity needed to reach `desired` in this cycle.
  double cmd_vel = (desired - prev_cmd_) / dt;

  // Acceleration limit: bound how far the command velocity may move from the
  // previous cycle's command velocity.
  if (accel_limit > 0.0) {
    const double dv = accel_limit * dt;
    cmd_vel = std::clamp(cmd_vel, prev_vel_ - dv, prev_vel_ + dv);
  }
  // Slew-rate limit: bound the command velocity magnitude.
  if (slew_rate > 0.0) {
    cmd_vel = std::clamp(cmd_vel, -slew_rate, slew_rate);
  }

  const double cmd = prev_cmd_ + cmd_vel * dt;
  prev_cmd_ = cmd;
  prev_vel_ = cmd_vel;
  return cmd;
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
