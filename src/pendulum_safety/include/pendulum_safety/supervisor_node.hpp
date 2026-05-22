#ifndef PENDULUM_SAFETY__SUPERVISOR_NODE_HPP_
#define PENDULUM_SAFETY__SUPERVISOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace pendulum_safety
{

/// Standalone safety supervisor. Monitors joint state and drive telemetry,
/// detects limit breaches and drives a configurable, manually-reset e-stop.
///
/// Scaffolding state (Phase A): declares its parameters and spins. Breach
/// detection, the watchdog timer, the ~/estop and ~/reset services and the
/// estop_state / kp_scale / breach_reason / diagnostics publishers are added
/// in Phase C.
class SafetySupervisor : public rclcpp::Node
{
public:
  explicit SafetySupervisor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Phase C: SafetyLimits, subscriptions, watchdog timer, services, publishers.
};

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__SUPERVISOR_NODE_HPP_
