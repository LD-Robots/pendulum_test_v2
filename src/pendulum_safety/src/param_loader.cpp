#include "pendulum_safety/param_loader.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pendulum_safety
{

SafetyLimits loadSafetyLimits(rclcpp::Node & node, const std::string & prefix)
{
  // TODO(phase-b): declare-if-absent and read every SafetyLimits field as
  // parameters under `prefix` (e.g. "safety.effort_limit").
  // Scaffolding stub: returns the built-in defaults.
  (void)node;
  (void)prefix;
  return SafetyLimits{};
}

SafetyLimits loadSafetyLimits(rclcpp_lifecycle::LifecycleNode & node,
  const std::string & prefix)
{
  // TODO(phase-b): shared implementation with the rclcpp::Node overload.
  // Scaffolding stub: returns the built-in defaults.
  (void)node;
  (void)prefix;
  return SafetyLimits{};
}

}  // namespace pendulum_safety
