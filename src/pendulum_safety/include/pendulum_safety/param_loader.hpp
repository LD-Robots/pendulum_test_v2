#ifndef PENDULUM_SAFETY__PARAM_LOADER_HPP_
#define PENDULUM_SAFETY__PARAM_LOADER_HPP_

#include <string>

#include "pendulum_safety/safety_limits.hpp"

namespace rclcpp { class Node; }
namespace rclcpp_lifecycle { class LifecycleNode; }

namespace pendulum_safety
{

/// Declare (if not already declared) and read every SafetyLimits field as ROS
/// parameters on `node`, under `prefix` (e.g. "safety." -> "safety.effort_limit").
/// Two overloads cover the two node types in this workspace: a plain
/// rclcpp::Node (the supervisor and the policy node) and the lifecycle node
/// behind a ros2_control controller's get_node() handle.
SafetyLimits loadSafetyLimits(rclcpp::Node & node,
  const std::string & prefix = "safety.");
SafetyLimits loadSafetyLimits(rclcpp_lifecycle::LifecycleNode & node,
  const std::string & prefix = "safety.");

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__PARAM_LOADER_HPP_
