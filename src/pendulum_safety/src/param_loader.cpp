#include "pendulum_safety/param_loader.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pendulum_safety
{
namespace
{

/// Declare-if-absent and read every SafetyLimits field under `prefix`. Works
/// for any node exposing the rclcpp parameter API (rclcpp::Node and
/// rclcpp_lifecycle::LifecycleNode), so the two public overloads share it.
/// has_parameter() guards re-declaration, making this safe to call after a
/// controller has already declared parameters of its own.
template<typename NodeT>
SafetyLimits loadImpl(NodeT & node, const std::string & prefix)
{
  SafetyLimits limits;  // built-in defaults — used when a key is absent

  auto get_double = [&](const std::string & name, double def) -> double {
      const std::string full = prefix + name;
      if (!node.has_parameter(full)) {
        node.declare_parameter(full, def);
      }
      return node.get_parameter(full).as_double();
    };
  auto get_bool = [&](const std::string & name, bool def) -> bool {
      const std::string full = prefix + name;
      if (!node.has_parameter(full)) {
        node.declare_parameter(full, def);
      }
      return node.get_parameter(full).as_bool();
    };

  limits.position_limits_enable =
    get_bool("position_limits_enable", limits.position_limits_enable);
  limits.position_min = get_double("position_min", limits.position_min);
  limits.position_max = get_double("position_max", limits.position_max);

  limits.effort_limit = get_double("effort_limit", limits.effort_limit);
  limits.sustained_effort_threshold =
    get_double("sustained_effort_threshold", limits.sustained_effort_threshold);
  limits.sustained_effort_window_sec =
    get_double("sustained_effort_window_sec", limits.sustained_effort_window_sec);

  limits.velocity_limit = get_double("velocity_limit", limits.velocity_limit);
  limits.slew_rate_limit = get_double("slew_rate_limit", limits.slew_rate_limit);
  limits.acceleration_limit =
    get_double("acceleration_limit", limits.acceleration_limit);

  limits.motor_temp_warn = get_double("motor_temp_warn", limits.motor_temp_warn);
  limits.motor_temp_error = get_double("motor_temp_error", limits.motor_temp_error);
  limits.drive_temp_warn = get_double("drive_temp_warn", limits.drive_temp_warn);
  limits.drive_temp_error = get_double("drive_temp_error", limits.drive_temp_error);
  limits.bus_voltage_min = get_double("bus_voltage_min", limits.bus_voltage_min);
  limits.bus_voltage_max = get_double("bus_voltage_max", limits.bus_voltage_max);
  limits.kp_scale_floor = get_double("kp_scale_floor", limits.kp_scale_floor);

  limits.joint_state_timeout_sec =
    get_double("joint_state_timeout_sec", limits.joint_state_timeout_sec);

  return limits;
}

}  // namespace

SafetyLimits loadSafetyLimits(rclcpp::Node & node, const std::string & prefix)
{
  return loadImpl(node, prefix);
}

SafetyLimits loadSafetyLimits(rclcpp_lifecycle::LifecycleNode & node,
  const std::string & prefix)
{
  return loadImpl(node, prefix);
}

}  // namespace pendulum_safety
