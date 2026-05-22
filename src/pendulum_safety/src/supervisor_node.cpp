#include "pendulum_safety/supervisor_node.hpp"

#include <string>

namespace pendulum_safety
{

SafetySupervisor::SafetySupervisor(const rclcpp::NodeOptions & options)
: rclcpp::Node("pendulum_safety_supervisor", options)
{
  // --- Supervisor parameters: telemetry topics, watchdog rate, and the
  // per-trigger e-stop action map ("free" zeroes torque, "hold" locks pos). ---
  declare_parameter<std::string>("joint_name", "pendulum_joint");
  declare_parameter<std::string>("joint_state_topic", "/joint_states");
  declare_parameter<std::string>("effort_topic", "/filtered_joint_states");
  declare_parameter<bool>("monitor_temperature", true);
  declare_parameter<std::string>(
    "temperature_topic_motor", "/drive_status_broadcaster/motor_temperature");
  declare_parameter<std::string>(
    "temperature_topic_drive", "/drive_status_broadcaster/drive_temperature");
  declare_parameter<std::string>(
    "bus_voltage_topic", "/drive_status_broadcaster/bus_voltage");
  declare_parameter<double>("watchdog_rate_hz", 200.0);
  declare_parameter<std::string>("action.temperature", "free");
  declare_parameter<std::string>("action.overspeed", "free");
  declare_parameter<std::string>("action.position", "hold");
  declare_parameter<std::string>("action.manual", "free");
  declare_parameter<std::string>("action.stale_joint_state", "hold");
  declare_parameter<std::string>("action.sustained_effort", "free");

  RCLCPP_INFO(
    get_logger(),
    "pendulum_safety_supervisor up (scaffolding — breach detection, the "
    "watchdog timer, the e-stop/reset services and the publishers arrive in "
    "Phase C)");

  // TODO(phase-c): load SafetyLimits via loadSafetyLimits(); create the joint
  // state / effort / temperature subscriptions; start the watchdog timer; add
  // the ~/estop and ~/reset services; add the estop_state, kp_scale,
  // breach_reason and /diagnostics publishers.
}

}  // namespace pendulum_safety
