#ifndef PENDULUM_SAFETY__SUPERVISOR_NODE_HPP_
#define PENDULUM_SAFETY__SUPERVISOR_NODE_HPP_

#include <string>

#include "pendulum_safety/safety_limits.hpp"
#include "pendulum_safety/sustained_effort_monitor.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace pendulum_safety
{

/// Standalone safety supervisor. Monitors joint state and drive telemetry,
/// detects limit breaches and drives a configurable, manually-reset e-stop.
///
/// Published surface:
///   /pendulum/safety/estop_state   std_msgs/Int8     latched — 0 clear / 1 FREE / 2 HOLD
///   /pendulum/safety/kp_scale      std_msgs/Float64  latched — thermal Kp multiplier
///   /pendulum/safety/breach_reason std_msgs/String   latched — latched breach name
///   /diagnostics                   diagnostic_msgs/DiagnosticArray
/// Services:
///   ~/estop  std_srvs/Trigger  manual e-stop (latches BreachReason::MANUAL)
///   ~/reset  std_srvs/Trigger  manual reset — refused while a breach is active
///
/// All callbacks and the watchdog timer run on one executor thread, so the
/// telemetry and latch state need no locking.
class SafetySupervisor : public rclcpp::Node
{
public:
  explicit SafetySupervisor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onJointState(sensor_msgs::msg::JointState::SharedPtr msg);
  void onEffort(sensor_msgs::msg::JointState::SharedPtr msg);
  void onWatchdog();
  void onEstopRequest(
    std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void onResetRequest(
    std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  /// Run every instantaneous breach predicate on the current telemetry.
  BreachReason detectBreach();
  /// Resolve a breach reason to its YAML-configured FREE / HOLD action.
  EstopAction actionFor(BreachReason reason) const;
  void latch(BreachReason reason);
  void publishEstopState();
  void publishDiagnostics(double kp_scale);

  // --- configuration ---
  SafetyLimits limits_;
  std::string joint_name_;
  bool monitor_temperature_{true};
  bool effort_from_joint_state_{false};
  double watchdog_rate_hz_{200.0};
  int report_period_cycles_{20};
  // Safety-by-default: boot latched (BreachReason::STARTUP, action FREE) so the
  // drives stay limp until the operator calls ~/reset.
  bool start_latched_{true};
  EstopAction action_temperature_{EstopAction::FREE};
  EstopAction action_overspeed_{EstopAction::FREE};
  EstopAction action_position_{EstopAction::HOLD};
  EstopAction action_manual_{EstopAction::FREE};
  EstopAction action_stale_{EstopAction::HOLD};
  EstopAction action_sustained_{EstopAction::FREE};

  // --- latest telemetry (NaN until first received) ---
  double position_;
  double velocity_;
  double effort_;
  double motor_temp_;
  double drive_temp_;
  double bus_voltage_;
  rclcpp::Time last_joint_state_;
  bool joint_state_received_{false};

  // --- e-stop latch ---
  bool estop_latched_{false};
  BreachReason latched_reason_{BreachReason::NONE};
  EstopAction latched_action_{EstopAction::FREE};
  SustainedEffortMonitor effort_monitor_;
  int cycle_count_{0};

  // --- startup arming ---
  // Breach detection runs only after the joint-state feed has been
  // continuously fresh for startup_grace_sec, so bringup transients (EtherCAT
  // reaching OP, controller activation) cannot latch a spurious e-stop.
  double startup_grace_sec_{2.0};
  bool detection_armed_{false};
  bool warmup_started_{false};
  rclcpp::Time warmup_start_;

  // --- ROS interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr effort_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_temp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drive_temp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bus_voltage_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kp_scale_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr breach_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__SUPERVISOR_NODE_HPP_
