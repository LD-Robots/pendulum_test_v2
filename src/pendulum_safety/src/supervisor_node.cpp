#include "pendulum_safety/supervisor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include "pendulum_safety/breach.hpp"
#include "pendulum_safety/param_loader.hpp"
#include "pendulum_safety/thermal.hpp"

namespace pendulum_safety
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Parse a "free" / "hold" YAML string into an EstopAction.
EstopAction parseAction(const std::string & text, EstopAction fallback)
{
  if (text == "hold" || text == "HOLD") {
    return EstopAction::HOLD;
  }
  if (text == "free" || text == "FREE") {
    return EstopAction::FREE;
  }
  return fallback;
}

/// Index of `joint` in a JointState name array, or -1 if absent.
int jointIndex(const sensor_msgs::msg::JointState & msg, const std::string & joint)
{
  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i] == joint) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace

SafetySupervisor::SafetySupervisor(const rclcpp::NodeOptions & options)
: rclcpp::Node("pendulum_safety_supervisor", options),
  position_(kNaN), velocity_(kNaN), effort_(kNaN),
  motor_temp_(kNaN), drive_temp_(kNaN), bus_voltage_(kNaN),
  last_joint_state_(0, 0, RCL_ROS_TIME)
{
  // --- parameters ---
  joint_name_ = declare_parameter<std::string>("joint_name", "pendulum_joint");
  const std::string joint_state_topic =
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
  const std::string effort_topic =
    declare_parameter<std::string>("effort_topic", "/filtered_joint_states");
  monitor_temperature_ = declare_parameter<bool>("monitor_temperature", true);
  const std::string motor_topic = declare_parameter<std::string>(
    "temperature_topic_motor", "/drive_status_broadcaster/motor_temperature");
  const std::string drive_topic = declare_parameter<std::string>(
    "temperature_topic_drive", "/drive_status_broadcaster/drive_temperature");
  const std::string voltage_topic = declare_parameter<std::string>(
    "bus_voltage_topic", "/drive_status_broadcaster/bus_voltage");
  watchdog_rate_hz_ = declare_parameter<double>("watchdog_rate_hz", 200.0);
  if (watchdog_rate_hz_ <= 0.0) {
    watchdog_rate_hz_ = 200.0;
  }
  report_period_cycles_ =
    std::max(1, static_cast<int>(std::lround(watchdog_rate_hz_ / 10.0)));

  action_temperature_ = parseAction(
    declare_parameter<std::string>("action.temperature", "free"),
    EstopAction::FREE);
  action_overspeed_ = parseAction(
    declare_parameter<std::string>("action.overspeed", "free"),
    EstopAction::FREE);
  action_position_ = parseAction(
    declare_parameter<std::string>("action.position", "hold"),
    EstopAction::HOLD);
  action_manual_ = parseAction(
    declare_parameter<std::string>("action.manual", "free"),
    EstopAction::FREE);
  action_stale_ = parseAction(
    declare_parameter<std::string>("action.stale_joint_state", "hold"),
    EstopAction::HOLD);
  action_sustained_ = parseAction(
    declare_parameter<std::string>("action.sustained_effort", "free"),
    EstopAction::FREE);

  limits_ = loadSafetyLimits(*this, "safety.");
  effort_from_joint_state_ = (effort_topic == joint_state_topic);

  // --- publishers (estop / kp_scale / breach_reason are latched) ---
  const auto latched = rclcpp::QoS(1).transient_local();
  estop_pub_ = create_publisher<std_msgs::msg::Int8>(
    "/pendulum/safety/estop_state", latched);
  kp_scale_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/pendulum/safety/kp_scale", latched);
  breach_pub_ = create_publisher<std_msgs::msg::String>(
    "/pendulum/safety/breach_reason", latched);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));

  // Seed the latched topics so consumers spawned later get an initial value.
  publishEstopState();
  std_msgs::msg::Float64 kp_init;
  kp_init.data = 1.0;
  kp_scale_pub_->publish(kp_init);

  // --- subscriptions ---
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic, rclcpp::SensorDataQoS(),
    std::bind(&SafetySupervisor::onJointState, this, std::placeholders::_1));
  if (!effort_from_joint_state_) {
    effort_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      effort_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetySupervisor::onEffort, this, std::placeholders::_1));
  }
  if (monitor_temperature_) {
    motor_temp_sub_ = create_subscription<std_msgs::msg::Float64>(
      motor_topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::SharedPtr msg) {motor_temp_ = msg->data;});
    drive_temp_sub_ = create_subscription<std_msgs::msg::Float64>(
      drive_topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::SharedPtr msg) {drive_temp_ = msg->data;});
    bus_voltage_sub_ = create_subscription<std_msgs::msg::Float64>(
      voltage_topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::SharedPtr msg) {bus_voltage_ = msg->data;});
  }

  // --- services ---
  estop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/estop",
    std::bind(&SafetySupervisor::onEstopRequest, this,
      std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(&SafetySupervisor::onResetRequest, this,
      std::placeholders::_1, std::placeholders::_2));

  // --- watchdog timer ---
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / watchdog_rate_hz_));
  watchdog_timer_ = create_wall_timer(
    period, std::bind(&SafetySupervisor::onWatchdog, this));

  RCLCPP_INFO(
    get_logger(),
    "pendulum_safety_supervisor active — watchdog %.0f Hz, temperature "
    "monitoring %s", watchdog_rate_hz_, monitor_temperature_ ? "on" : "off");
}

void SafetySupervisor::onJointState(sensor_msgs::msg::JointState::SharedPtr msg)
{
  const int idx = jointIndex(*msg, joint_name_);
  if (idx < 0) {
    return;
  }
  const size_t i = static_cast<size_t>(idx);
  if (i < msg->position.size()) {
    position_ = msg->position[i];
  }
  if (i < msg->velocity.size()) {
    velocity_ = msg->velocity[i];
  }
  if (effort_from_joint_state_ && i < msg->effort.size()) {
    effort_ = msg->effort[i];
  }
  last_joint_state_ = now();
  joint_state_received_ = true;
}

void SafetySupervisor::onEffort(sensor_msgs::msg::JointState::SharedPtr msg)
{
  const int idx = jointIndex(*msg, joint_name_);
  if (idx < 0) {
    return;
  }
  const size_t i = static_cast<size_t>(idx);
  if (i < msg->effort.size()) {
    effort_ = msg->effort[i];
  }
}

BreachReason SafetySupervisor::detectBreach()
{
  if (joint_state_received_) {
    const double age = (now() - last_joint_state_).seconds();
    if (age > limits_.joint_state_timeout_sec) {
      return BreachReason::STALE_JOINT_STATE;
    }
  }
  BreachReason reason = checkPosition(position_, limits_);
  if (reason != BreachReason::NONE) {
    return reason;
  }
  reason = checkVelocity(velocity_, limits_);
  if (reason != BreachReason::NONE) {
    return reason;
  }
  if (monitor_temperature_) {
    reason = checkMotorTemp(motor_temp_, limits_);
    if (reason != BreachReason::NONE) {
      return reason;
    }
    reason = checkDriveTemp(drive_temp_, limits_);
    if (reason != BreachReason::NONE) {
      return reason;
    }
    reason = checkBusVoltage(bus_voltage_, limits_);
    if (reason != BreachReason::NONE) {
      return reason;
    }
  }
  return BreachReason::NONE;
}

EstopAction SafetySupervisor::actionFor(BreachReason reason) const
{
  switch (reason) {
    case BreachReason::MOTOR_OVERTEMP:
    case BreachReason::DRIVE_OVERTEMP:
    case BreachReason::BUS_VOLTAGE_LOW:
    case BreachReason::BUS_VOLTAGE_HIGH:
      return action_temperature_;
    case BreachReason::OVERSPEED:
      return action_overspeed_;
    case BreachReason::POSITION_LOW:
    case BreachReason::POSITION_HIGH:
      return action_position_;
    case BreachReason::SUSTAINED_EFFORT:
      return action_sustained_;
    case BreachReason::STALE_JOINT_STATE:
      return action_stale_;
    case BreachReason::MANUAL:
      return action_manual_;
    case BreachReason::NONE:
      return EstopAction::FREE;
  }
  return EstopAction::FREE;
}

void SafetySupervisor::latch(BreachReason reason)
{
  estop_latched_ = true;
  latched_reason_ = reason;
  latched_action_ = actionFor(reason);
  RCLCPP_ERROR(
    get_logger(), "E-STOP latched: %s (action: %s)",
    to_string(reason), to_string(latched_action_));
  publishEstopState();
}

void SafetySupervisor::publishEstopState()
{
  std_msgs::msg::Int8 state;
  state.data = !estop_latched_
    ? 0
    : (latched_action_ == EstopAction::HOLD ? 2 : 1);
  estop_pub_->publish(state);

  std_msgs::msg::String breach;
  breach.data = to_string(estop_latched_ ? latched_reason_ : BreachReason::NONE);
  breach_pub_->publish(breach);
}

void SafetySupervisor::onWatchdog()
{
  const double dt = 1.0 / watchdog_rate_hz_;
  const bool effort_tripped = effort_monitor_.update(
    std::abs(effort_), dt, limits_.sustained_effort_threshold,
    limits_.sustained_effort_window_sec);

  if (!estop_latched_) {
    BreachReason reason = detectBreach();
    if (reason == BreachReason::NONE && effort_tripped) {
      reason = BreachReason::SUSTAINED_EFFORT;
    }
    if (reason != BreachReason::NONE) {
      latch(reason);
    }
  }

  // Thermal Kp derating runs continuously, independent of the e-stop latch.
  const double kp_scale = monitor_temperature_
    ? kpScaleCombined(motor_temp_, drive_temp_, limits_)
    : 1.0;

  // estop_state is event-driven; kp_scale and diagnostics are decimated to
  // ~10 Hz — the watchdog itself keeps detecting breaches every cycle.
  if (++cycle_count_ >= report_period_cycles_) {
    cycle_count_ = 0;
    std_msgs::msg::Float64 kp;
    kp.data = kp_scale;
    kp_scale_pub_->publish(kp);
    publishDiagnostics(kp_scale);
  }
}

void SafetySupervisor::onEstopRequest(
  std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  if (!estop_latched_) {
    latch(BreachReason::MANUAL);
  }
  response->success = true;
  response->message = "e-stop latched (manual)";
}

void SafetySupervisor::onResetRequest(
  std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  if (!estop_latched_) {
    response->success = true;
    response->message = "e-stop already clear";
    return;
  }
  const BreachReason active = detectBreach();
  if (active != BreachReason::NONE) {
    response->success = false;
    response->message =
      std::string("reset refused — active breach: ") + to_string(active);
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  estop_latched_ = false;
  latched_reason_ = BreachReason::NONE;
  latched_action_ = EstopAction::FREE;
  effort_monitor_.reset();
  publishEstopState();
  RCLCPP_INFO(get_logger(), "e-stop reset — supervisor clear");
  response->success = true;
  response->message = "e-stop cleared";
}

void SafetySupervisor::publishDiagnostics(double kp_scale)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pendulum_safety: supervisor";
  status.hardware_id = joint_name_;
  if (estop_latched_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = std::string("E-STOP latched: ") +
      to_string(latched_reason_) + " (action " +
      to_string(latched_action_) + ")";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  }

  const auto kv = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };
  kv("position", std::to_string(position_));
  kv("velocity", std::to_string(velocity_));
  kv("effort", std::to_string(effort_));
  if (monitor_temperature_) {
    kv("motor_temp", std::to_string(motor_temp_));
    kv("drive_temp", std::to_string(drive_temp_));
    kv("bus_voltage", std::to_string(bus_voltage_));
  }
  kv("kp_scale", std::to_string(kp_scale));
  kv("effort_accum_sec",
    std::to_string(effort_monitor_.timeAboveThreshold()));

  array.status.push_back(status);
  diag_pub_->publish(array);
}

}  // namespace pendulum_safety
