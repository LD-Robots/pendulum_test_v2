#include "pendulum_pd_control/drive_status_broadcaster.hpp"

#include <algorithm>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace pendulum_pd_control
{

namespace
{
constexpr const char * kDefaultSignals[] = {
  "motor_temperature", "drive_temperature", "bus_voltage", "first_encoder",
  "filtered_velocity", "filtered_torque", "position_demand", "following_error",
};
}  // namespace

controller_interface::CallbackReturn DriveStatusBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("joint", "pendulum_joint");
    auto_declare<std::vector<std::string>>(
      "signals",
      std::vector<std::string>(std::begin(kDefaultSignals), std::end(kDefaultSignals)));
    auto_declare<double>("publish_rate", 20.0);
    auto_declare<double>("motor_temp_warn", 70.0);
    auto_declare<double>("motor_temp_error", 90.0);
    auto_declare<double>("drive_temp_warn", 70.0);
    auto_declare<double>("drive_temp_error", 85.0);
    auto_declare<double>("bus_voltage_min", 40.0);
    auto_declare<double>("bus_voltage_max", 54.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DriveStatusBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joint_ = node->get_parameter("joint").as_string();
  signals_ = node->get_parameter("signals").as_string_array();
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  thr_.motor_temp_warn  = node->get_parameter("motor_temp_warn").as_double();
  thr_.motor_temp_error = node->get_parameter("motor_temp_error").as_double();
  thr_.drive_temp_warn  = node->get_parameter("drive_temp_warn").as_double();
  thr_.drive_temp_error = node->get_parameter("drive_temp_error").as_double();
  thr_.bus_voltage_min  = node->get_parameter("bus_voltage_min").as_double();
  thr_.bus_voltage_max  = node->get_parameter("bus_voltage_max").as_double();

  if (signals_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'signals' is empty — nothing to broadcast.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'publish_rate' must be > 0.");
    return controller_interface::CallbackReturn::ERROR;
  }
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  signal_pubs_.clear();
  signal_rt_pubs_.clear();
  for (const auto & sig : signals_) {
    auto pub = node->create_publisher<std_msgs::msg::Float64>(
      "~/" + sig, rclcpp::SystemDefaultsQoS());
    signal_rt_pubs_.push_back(std::make_unique<Float64Publisher>(pub));
    signal_pubs_.push_back(pub);
  }

  diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  diag_rt_pub_ = std::make_unique<DiagPublisher>(diag_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DriveStatusBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
DriveStatusBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(signals_.size());
  for (const auto & sig : signals_) {
    cfg.names.push_back(joint_ + "/" + sig);
  }
  return cfg;
}

controller_interface::CallbackReturn DriveStatusBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_publish_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DriveStatusBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DriveStatusBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Telemetry is slow-moving — throttle publishing to publish_rate.
  if ((time - last_publish_time_) < publish_period_) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pendulum_drive: telemetry";
  status.hardware_id = joint_;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "OK";

  for (size_t i = 0; i < signals_.size(); ++i) {
    const auto value_opt = state_interfaces_[i].get_optional();
    const double value = value_opt ? value_opt.value() : std::numeric_limits<double>::quiet_NaN();
    const std::string & name = signals_[i];

    if (signal_rt_pubs_[i]->trylock()) {
      signal_rt_pubs_[i]->msg_.data = value;
      signal_rt_pubs_[i]->unlockAndPublish();
    }

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = name;
    kv.value = std::to_string(value);
    status.values.push_back(kv);

    // Thresholding for the three signals we recognise.
    auto escalate = [&status](uint8_t level, const std::string & msg) {
      if (level > status.level) {
        status.level = level;
        status.message = msg;
      }
    };
    if (name == "motor_temperature") {
      if (value >= thr_.motor_temp_error) {
        escalate(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor over-temperature");
      } else if (value >= thr_.motor_temp_warn) {
        escalate(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Motor temperature high");
      }
    } else if (name == "drive_temperature") {
      if (value >= thr_.drive_temp_error) {
        escalate(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Drive over-temperature");
      } else if (value >= thr_.drive_temp_warn) {
        escalate(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Drive temperature high");
      }
    } else if (name == "bus_voltage") {
      if (value < thr_.bus_voltage_min || value > thr_.bus_voltage_max) {
        escalate(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Bus voltage out of range");
      }
    }
  }

  if (diag_rt_pub_->trylock()) {
    diag_rt_pub_->msg_.header.stamp = time;
    diag_rt_pub_->msg_.status.clear();
    diag_rt_pub_->msg_.status.push_back(status);
    diag_rt_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace pendulum_pd_control

PLUGINLIB_EXPORT_CLASS(
  pendulum_pd_control::DriveStatusBroadcaster,
  controller_interface::ControllerInterface)
