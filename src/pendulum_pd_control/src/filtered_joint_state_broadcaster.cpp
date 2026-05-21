#include "pendulum_pd_control/filtered_joint_state_broadcaster.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace pendulum_pd_control
{

namespace
{
constexpr double kTwoPi = 6.283185307179586;

// First-order IIR low-pass smoothing factor for a given cutoff and time step.
// cutoff_hz <= 0 disables filtering (alpha = 1 → straight passthrough).
double lpf_alpha(double cutoff_hz, double dt)
{
  if (cutoff_hz <= 0.0 || dt <= 0.0) {
    return 1.0;
  }
  const double alpha = 1.0 - std::exp(-kTwoPi * cutoff_hz * dt);
  return std::clamp(alpha, 0.0, 1.0);
}
}  // namespace

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("joint", "pendulum_joint");
    auto_declare<double>("velocity_cutoff_hz", 30.0);
    auto_declare<double>("torque_cutoff_hz", 30.0);
    auto_declare<double>("publish_rate", 100.0);
    auto_declare<std::string>("topic", "filtered_joint_states");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joint_ = node->get_parameter("joint").as_string();
  topic_ = node->get_parameter("topic").as_string();
  velocity_cutoff_hz_ = node->get_parameter("velocity_cutoff_hz").as_double();
  torque_cutoff_hz_ = node->get_parameter("torque_cutoff_hz").as_double();
  publish_rate_ = node->get_parameter("publish_rate").as_double();

  if (joint_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joint' is required.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (topic_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'topic' is required.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'publish_rate' must be > 0.");
    return controller_interface::CallbackReturn::ERROR;
  }
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
    topic_, rclcpp::SystemDefaultsQoS());
  rt_joint_state_pub_ = std::make_unique<JointStatePublisher>(joint_state_pub_);

  // Pre-size the message so the realtime update path never allocates.
  rt_joint_state_pub_->msg_.name = {joint_};
  rt_joint_state_pub_->msg_.position.assign(1, 0.0);
  rt_joint_state_pub_->msg_.velocity.assign(1, 0.0);
  rt_joint_state_pub_->msg_.effort.assign(1, 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FilteredJointStateBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
FilteredJointStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Order matters — update() indexes state_interfaces_ as 0/1/2 below.
  cfg.names = {joint_ + "/position", joint_ + "/velocity", joint_ + "/effort"};
  return cfg;
}

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  filter_initialized_ = false;
  last_publish_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FilteredJointStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Cutoffs are live-tunable — refreshed each cycle like PendulumPDController.
  velocity_cutoff_hz_ = get_node()->get_parameter("velocity_cutoff_hz").as_double();
  torque_cutoff_hz_ = get_node()->get_parameter("torque_cutoff_hz").as_double();

  const auto pos_opt = state_interfaces_[0].get_optional();
  const auto vel_opt = state_interfaces_[1].get_optional();
  const auto eff_opt = state_interfaces_[2].get_optional();
  if (!pos_opt || !vel_opt || !eff_opt) {
    return controller_interface::return_type::OK;
  }
  const double pos = pos_opt.value();
  const double vel = vel_opt.value();
  const double eff = eff_opt.value();
  // State interfaces start as NaN — wait until the drive is actually streaming.
  if (!std::isfinite(vel) || !std::isfinite(eff)) {
    return controller_interface::return_type::OK;
  }

  // Run the low-pass filter every cycle (full 1 kHz), regardless of publish rate.
  const double dt = period.seconds();
  if (!filter_initialized_) {
    vel_filt_ = vel;   // seed on the first finite sample — skips the startup ramp
    tau_filt_ = eff;
    filter_initialized_ = true;
  } else {
    vel_filt_ += lpf_alpha(velocity_cutoff_hz_, dt) * (vel - vel_filt_);
    tau_filt_ += lpf_alpha(torque_cutoff_hz_, dt) * (eff - tau_filt_);
  }

  // Throttle publishing — telemetry consumers don't need the full 1 kHz.
  if ((time - last_publish_time_) < publish_period_) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  if (rt_joint_state_pub_->trylock()) {
    auto & msg = rt_joint_state_pub_->msg_;
    msg.header.stamp = time;
    msg.position[0] = pos;        // unfiltered — position is a clean encoder count
    msg.velocity[0] = vel_filt_;
    msg.effort[0] = tau_filt_;
    rt_joint_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace pendulum_pd_control

PLUGINLIB_EXPORT_CLASS(
  pendulum_pd_control::FilteredJointStateBroadcaster,
  controller_interface::ControllerInterface)
