#include "pendulum_pd_control/pendulum_pd_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace pendulum_pd_control
{

namespace
{
constexpr double kInitialValue = std::numeric_limits<double>::quiet_NaN();

double clamp_symmetric(double v, double limit)
{
  if (limit <= 0.0) {
    return v;
  }
  return std::clamp(v, -limit, limit);
}
}  // namespace

controller_interface::CallbackReturn PendulumPDController::on_init()
{
  try {
    auto_declare<std::string>("joint", "");
    auto_declare<double>("Kp", 0.0);
    auto_declare<double>("Kd", 0.0);
    auto_declare<double>("tau_limit", 0.0);
    auto_declare<double>("mgl", 0.0);
    auto_declare<double>("J", 0.0);
    auto_declare<double>("Fv", 0.0);
    auto_declare<double>("comp_sign", 1.0);
    auto_declare<bool>("ff_gravity", true);
    auto_declare<bool>("ff_inertia", true);
    auto_declare<bool>("ff_viscous", false);
    auto_declare<double>("hold_position", 0.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void PendulumPDController::load_params()
{
  auto node = get_node();
  params_.joint         = node->get_parameter("joint").as_string();
  params_.Kp            = node->get_parameter("Kp").as_double();
  params_.Kd            = node->get_parameter("Kd").as_double();
  params_.tau_limit     = node->get_parameter("tau_limit").as_double();
  params_.mgl           = node->get_parameter("mgl").as_double();
  params_.J             = node->get_parameter("J").as_double();
  params_.Fv            = node->get_parameter("Fv").as_double();
  params_.comp_sign     = node->get_parameter("comp_sign").as_double();
  params_.ff_gravity    = node->get_parameter("ff_gravity").as_bool();
  params_.ff_inertia    = node->get_parameter("ff_inertia").as_bool();
  params_.ff_viscous    = node->get_parameter("ff_viscous").as_bool();
  params_.hold_position = node->get_parameter("hold_position").as_double();
}

controller_interface::CallbackReturn PendulumPDController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  load_params();

  if (params_.joint.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joint' is required.");
    return controller_interface::CallbackReturn::ERROR;
  }

  setpoint_buf_.writeFromNonRT(Setpoint{params_.hold_position, 0.0, 0.0});

  auto node = get_node();
  setpoint_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    "~/setpoint", rclcpp::SystemDefaultsQoS(),
    std::bind(&PendulumPDController::setpoint_callback, this, std::placeholders::_1));

  hold_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/hold",
    std::bind(&PendulumPDController::hold_service, this,
              std::placeholders::_1, std::placeholders::_2));
  free_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/free",
    std::bind(&PendulumPDController::free_service, this,
              std::placeholders::_1, std::placeholders::_2));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PendulumPDController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {params_.joint + "/effort"};
  return cfg;
}

controller_interface::InterfaceConfiguration
PendulumPDController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {params_.joint + "/position", params_.joint + "/velocity"};
  return cfg;
}

controller_interface::CallbackReturn PendulumPDController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Refresh in case the user changed params between configure and activate.
  load_params();

  // Default to FREE on activation so a stale setpoint can't kick the joint.
  mode_.store(Mode::FREE);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PendulumPDController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!command_interfaces_.empty()) {
    (void)command_interfaces_[0].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PendulumPDController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Refresh FF flags & gains live — cheap, lets the user toggle ff_gravity
  // or sweep Kp via `ros2 param set` without re-spawning the controller.
  load_params();

  const auto mode = mode_.load();
  const Setpoint sp = *setpoint_buf_.readFromRT();

  const auto pos_opt = state_interfaces_[0].get_optional();
  const auto vel_opt = state_interfaces_[1].get_optional();
  if (!pos_opt || !vel_opt) {
    return controller_interface::return_type::OK;
  }
  const double q  = pos_opt.value();
  const double qd = vel_opt.value();

  double tau = 0.0;
  if (mode != Mode::FREE) {
    const double tau_g = params_.ff_gravity ? params_.mgl * std::sin(q) : 0.0;
    const double tau_J = params_.ff_inertia ? params_.J * sp.acceleration : 0.0;
    const double tau_v = params_.ff_viscous ? params_.Fv * qd : 0.0;
    const double tau_pd = (mode == Mode::PD)
      ? params_.Kp * (sp.position - q) + params_.Kd * (sp.velocity - qd)
      : 0.0;
    tau = params_.comp_sign * (tau_g + tau_J + tau_v) + tau_pd;
    tau = clamp_symmetric(tau, params_.tau_limit);
  }

  (void)command_interfaces_[0].set_value(tau);
  return controller_interface::return_type::OK;
}

void PendulumPDController::setpoint_callback(
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  Setpoint sp{params_.hold_position, 0.0, 0.0};
  if (!msg->positions.empty())     sp.position     = msg->positions[0];
  if (!msg->velocities.empty())    sp.velocity     = msg->velocities[0];
  if (!msg->accelerations.empty()) sp.acceleration = msg->accelerations[0];
  setpoint_buf_.writeFromNonRT(sp);
  // A streaming setpoint implies the caller wants tracking — switch to PD.
  mode_.store(Mode::PD);
}

void PendulumPDController::hold_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Snapshot the current measured position as the hold target.
  Setpoint sp{params_.hold_position, 0.0, 0.0};
  if (!state_interfaces_.empty()) {
    const auto pos_opt = state_interfaces_[0].get_optional();
    if (pos_opt) {
      sp.position = pos_opt.value();
    }
  }
  setpoint_buf_.writeFromNonRT(sp);
  mode_.store(Mode::PD);
  response->success = true;
  response->message = "Holding at q = " + std::to_string(sp.position);
}

void PendulumPDController::free_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  mode_.store(Mode::FREE);
  response->success = true;
  response->message = "FREE mode — zero effort";
}

}  // namespace pendulum_pd_control

PLUGINLIB_EXPORT_CLASS(
  pendulum_pd_control::PendulumPDController,
  controller_interface::ControllerInterface)
