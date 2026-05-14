#include "pendulum_pvt_control/pendulum_pvt_controller.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace pendulum_pvt_control
{

namespace
{
double clamp_symmetric(double v, double limit)
{
  if (limit <= 0.0) {
    return v;
  }
  return std::clamp(v, -limit, limit);
}

// Command-interface index order. Must match command_interface_configuration().
// drive_side_pd:true claims all five; sim claims only effort at index 0.
constexpr std::size_t kCmdPosition = 0;
constexpr std::size_t kCmdVelocity = 1;
constexpr std::size_t kCmdEffort   = 2;
constexpr std::size_t kCmdKp       = 3;
constexpr std::size_t kCmdKd       = 4;
constexpr std::size_t kCmdSimEffort = 0;
}  // namespace

controller_interface::CallbackReturn PendulumPVTController::on_init()
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
    auto_declare<bool>("drive_side_pd", true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void PendulumPVTController::load_params()
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
  params_.drive_side_pd = node->get_parameter("drive_side_pd").as_bool();
}

controller_interface::CallbackReturn PendulumPVTController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  load_params();

  if (params_.joint.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joint' is required.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // The interface set is fixed at configure time; toggling drive_side_pd later
  // has no effect until the controller is re-spawned.
  drive_side_pd_ = params_.drive_side_pd;

  // comp_sign only negates the host feedforward. In drive_side_pd mode the
  // drive's internal Kp/Kd term still runs in the encoder's own sign, so a
  // flipped mounting must be corrected with negative EtherCAT factors instead.
  if (drive_side_pd_ && params_.comp_sign != 1.0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "comp_sign=%.3f has no effect on the drive-side PD term in drive_side_pd "
      "mode; correct a flipped mounting with negative EtherCAT factors instead.",
      params_.comp_sign);
  }

  setpoint_buf_.writeFromNonRT(Setpoint{params_.hold_position, 0.0, 0.0});

  auto node = get_node();
  setpoint_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    "~/setpoint", rclcpp::SystemDefaultsQoS(),
    std::bind(&PendulumPVTController::setpoint_callback, this, std::placeholders::_1));

  hold_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/hold",
    std::bind(&PendulumPVTController::hold_service, this,
              std::placeholders::_1, std::placeholders::_2));
  free_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/free",
    std::bind(&PendulumPVTController::free_service, this,
              std::placeholders::_1, std::placeholders::_2));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PendulumPVTController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (drive_side_pd_) {
    // Order locked — see kCmd* constants and update().
    cfg.names = {
      params_.joint + "/position",
      params_.joint + "/velocity",
      params_.joint + "/effort",
      params_.joint + "/kp",
      params_.joint + "/kd",
    };
  } else {
    cfg.names = {params_.joint + "/effort"};
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
PendulumPVTController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {params_.joint + "/position", params_.joint + "/velocity"};
  return cfg;
}

controller_interface::CallbackReturn PendulumPVTController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Refresh in case the user changed params between configure and activate.
  load_params();

  // Default to FREE on activation so a stale setpoint can't kick the joint.
  mode_.store(Mode::FREE);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PendulumPVTController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  write_free_outputs();
  return controller_interface::CallbackReturn::SUCCESS;
}

void PendulumPVTController::write_free_outputs()
{
  if (command_interfaces_.empty()) {
    return;
  }
  if (drive_side_pd_) {
    // kp=kd=0 collapses the drive's law to its feedforward term; effort=0 zeroes
    // that too. position=current q (vel=0) keeps the held target meaningless but
    // harmless even if a later kp write happens before the next FREE pass.
    double q = 0.0;
    if (state_interfaces_.size() > 0) {
      const auto pos_opt = state_interfaces_[0].get_optional();
      if (pos_opt) {
        q = pos_opt.value();
      }
    }
    (void)command_interfaces_[kCmdPosition].set_value(q);
    (void)command_interfaces_[kCmdVelocity].set_value(0.0);
    (void)command_interfaces_[kCmdEffort].set_value(0.0);
    (void)command_interfaces_[kCmdKp].set_value(0.0);
    (void)command_interfaces_[kCmdKd].set_value(0.0);
  } else {
    (void)command_interfaces_[kCmdSimEffort].set_value(0.0);
  }
}

controller_interface::return_type PendulumPVTController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Refresh gains & FF flags live — lets the user ramp Kp/Kd or toggle
  // ff_gravity via `ros2 param set` without re-spawning the controller.
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

  if (mode == Mode::FREE) {
    write_free_outputs();
    return controller_interface::return_type::OK;
  }

  // Host-side feedforward — gravity / inertia / viscous, gated by their flags.
  const double tau_g = params_.ff_gravity ? params_.mgl * std::sin(q) : 0.0;
  const double tau_J = params_.ff_inertia ? params_.J * sp.acceleration : 0.0;
  const double tau_v = params_.ff_viscous ? params_.Fv * qd : 0.0;
  const double tau_ff = params_.comp_sign * (tau_g + tau_J + tau_v);

  if (drive_side_pd_) {
    // Stream the PVT payload; the drive computes Kp*(p_des-q) + Kd*(v_des-qd) +
    // tau_ff internally. tau_limit clamps the feedforward only — the hard limit
    // is the 0x6072 max_torque SDO in the EtherCAT slave config.
    (void)command_interfaces_[kCmdPosition].set_value(sp.position);
    (void)command_interfaces_[kCmdVelocity].set_value(sp.velocity);
    (void)command_interfaces_[kCmdEffort].set_value(clamp_symmetric(tau_ff, params_.tau_limit));
    (void)command_interfaces_[kCmdKp].set_value(params_.Kp);
    (void)command_interfaces_[kCmdKd].set_value(params_.Kd);
  } else {
    // Sim — replicate the drive's MIT law in software and write effort only.
    const double tau_pd =
      params_.Kp * (sp.position - q) + params_.Kd * (sp.velocity - qd);
    const double tau = clamp_symmetric(tau_ff + tau_pd, params_.tau_limit);
    (void)command_interfaces_[kCmdSimEffort].set_value(tau);
  }

  return controller_interface::return_type::OK;
}

void PendulumPVTController::setpoint_callback(
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  Setpoint sp{params_.hold_position, 0.0, 0.0};
  if (!msg->positions.empty())     sp.position     = msg->positions[0];
  if (!msg->velocities.empty())    sp.velocity     = msg->velocities[0];
  if (!msg->accelerations.empty()) sp.acceleration = msg->accelerations[0];
  setpoint_buf_.writeFromNonRT(sp);
  // A streaming setpoint implies the caller wants tracking — switch to PVT.
  mode_.store(Mode::PVT);
}

void PendulumPVTController::hold_service(
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
  mode_.store(Mode::PVT);
  response->success = true;
  response->message = "Holding at q = " + std::to_string(sp.position);
}

void PendulumPVTController::free_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  mode_.store(Mode::FREE);
  response->success = true;
  response->message = "FREE mode — zero drive torque";
}

}  // namespace pendulum_pvt_control

PLUGINLIB_EXPORT_CLASS(
  pendulum_pvt_control::PendulumPVTController,
  controller_interface::ControllerInterface)
