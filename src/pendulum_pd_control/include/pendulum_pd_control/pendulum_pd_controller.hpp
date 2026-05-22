#ifndef PENDULUM_PD_CONTROL__PENDULUM_PD_CONTROLLER_HPP_
#define PENDULUM_PD_CONTROL__PENDULUM_PD_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "pendulum_safety/clamp.hpp"
#include "pendulum_safety/estop_subscriber.hpp"
#include "pendulum_safety/param_loader.hpp"
#include "pendulum_safety/safety_limits.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace pendulum_pd_control
{

enum class Mode : uint8_t
{
  FREE = 0,   // write zero effort
  TUNE = 1,   // feedforward only (no PD)
  PD   = 2,   // feedforward + PD (production)
};

class PendulumPDController : public controller_interface::ControllerInterface
{
public:
  PendulumPDController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  struct Setpoint
  {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
  };

  // Loaded once in on_configure, refreshed each update() from rclcpp params.
  struct Params
  {
    std::string joint;
    double Kp{0.0};
    double Kd{0.0};
    double mgl{0.0};
    double J{0.0};
    double Fv{0.0};
    double comp_sign{1.0};
    bool ff_gravity{true};
    bool ff_inertia{true};
    bool ff_viscous{false};
    double hold_position{0.0};
  };

  void load_params();
  void setpoint_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
  void hold_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void free_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  Params params_;

  // Centralised safety: limits loaded once in on_configure; the supervisor's
  // e-stop / Kp-derate signal read every update().
  pendulum_safety::SafetyLimits limits_;
  pendulum_safety::EstopSubscriber safety_;
  double estop_hold_pos_{0.0};   // joint position snapshotted when e-stop fires
  bool estop_was_active_{false};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr setpoint_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr free_srv_;

  realtime_tools::RealtimeBuffer<Setpoint> setpoint_buf_;
  std::atomic<Mode> mode_{Mode::FREE};
};

}  // namespace pendulum_pd_control

#endif  // PENDULUM_PD_CONTROL__PENDULUM_PD_CONTROLLER_HPP_
