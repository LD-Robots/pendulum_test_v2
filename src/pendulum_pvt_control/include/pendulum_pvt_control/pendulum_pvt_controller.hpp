#ifndef PENDULUM_PVT_CONTROL__PENDULUM_PVT_CONTROLLER_HPP_
#define PENDULUM_PVT_CONTROL__PENDULUM_PVT_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace pendulum_pvt_control
{

enum class Mode : uint8_t
{
  FREE = 0,   // neutral output — drive produces zero torque
  PVT  = 1,   // active — track the streaming setpoint
};

class PendulumPVTController : public controller_interface::ControllerInterface
{
public:
  PendulumPVTController() = default;

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
    double tau_limit{0.0};
    double mgl{0.0};
    double J{0.0};
    double Fv{0.0};
    double comp_sign{1.0};
    bool ff_gravity{true};
    bool ff_inertia{true};
    bool ff_viscous{false};
    double hold_position{0.0};
    // true  -> real hardware: stream pos/vel/effort/kp/kd, drive runs the PD law.
    // false -> Gazebo sim: claim only effort, run the PD law in software.
    bool drive_side_pd{true};
  };

  void load_params();
  // Write a safe neutral command to every claimed command interface. In
  // drive_side_pd mode that means kp=kd=effort=velocity=0 and position=q so the
  // drive's law collapses to zero torque; in sim it means effort=0.
  void write_free_outputs();
  void setpoint_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
  void hold_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void free_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  Params params_;
  // Cached from params_ in on_configure — command_interface_configuration() runs
  // before activation and must agree with the index order used in update().
  bool drive_side_pd_{true};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr setpoint_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr free_srv_;

  realtime_tools::RealtimeBuffer<Setpoint> setpoint_buf_;
  std::atomic<Mode> mode_{Mode::FREE};
};

}  // namespace pendulum_pvt_control

#endif  // PENDULUM_PVT_CONTROL__PENDULUM_PVT_CONTROLLER_HPP_
