#ifndef PENDULUM_PD_CONTROL__DRIVE_STATUS_BROADCASTER_HPP_
#define PENDULUM_PD_CONTROL__DRIVE_STATUS_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pendulum_pd_control
{

// Broadcaster for myActuator drive telemetry (TxPDO 0x1A02 on the 2026-04-24
// firmware). Claims a configurable set of state interfaces, republishes each
// as std_msgs/Float64 and bundles them into one thresholded
// diagnostic_msgs/DiagnosticArray. No command interfaces — pure broadcaster.
class DriveStatusBroadcaster : public controller_interface::ControllerInterface
{
public:
  DriveStatusBroadcaster() = default;

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
  using Float64Publisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64>;
  using DiagPublisher = realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>;

  struct Thresholds
  {
    double motor_temp_warn{70.0};
    double motor_temp_error{90.0};
    double drive_temp_warn{70.0};
    double drive_temp_error{85.0};
    double bus_voltage_min{40.0};
    double bus_voltage_max{54.0};
  };

  std::string joint_;
  std::vector<std::string> signals_;
  double publish_rate_{20.0};
  Thresholds thr_;

  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> signal_pubs_;
  std::vector<std::unique_ptr<Float64Publisher>> signal_rt_pubs_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::unique_ptr<DiagPublisher> diag_rt_pub_;
};

}  // namespace pendulum_pd_control

#endif  // PENDULUM_PD_CONTROL__DRIVE_STATUS_BROADCASTER_HPP_
