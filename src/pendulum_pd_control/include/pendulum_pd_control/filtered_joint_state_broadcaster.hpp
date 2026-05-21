#ifndef PENDULUM_PD_CONTROL__FILTERED_JOINT_STATE_BROADCASTER_HPP_
#define PENDULUM_PD_CONTROL__FILTERED_JOINT_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace pendulum_pd_control
{

// Broadcaster that low-pass-filters the joint velocity and effort and publishes
// the result as a sensor_msgs/JointState. The myActuator firmware computes its
// own "Filtered Velocity" (0x200D) and "Filtered Torque" (0x200F), but never
// packs them into a TxPDO, so they cannot be streamed to the host. This
// controller reproduces them host-side from the raw velocity (0x606C) and
// effort (0x6077) that TxPDO 0x1A02 does carry. Position is passed through
// unfiltered. Pure broadcaster — no command interfaces.
class FilteredJointStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  FilteredJointStateBroadcaster() = default;

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
  using JointStatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;

  std::string joint_;
  std::string topic_;
  double velocity_cutoff_hz_{30.0};
  double torque_cutoff_hz_{30.0};
  double publish_rate_{100.0};

  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  // First-order IIR low-pass state, seeded on the first finite sample.
  bool filter_initialized_{false};
  double vel_filt_{0.0};
  double tau_filt_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<JointStatePublisher> rt_joint_state_pub_;
};

}  // namespace pendulum_pd_control

#endif  // PENDULUM_PD_CONTROL__FILTERED_JOINT_STATE_BROADCASTER_HPP_
