#ifndef PENDULUM_SAFETY__ESTOP_SUBSCRIBER_HPP_
#define PENDULUM_SAFETY__ESTOP_SUBSCRIBER_HPP_

#include "pendulum_safety/safety_limits.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"

namespace pendulum_safety
{

/// Snapshot of the supervisor's safety signal, as seen by a control path.
struct SafetySignal
{
  bool        estop_active = false;
  EstopAction action       = EstopAction::FREE;
  double      kp_scale     = 1.0;
};

/// Consumer-side helper shared by the PD, PVT and policy control paths. Owns
/// the two latched subscriptions the supervisor publishes and funnels them
/// into a realtime buffer so update() can read the current SafetySignal
/// lock-free. Topics:
///   /pendulum/safety/estop_state  (std_msgs/Int8:    0 clear, 1 FREE, 2 HOLD)
///   /pendulum/safety/kp_scale     (std_msgs/Float64: thermal Kp multiplier)
///
/// Both subscription callbacks run on the consumer node's executor thread, so
/// the read-modify-write into the buffer is serialised.
class EstopSubscriber
{
public:
  EstopSubscriber() = default;

  /// Create the subscriptions on `node`. Safe to call from on_configure() /
  /// the node constructor. NodeT is any handle exposing create_subscription
  /// (rclcpp::Node or a ros2_control controller's lifecycle node handle).
  template<typename NodeT>
  void subscribe(NodeT node)
  {
    buffer_.writeFromNonRT(SafetySignal{});

    const auto latched = rclcpp::QoS(1).transient_local();

    estop_sub_ = node->template create_subscription<std_msgs::msg::Int8>(
      "/pendulum/safety/estop_state", latched,
      [this](std_msgs::msg::Int8::SharedPtr msg) {
        SafetySignal sig = *buffer_.readFromNonRT();
        sig.estop_active = msg->data != 0;
        sig.action = (msg->data == 2) ? EstopAction::HOLD : EstopAction::FREE;
        buffer_.writeFromNonRT(sig);
      });

    kp_scale_sub_ = node->template create_subscription<std_msgs::msg::Float64>(
      "/pendulum/safety/kp_scale", latched,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        SafetySignal sig = *buffer_.readFromNonRT();
        sig.kp_scale = msg->data;
        buffer_.writeFromNonRT(sig);
      });
  }

  /// Current safety signal — realtime-safe, call from update().
  SafetySignal get() { return *buffer_.readFromRT(); }

private:
  realtime_tools::RealtimeBuffer<SafetySignal> buffer_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr kp_scale_sub_;
};

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__ESTOP_SUBSCRIBER_HPP_
