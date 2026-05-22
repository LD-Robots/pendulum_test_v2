#ifndef PENDULUM_SAFETY__ESTOP_SUBSCRIBER_HPP_
#define PENDULUM_SAFETY__ESTOP_SUBSCRIBER_HPP_

#include <atomic>
#include <cstdint>

#include "pendulum_safety/safety_limits.hpp"
#include "rclcpp/rclcpp.hpp"
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

/// Consumer-side helper shared by the PD and PVT control paths. Owns the two
/// latched subscriptions the supervisor publishes and stores their values in
/// lock-free atomics so update() can read the current SafetySignal with no
/// allocation and no locking. Topics:
///   /pendulum/safety/estop_state  (std_msgs/Int8:    0 clear, 1 FREE, 2 HOLD)
///   /pendulum/safety/kp_scale     (std_msgs/Float64: thermal Kp multiplier)
///
/// When no supervisor is running the topics have no publisher; the atomics
/// keep their safe defaults (no e-stop, kp_scale 1.0), so a control path
/// behaves exactly as it did before the safety package existed.
class EstopSubscriber
{
public:
  EstopSubscriber() = default;

  /// Create the subscriptions on `node`. Safe to call from on_configure().
  /// NodeT is any handle exposing create_subscription — a controller's
  /// get_node() lifecycle handle or a plain rclcpp::Node.
  template<typename NodeT>
  void subscribe(NodeT node)
  {
    const auto latched = rclcpp::QoS(1).transient_local();

    estop_sub_ = node->template create_subscription<std_msgs::msg::Int8>(
      "/pendulum/safety/estop_state", latched,
      [this](std_msgs::msg::Int8::SharedPtr msg) {estop_state_.store(msg->data);});

    kp_scale_sub_ = node->template create_subscription<std_msgs::msg::Float64>(
      "/pendulum/safety/kp_scale", latched,
      [this](std_msgs::msg::Float64::SharedPtr msg) {kp_scale_.store(msg->data);});
  }

  /// Current safety signal — realtime-safe, call from update().
  SafetySignal get() const
  {
    SafetySignal signal;
    const int8_t state = estop_state_.load();
    signal.estop_active = (state != 0);
    signal.action = (state == 2) ? EstopAction::HOLD : EstopAction::FREE;
    signal.kp_scale = kp_scale_.load();
    return signal;
  }

private:
  std::atomic<int8_t> estop_state_{0};
  std::atomic<double> kp_scale_{1.0};
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr kp_scale_sub_;
};

}  // namespace pendulum_safety

#endif  // PENDULUM_SAFETY__ESTOP_SUBSCRIBER_HPP_
