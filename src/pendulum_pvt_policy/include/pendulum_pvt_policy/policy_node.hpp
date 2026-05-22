#ifndef PENDULUM_PVT_POLICY__POLICY_NODE_HPP_
#define PENDULUM_PVT_POLICY__POLICY_NODE_HPP_

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <onnxruntime_cxx_api.h>

namespace pendulum_pvt_policy
{

/// ONNX inference node for the PVT (Mode 5) target-tracking policy.
///
/// Runs the trained policy and feeds the PendulumPVTController via its
/// `~/setpoint` topic. The drive firmware computes the PD torque at 1 kHz;
/// this node only decides WHERE the joint should go.
///
/// Two timers:
///   - inference @ 50 Hz : run ONNX, compute target_pd goal (matches the
///                          training cadence — 50 Hz policy decisions).
///   - output @ ~200 Hz  : slew the published setpoint toward the goal at
///                          `target_pd_slew_rate` and publish. Decoupling the
///                          output from inference keeps the drive from seeing
///                          50 Hz step pulses (audible torque noise).
///
/// Observation (8-dim, matches training env_cfg.py):
///   [0]    (user_target − q) / π    linear error, NOT wrapped
///   [1]    q_dot / VELOCITY_LIMIT   (16.0 rad/s)
///   [2]    sin(q)
///   [3]    cos(q)
///   [4..7] prev_action history (4 raw actions, oldest first)
///
/// Action mapping (training scale=2π/3, clip=3):
///   target_pd = (2π/3) · raw_action + user_target
///
/// Safety:
///   - Fall e-stop if |q_dot| > fall_vel_limit OR |q − default_pos| > fall_pos_limit;
///     on fall, the last good setpoint is republished so the drive holds position.
///   - /joint_states freshness watchdog.
class PvtPolicyNode : public rclcpp::Node
{
public:
  explicit PvtPolicyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onTarget(const std_msgs::msg::Float64::SharedPtr msg);
  void onInferenceTick();   // 50 Hz
  void onOutputTick();      // ~200 Hz
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

  // Helpers
  void buildObservation(double pos, double vel);
  float runInference();
  void publishSetpoint(double target_pd);

  static constexpr double kVelocityLimit = 16.0;
  static constexpr double kErrScale = 3.14159265358979323846;  // π

  // --- Config (ROS params) ---
  std::string joint_name_;
  double action_scale_{2.0 * 3.14159265358979323846 / 3.0};   // 2π/3
  double clip_actions_{3.0};
  bool debug_{false};   // when true, log the observation vector each inference


  double joint_state_timeout_sec_{0.05};
  double output_dt_{1.0 / 200.0};

  // --- Slew rate limiter ---
  std::atomic<double> target_pd_slew_rate_{5.5};
  std::atomic<double> target_pd_goal_{0.0};
  std::atomic<bool> have_target_pd_goal_{false};
  double last_published_target_pd_{std::numeric_limits<double>::quiet_NaN()};

  // --- Fall safety ---
  std::atomic<bool> policy_enabled_{true};
  std::atomic<double> default_pos_{0.0};
  double fall_vel_limit_{20.0};
  double fall_pos_limit_{10.0};
  std::atomic<bool> fall_latched_{false};

  // --- Target ---
  std::atomic<double> user_target_{0.0};
  std::atomic<double> policy_target_{0.0};

  // --- ONNX ---
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  Ort::MemoryInfo ort_mem_info_{
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  // --- Observation ---
  std::array<float, 8> obs_{};
  std::array<float, 4> action_history_{};   // oldest [0], newest [3]
  std::atomic<float> last_raw_action_{0.0f};

  // --- Joint state ---
  std::atomic<double> last_pos_{0.0};
  std::atomic<double> last_vel_{0.0};
  std::atomic<bool> have_joint_state_{false};
  rclcpp::Time last_js_stamp_;
  std::mutex stamp_mutex_;

  // --- ROS I/O ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_pd_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fall_pub_;
  rclcpp::TimerBase::SharedPtr inference_timer_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace pendulum_pvt_policy

#endif  // PENDULUM_PVT_POLICY__POLICY_NODE_HPP_
