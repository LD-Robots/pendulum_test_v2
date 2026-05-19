#include "pendulum_pvt_policy/policy_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace pendulum_pvt_policy
{

using namespace std::chrono_literals;

PvtPolicyNode::PvtPolicyNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pendulum_pvt_policy", options),
  last_js_stamp_(0, 0, RCL_ROS_TIME)
{
  // --- ONNX + joint ---
  const std::string onnx_path = declare_parameter<std::string>("onnx_path", "");
  joint_name_ = declare_parameter<std::string>("joint_name", "pendulum_joint");

  // --- Action / policy (match training env_cfg.py) ---
  action_scale_ = declare_parameter<double>("action_scale", 2.0 * M_PI / 3.0);
  clip_actions_ = declare_parameter<double>("clip_actions", 3.0);
  const double inference_rate_hz =
    declare_parameter<double>("inference_rate_hz", 50.0);
  const double output_rate_hz =
    declare_parameter<double>("output_rate_hz", 200.0);
  output_dt_ = 1.0 / output_rate_hz;

  target_pd_slew_rate_.store(declare_parameter<double>("target_pd_slew_rate", 5.5));
  policy_enabled_.store(declare_parameter<bool>("policy_enabled", true));
  default_pos_.store(declare_parameter<double>("default_pos", 0.0));

  // --- Fall safety ---
  fall_vel_limit_ = declare_parameter<double>("fall_vel_limit", 20.0);
  fall_pos_limit_ = declare_parameter<double>("fall_pos_limit", 10.0);

  // --- Topics ---
  joint_state_timeout_sec_ =
    declare_parameter<double>("joint_state_timeout_sec", 0.05);
  const std::string joint_state_topic =
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
  const std::string target_topic =
    declare_parameter<std::string>("target_topic", "/pendulum/target");
  const std::string fall_topic =
    declare_parameter<std::string>("fall_topic", "/pendulum/fall_latched");
  // Setpoint goes to the PendulumPVTController's ~/setpoint topic.
  const std::string setpoint_topic = declare_parameter<std::string>(
    "setpoint_topic", "/pendulum_pvt_controller/setpoint");

  user_target_.store(declare_parameter<double>("target_pos", default_pos_.load()));
  policy_target_.store(user_target_.load());

  if (onnx_path.empty()) {
    throw std::runtime_error("onnx_path parameter is required");
  }

  ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PvtPolicy");
  Ort::SessionOptions opts;
  opts.SetIntraOpNumThreads(1);
  ort_session_ = std::make_unique<Ort::Session>(*ort_env_, onnx_path.c_str(), opts);
  RCLCPP_INFO(get_logger(), "Loaded ONNX model: %s", onnx_path.c_str());

  // Publishers
  setpoint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
    setpoint_topic, 10);
  target_pd_debug_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/pendulum/target_pd", 10);
  fall_pub_ = create_publisher<std_msgs::msg::Bool>(
    fall_topic, rclcpp::QoS(1).transient_local());

  // Subscribers
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic, rclcpp::SensorDataQoS(),
    std::bind(&PvtPolicyNode::onJointState, this, std::placeholders::_1));
  target_sub_ = create_subscription<std_msgs::msg::Float64>(
    target_topic, 10,
    std::bind(&PvtPolicyNode::onTarget, this, std::placeholders::_1));

  param_cb_ = add_on_set_parameters_callback(
    std::bind(&PvtPolicyNode::onParamChange, this, std::placeholders::_1));

  // Timers
  const auto inf_period =
    std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / inference_rate_hz));
  inference_timer_ = create_wall_timer(
    inf_period, std::bind(&PvtPolicyNode::onInferenceTick, this));

  const auto out_period =
    std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / output_rate_hz));
  output_timer_ = create_wall_timer(
    out_period, std::bind(&PvtPolicyNode::onOutputTick, this));

  RCLCPP_INFO(get_logger(),
    "PVT policy ready: joint=%s action_scale=%.4f clip=%.2f "
    "inference=%.0fHz output=%.0fHz slew=%.2f setpoint_topic=%s",
    joint_name_.c_str(), action_scale_, clip_actions_,
    inference_rate_hz, output_rate_hz, target_pd_slew_rate_.load(),
    setpoint_topic.c_str());
}

void PvtPolicyNode::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == joint_name_) {
      last_pos_.store((i < msg->position.size()) ? msg->position[i] : 0.0);
      last_vel_.store((i < msg->velocity.size()) ? msg->velocity[i] : 0.0);
      {
        std::lock_guard<std::mutex> lock(stamp_mutex_);
        last_js_stamp_ =
          (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
            ? now()
            : rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
      }
      have_joint_state_.store(true);
      return;
    }
  }
}

void PvtPolicyNode::onTarget(const std_msgs::msg::Float64::SharedPtr msg)
{
  user_target_.store(msg->data);
  RCLCPP_INFO(get_logger(), "New user target: %.4f rad", msg->data);
}

rcl_interfaces::msg::SetParametersResult PvtPolicyNode::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    if (p.get_name() == "policy_enabled") {
      policy_enabled_.store(p.as_bool());
    } else if (p.get_name() == "default_pos") {
      default_pos_.store(p.as_double());
    } else if (p.get_name() == "target_pos") {
      user_target_.store(p.as_double());
    } else if (p.get_name() == "target_pd_slew_rate") {
      target_pd_slew_rate_.store(p.as_double());
    }
  }
  return result;
}

void PvtPolicyNode::buildObservation(double pos, double vel)
{
  const double err = user_target_.load() - pos;
  obs_[0] = static_cast<float>(err / kErrScale);
  obs_[1] = static_cast<float>(vel / kVelocityLimit);
  obs_[2] = static_cast<float>(std::sin(pos));
  obs_[3] = static_cast<float>(std::cos(pos));
  obs_[4] = action_history_[0];
  obs_[5] = action_history_[1];
  obs_[6] = action_history_[2];
  obs_[7] = action_history_[3];
}

float PvtPolicyNode::runInference()
{
  std::array<int64_t, 2> input_shape = {1, static_cast<int64_t>(obs_.size())};
  auto input_tensor = Ort::Value::CreateTensor<float>(
    ort_mem_info_, obs_.data(), obs_.size(),
    input_shape.data(), input_shape.size());

  const char * input_names[] = {"obs"};
  const char * output_names[] = {"actions"};

  auto output_tensors = ort_session_->Run(
    Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

  float action = output_tensors[0].GetTensorMutableData<float>()[0];
  return std::clamp(action,
    static_cast<float>(-clip_actions_), static_cast<float>(clip_actions_));
}

void PvtPolicyNode::publishSetpoint(double target_pd)
{
  trajectory_msgs::msg::JointTrajectoryPoint msg;
  msg.positions = {target_pd};
  // velocity / acceleration left empty — training used v_des=0; the controller
  // defaults missing fields to 0.
  setpoint_pub_->publish(msg);

  std_msgs::msg::Float64 dbg;
  dbg.data = target_pd;
  target_pd_debug_pub_->publish(dbg);
}

void PvtPolicyNode::onInferenceTick()
{
  if (!have_joint_state_.load()) {
    return;
  }
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(stamp_mutex_);
    stamp = last_js_stamp_;
  }
  if ((now() - stamp).seconds() > joint_state_timeout_sec_) {
    return;
  }

  const double pos = last_pos_.load();
  const double vel = last_vel_.load();

  // --- Fall safety ---
  if (!fall_latched_.load()) {
    const bool overspeed = std::abs(vel) > fall_vel_limit_;
    const bool offpos = std::abs(pos - default_pos_.load()) > fall_pos_limit_;
    if (overspeed || offpos) {
      fall_latched_.store(true);
      std_msgs::msg::Bool latched;
      latched.data = true;
      fall_pub_->publish(latched);
      RCLCPP_ERROR(get_logger(),
        "FALL e-stop: vel=%.2f pos=%.3f (default=%.3f) — holding last target.",
        vel, pos, default_pos_.load());
    }
  }
  if (fall_latched_.load()) {
    return;  // output tick keeps republishing the held setpoint
  }

  // --- Inference ---
  buildObservation(pos, vel);
  const float raw_action = policy_enabled_.load() ? runInference() : 0.0f;
  last_raw_action_.store(raw_action);

  action_history_[0] = action_history_[1];
  action_history_[1] = action_history_[2];
  action_history_[2] = action_history_[3];
  action_history_[3] = raw_action;

  const double target_pd_goal =
    user_target_.load() + action_scale_ * static_cast<double>(raw_action);
  target_pd_goal_.store(target_pd_goal);
  have_target_pd_goal_.store(true);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
    "obs=[err/π=%.3f vel/16=%.3f sin=%.3f cos=%.3f "
    "a-3=%.3f a-2=%.3f a-1=%.3f a0=%.3f] -> raw_a=%.4f "
    "(pos=%.4f vel=%.4f utgt=%.4f goal=%.4f)",
    obs_[0], obs_[1], obs_[2], obs_[3],
    obs_[4], obs_[5], obs_[6], obs_[7],
    raw_action, pos, vel, user_target_.load(), target_pd_goal);
}

void PvtPolicyNode::onOutputTick()
{
  if (!have_target_pd_goal_.load()) {
    return;
  }
  if (fall_latched_.load()) {
    // Hold the last good setpoint so the drive keeps position.
    publishSetpoint(policy_target_.load());
    return;
  }

  const double goal = target_pd_goal_.load();

  double target_pd;
  const double slew_rate = target_pd_slew_rate_.load();
  if (std::isnan(last_published_target_pd_) || slew_rate <= 0.0) {
    target_pd = goal;
  } else {
    const double max_step = slew_rate * output_dt_;
    const double delta = goal - last_published_target_pd_;
    target_pd = last_published_target_pd_ + std::clamp(delta, -max_step, max_step);
  }
  last_published_target_pd_ = target_pd;
  policy_target_.store(target_pd);

  publishSetpoint(target_pd);
}

}  // namespace pendulum_pvt_policy
