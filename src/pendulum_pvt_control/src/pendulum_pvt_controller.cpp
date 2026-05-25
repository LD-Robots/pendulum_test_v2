#include "pendulum_pvt_control/pendulum_pvt_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace pendulum_pvt_control
{

namespace
{
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
    auto_declare<double>("mgl", 0.0);
    auto_declare<double>("J", 0.0);
    auto_declare<double>("Fv", 0.0);
    auto_declare<double>("comp_sign", 1.0);
    auto_declare<bool>("ff_gravity", true);
    auto_declare<bool>("ff_inertia", true);
    auto_declare<bool>("ff_viscous", false);
    auto_declare<double>("hold_position", 0.0);
    auto_declare<bool>("drive_side_pd", true);
    auto_declare<double>("lag_free", 0.04);
    auto_declare<double>("lag_pause", 0.14);
    auto_declare<double>("alpha_slew", 2.0);
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
  params_.mgl           = node->get_parameter("mgl").as_double();
  params_.J             = node->get_parameter("J").as_double();
  params_.Fv            = node->get_parameter("Fv").as_double();
  params_.comp_sign     = node->get_parameter("comp_sign").as_double();
  params_.ff_gravity    = node->get_parameter("ff_gravity").as_bool();
  params_.ff_inertia    = node->get_parameter("ff_inertia").as_bool();
  params_.ff_viscous    = node->get_parameter("ff_viscous").as_bool();
  params_.hold_position = node->get_parameter("hold_position").as_double();
  params_.drive_side_pd = node->get_parameter("drive_side_pd").as_bool();
  params_.lag_free      = node->get_parameter("lag_free").as_double();
  params_.lag_pause     = node->get_parameter("lag_pause").as_double();
  params_.alpha_slew    = node->get_parameter("alpha_slew").as_double();
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

  // Centralised safety: load the shared limits and subscribe to the
  // supervisor's e-stop / Kp-derate signal.
  limits_ = pendulum_safety::loadSafetyLimits(*get_node(), "safety.");
  safety_.subscribe(get_node());

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

  // Action server. get_node() is lifecycle-backed, so pass the interface
  // pointers explicitly. action callbacks land on the controller_manager
  // executor thread, the same one that runs setpoint_callback / hold_service /
  // free_service — so all non-RT writes to controller state are serialised.
  traj_buf_.writeFromNonRT(nullptr);
  sampled_state_.writeFromNonRT({0.0, 0.0, 0.0, 0.0});
  action_server_ = rclcpp_action::create_server<FJT>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "~/follow_joint_trajectory",
    [this](const rclcpp_action::GoalUUID & uuid,
           std::shared_ptr<const FJT::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleFJT> gh) {
      return this->handle_cancel(gh);
    },
    [this](const std::shared_ptr<GoalHandleFJT> gh) {
      this->handle_accepted(gh);
    });

  // 10 Hz feedback / completion poll. Cheap; only does work while a goal is
  // active. Running on the executor means it serialises with action callbacks.
  feedback_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { this->on_feedback_tick(); });

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
  // Tear down any in-flight goal before zeroing outputs so the client sees a
  // clean abort rather than a silently-dropped goal handle.
  preempt_goal("controller deactivating");
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Refresh gains & FF flags live — lets the user ramp Kp/Kd or toggle
  // ff_gravity via `ros2 param set` without re-spawning the controller.
  load_params();

  const pendulum_safety::SafetySignal safety = safety_.get();
  const auto mode = mode_.load();
  const Setpoint sp = *setpoint_buf_.readFromRT();

  const auto pos_opt = state_interfaces_[0].get_optional();
  const auto vel_opt = state_interfaces_[1].get_optional();
  if (!pos_opt || !vel_opt) {
    return controller_interface::return_type::OK;
  }
  const double q  = pos_opt.value();
  const double qd = vel_opt.value();

  // First cycle of an e-stop: pick the HOLD target. The joint may be moving
  // fast (e.g. an overspeed trip), so a HOLD targets a *braking distance*
  // ahead and seeds the rate limiter at the joint's actual (position,
  // velocity) — the command then becomes a smooth deceleration ramp to rest.
  // Pinning the instantaneous position would make the drive fight the joint's
  // momentum and ring.
  if (safety.estop_active && !estop_was_active_) {
    if (safety.action == pendulum_safety::EstopAction::HOLD) {
      const double accel = limits_.acceleration_limit > 0.0
        ? limits_.acceleration_limit : 60.0;
      double v0 = qd;
      if (limits_.slew_rate_limit > 0.0) {
        v0 = std::clamp(qd, -limits_.slew_rate_limit, limits_.slew_rate_limit);
      }
      const double brake = (v0 * v0) / (2.0 * accel);
      estop_hold_pos_ = pendulum_safety::clampPosition(
        q + std::copysign(brake, v0), limits_);
      rate_limiter_.seed(q, v0);
    } else {
      estop_hold_pos_ = pendulum_safety::clampPosition(q, limits_);
    }
  }
  // Falling edge — the e-stop just cleared. Hold at the current joint
  // position until a fresh ~/setpoint arrives, so the controller does not
  // resume toward a stale buffered point. setpoint_callback / hold_service
  // clear waiting_for_setpoint_ when a fresh setpoint comes in.
  if (!safety.estop_active && estop_was_active_) {
    resume_hold_pos_ = pendulum_safety::clampPosition(q, limits_);
    rate_limiter_.seed(q);
    waiting_for_setpoint_.store(true);
    // Any action trajectory in flight was already overridden by the e-stop
    // (the safety branch below picks estop_hold_pos_); now drop the trajectory
    // so the controller does not resume sampling it post-reset, and signal the
    // feedback timer to abort the active goal handle (traj_completed_clean_
    // stays false, so the feedback timer routes the termination to abort()).
    traj_buf_.writeFromNonRT(nullptr);
    traj_done_.store(true);
  }
  estop_was_active_ = safety.estop_active;

  // E-stop FREE, or the controller's own FREE mode when no e-stop overrides
  // it, drives the neutral zero-torque output. An e-stop HOLD takes precedence
  // over FREE mode — safety must hold the joint regardless of controller mode.
  const bool estop_free =
    safety.estop_active && safety.action == pendulum_safety::EstopAction::FREE;
  if (estop_free || (mode == Mode::FREE && !safety.estop_active)) {
    rate_limiter_.reset();
    write_free_outputs();
    return controller_interface::return_type::OK;
  }

  // Reference: an active e-stop HOLD overrides the streamed setpoint; after a
  // reset we hold at resume_hold_pos_ until a fresh ~/setpoint arrives, so
  // the joint does not lunge to a stale buffered point. With an action goal
  // active and no safety override, sample the trajectory in-RT with the lag
  // governor; otherwise fall through to the legacy ~/setpoint buffer.
  double ref_pos, ref_vel, ref_acc;
  if (safety.estop_active) {   // HOLD — FREE already returned above
    ref_pos = estop_hold_pos_;
    ref_vel = 0.0;
    ref_acc = 0.0;
  } else if (waiting_for_setpoint_.load()) {
    ref_pos = resume_hold_pos_;
    ref_vel = 0.0;
    ref_acc = 0.0;
  } else if (auto traj_ptr = *traj_buf_.readFromRT(); traj_ptr && !traj_done_.load()) {
    // Sample at the current virtual time *before* advancing tv_, so the lag
    // governor measures lead against the reference about to be commanded.
    double p_ref, v_ref, a_ref;
    sample_trajectory(*traj_ptr, tv_, p_ref, v_ref, a_ref);

    // Lag governor — port of pvt_goto.py:174-189. `lead` is how far the
    // reference is ahead of the joint in the direction of net travel; the
    // time-scale alpha goes to 0 at lag_pause and stays at 1 below lag_free.
    const double dt = period.seconds();
    const double lead = (p_ref - q) * dq_sign_;
    const double denom = std::max(params_.lag_pause - params_.lag_free, 1e-6);
    double alpha_target = (params_.lag_pause - lead) / denom;
    alpha_target = std::clamp(alpha_target, 0.0, 1.0);
    const double max_step = params_.alpha_slew * dt;
    alpha_ += std::clamp(alpha_target - alpha_, -max_step, +max_step);
    alpha_ = std::clamp(alpha_, 0.0, 1.0);

    // Advance the virtual clock; chain-rule v and a for the scaled time.
    tv_ = std::min(tv_ + alpha_ * dt, traj_ptr->duration);
    ref_pos = p_ref;
    ref_vel = v_ref * alpha_;
    ref_acc = a_ref * alpha_ * alpha_;

    sampled_state_.writeFromNonRT({ref_pos, ref_vel, ref_acc, q});

    if (tv_ >= traj_ptr->duration) {
      traj_completed_clean_.store(true);
      traj_done_.store(true);
    }
  } else {
    ref_pos = sp.position;
    ref_vel = sp.velocity;
    ref_acc = sp.acceleration;
  }

  // Slew- and acceleration-limit the position command, then clamp it and the
  // velocity command to the software limits. Re-seed the limiter from the
  // measured q on every (re)entry from FREE so the resumed motion has no jump.
  if (!rate_limiter_.seeded()) {
    rate_limiter_.seed(q);
  }
  double p_cmd = rate_limiter_.limit(
    ref_pos, period.seconds(), limits_.slew_rate_limit,
    limits_.acceleration_limit);
  p_cmd = pendulum_safety::clampPosition(p_cmd, limits_);
  const double v_cmd = pendulum_safety::clampVelocity(ref_vel, limits_);

  // Host-side feedforward — gravity / inertia / viscous, gated by their flags.
  const double tau_g = params_.ff_gravity ? params_.mgl * std::sin(q) : 0.0;
  const double tau_J = params_.ff_inertia ? params_.J * ref_acc : 0.0;
  const double tau_v = params_.ff_viscous ? params_.Fv * qd : 0.0;
  const double tau_ff = params_.comp_sign * (tau_g + tau_J + tau_v);

  // Thermal Kp derating from the supervisor (1.0 when no supervisor / cool).
  const double kp_eff = params_.Kp * safety.kp_scale;

  if (drive_side_pd_) {
    // Stream the PVT payload; the drive computes Kp*(p_des-q) + Kd*(v_des-qd) +
    // tau_ff internally. clampEffort caps the host feedforward — the hard limit
    // is the 0x6072 max_torque SDO in the EtherCAT slave config.
    (void)command_interfaces_[kCmdPosition].set_value(p_cmd);
    (void)command_interfaces_[kCmdVelocity].set_value(v_cmd);
    (void)command_interfaces_[kCmdEffort].set_value(
      pendulum_safety::clampEffort(tau_ff, limits_));
    (void)command_interfaces_[kCmdKp].set_value(kp_eff);
    (void)command_interfaces_[kCmdKd].set_value(params_.Kd);
  } else {
    // Sim — replicate the drive's MIT law in software and write effort only.
    const double tau_pd = kp_eff * (p_cmd - q) + params_.Kd * (v_cmd - qd);
    const double tau = pendulum_safety::clampEffort(tau_ff + tau_pd, limits_);
    (void)command_interfaces_[kCmdSimEffort].set_value(tau);
  }

  return controller_interface::return_type::OK;
}

void PendulumPVTController::setpoint_callback(
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  // Last-writer-wins: a direct setpoint pre-empts any in-flight action goal.
  preempt_goal("preempted by direct ~/setpoint");
  Setpoint sp{params_.hold_position, 0.0, 0.0};
  if (!msg->positions.empty())     sp.position     = msg->positions[0];
  if (!msg->velocities.empty())    sp.velocity     = msg->velocities[0];
  if (!msg->accelerations.empty()) sp.acceleration = msg->accelerations[0];
  setpoint_buf_.writeFromNonRT(sp);
  // A streaming setpoint implies the caller wants tracking — switch to PVT
  // and clear any post-reset hold so the controller resumes following.
  mode_.store(Mode::PVT);
  waiting_for_setpoint_.store(false);
}

void PendulumPVTController::hold_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  preempt_goal("preempted by ~/hold");
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
  waiting_for_setpoint_.store(false);
  response->success = true;
  response->message = "Holding at q = " + std::to_string(sp.position);
}

void PendulumPVTController::free_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  preempt_goal("preempted by ~/free");
  mode_.store(Mode::FREE);
  response->success = true;
  response->message = "FREE mode — zero drive torque";
}

void PendulumPVTController::preempt_goal(const std::string & why)
{
  // Drop the trajectory first so the next RT tick stops sampling, then settle
  // the goal handle. The feedback timer will not finalise this goal because
  // we reset active_goal_ here (it short-circuits on null).
  traj_buf_.writeFromNonRT(nullptr);
  traj_done_.store(false);
  traj_completed_clean_.store(false);
  if (active_goal_) {
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::INVALID_GOAL;
    result->error_string = why;
    if (active_goal_->is_active()) {
      active_goal_->abort(result);
    }
    active_goal_.reset();
  }
}

rclcpp_action::GoalResponse PendulumPVTController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const FJT::Goal> goal)
{
  if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_node()->get_logger(),
                "FJT goal rejected: controller is not ACTIVE");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto & names = goal->trajectory.joint_names;
  if (names.size() != 1 || names[0] != params_.joint) {
    RCLCPP_WARN(get_node()->get_logger(),
                "FJT goal rejected: joint_names must be exactly [%s]",
                params_.joint.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto & points = goal->trajectory.points;
  if (points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "FJT goal rejected: empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Times must be strictly increasing and the final time must be positive
  // (otherwise the implicit start knot would create a zero-duration segment).
  double prev_t = -1.0;
  for (const auto & pt : points) {
    const double t = rclcpp::Duration(pt.time_from_start).seconds();
    if (t <= prev_t) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "FJT goal rejected: non-monotonic time_from_start");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (pt.positions.size() != 1) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "FJT goal rejected: each point must have one position");
      return rclcpp_action::GoalResponse::REJECT;
    }
    prev_t = t;
  }
  if (prev_t <= 0.0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "FJT goal rejected: total duration must be > 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PendulumPVTController::handle_cancel(
  const std::shared_ptr<GoalHandleFJT> /*goal_handle*/)
{
  // Finalisation runs in on_feedback_tick once the RT loop sees a null
  // trajectory (we will write null there). Accepting unconditionally is the
  // standard FJT contract.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PendulumPVTController::handle_accepted(
  const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  // Pre-empt any in-flight goal first so we never have two live goal handles.
  if (active_goal_) {
    preempt_goal("preempted by new goal");
  }

  // Read the joint position for the implicit start knot. Fall back to the
  // configured hold position if state is unavailable (boot grace etc.).
  double q_start = params_.hold_position;
  if (!state_interfaces_.empty()) {
    const auto pos_opt = state_interfaces_[0].get_optional();
    if (pos_opt) {
      q_start = pos_opt.value();
    }
  }

  auto traj = std::make_shared<Trajectory>();
  const auto & points = goal_handle->get_goal()->trajectory.points;
  const double first_t = rclcpp::Duration(points.front().time_from_start).seconds();

  // Prepend the implicit start knot so a single-point goal becomes a smooth
  // move from wherever the joint currently is. Skip it if the message itself
  // already begins at (or before) t=0.
  if (first_t > 0.0) {
    Knot k;
    k.t = 0.0;
    k.pos = q_start;
    k.vel = 0.0;
    k.acc = 0.0;
    k.has_acc = false;
    traj->knots.push_back(k);
  }
  for (const auto & pt : points) {
    Knot k;
    k.t = rclcpp::Duration(pt.time_from_start).seconds();
    k.pos = pt.positions[0];
    k.vel = pt.velocities.empty() ? 0.0 : pt.velocities[0];
    k.has_acc = !pt.accelerations.empty();
    k.acc = k.has_acc ? pt.accelerations[0] : 0.0;
    traj->knots.push_back(k);
  }
  traj->duration = traj->knots.back().t;

  // Reset governor state *before* publishing the trajectory so the RT loop
  // never sees a fresh trajectory with stale tv_/alpha_/dq_sign_.
  tv_ = 0.0;
  alpha_ = 1.0;
  dq_sign_ = (traj->knots.back().pos >= traj->knots.front().pos) ? 1.0 : -1.0;
  traj_done_.store(false);
  traj_completed_clean_.store(false);
  sampled_state_.writeFromNonRT({traj->knots.front().pos, 0.0, 0.0, q_start});

  traj_buf_.writeFromNonRT(std::shared_ptr<const Trajectory>(std::move(traj)));
  mode_.store(Mode::PVT);
  waiting_for_setpoint_.store(false);
  active_goal_ = goal_handle;
}

void PendulumPVTController::on_feedback_tick()
{
  if (!active_goal_) {
    return;
  }

  // Client-requested cancel takes priority over completion / pre-emption.
  if (active_goal_->is_canceling()) {
    const auto sampled = *sampled_state_.readFromRT();
    // Snapshot the last commanded position into setpoint_buf_ so the joint
    // holds where the trajectory was when cancellation arrived.
    Setpoint hold{sampled[0], 0.0, 0.0};
    setpoint_buf_.writeFromNonRT(hold);
    traj_buf_.writeFromNonRT(nullptr);
    traj_done_.store(false);
    traj_completed_clean_.store(false);
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::SUCCESSFUL;
    result->error_string = "canceled";
    active_goal_->canceled(result);
    active_goal_.reset();
    return;
  }

  const auto sampled = *sampled_state_.readFromRT();

  // Standard FJT feedback: desired / actual / error. We only track the single
  // joint, so the JointTolerance vectors in path_/goal_tolerance are not used.
  auto feedback = std::make_shared<FJT::Feedback>();
  feedback->joint_names = {params_.joint};
  feedback->desired.positions     = {sampled[0]};
  feedback->desired.velocities    = {sampled[1]};
  feedback->desired.accelerations = {sampled[2]};
  feedback->actual.positions      = {sampled[3]};
  feedback->error.positions       = {sampled[0] - sampled[3]};
  active_goal_->publish_feedback(feedback);

  if (traj_done_.load()) {
    // Hold the trajectory endpoint after the goal exits: writing the last
    // sampled position into setpoint_buf_ makes the legacy code path resume
    // and keep the joint at the goal.
    Setpoint hold{sampled[0], 0.0, 0.0};
    setpoint_buf_.writeFromNonRT(hold);
    traj_buf_.writeFromNonRT(nullptr);
    auto result = std::make_shared<FJT::Result>();
    if (traj_completed_clean_.load()) {
      result->error_code = FJT::Result::SUCCESSFUL;
      result->error_string = "trajectory completed";
      active_goal_->succeed(result);
    } else {
      result->error_code = FJT::Result::INVALID_GOAL;
      result->error_string = "preempted (e-stop or external)";
      active_goal_->abort(result);
    }
    traj_done_.store(false);
    traj_completed_clean_.store(false);
    active_goal_.reset();
  }
}

void PendulumPVTController::sample_trajectory(
  const Trajectory & traj, double t,
  double & p, double & v, double & a)
{
  // Clamp to the trajectory's defined range. Before the first knot is
  // structurally impossible (knots[0].t == 0 and tv_ starts at 0), but the
  // post-end case happens on the final tick when tv_ hits duration.
  if (traj.knots.size() == 1) {
    p = traj.knots[0].pos;
    v = traj.knots[0].vel;
    a = traj.knots[0].has_acc ? traj.knots[0].acc : 0.0;
    return;
  }
  if (t <= traj.knots.front().t) {
    p = traj.knots.front().pos;
    v = traj.knots.front().vel;
    a = traj.knots.front().has_acc ? traj.knots.front().acc : 0.0;
    return;
  }
  if (t >= traj.knots.back().t) {
    p = traj.knots.back().pos;
    v = traj.knots.back().vel;
    a = traj.knots.back().has_acc ? traj.knots.back().acc : 0.0;
    return;
  }
  // Linear scan — trajectories from FJT clients are short (typically <100
  // points); bisection would optimise away the wrong constant factor.
  for (std::size_t i = 1; i < traj.knots.size(); ++i) {
    if (t <= traj.knots[i].t) {
      sample_segment(traj.knots[i - 1], traj.knots[i], t, p, v, a);
      return;
    }
  }
  // Unreachable given the clamps above.
  p = traj.knots.back().pos;
  v = 0.0;
  a = 0.0;
}

void PendulumPVTController::sample_segment(
  const Knot & a, const Knot & b, double t,
  double & p, double & v, double & a_out)
{
  const double h = b.t - a.t;
  if (h <= 0.0) {
    p = b.pos;
    v = b.vel;
    a_out = b.has_acc ? b.acc : 0.0;
    return;
  }
  const double u = std::clamp((t - a.t) / h, 0.0, 1.0);

  if (a.has_acc && b.has_acc) {
    // Quintic Hermite — matches (pos, vel, acc) at both endpoints. Basis
    // h0..h5 satisfy h0(0)=1, h1'(0)=1, h2''(0)=1, h3(1)=1, h4'(1)=1,
    // h5''(1)=1 (everything else zero at both ends).
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
    const double h1 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5;
    const double h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
    const double h3 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5;
    const double h5 = 0.5 * u3 - u4 + 0.5 * u5;
    p = h0 * a.pos + h1 * (a.vel * h) + h2 * (a.acc * h * h)
      + h3 * b.pos + h4 * (b.vel * h) + h5 * (b.acc * h * h);

    const double h0p = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
    const double h1p = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4;
    const double h2p = u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4;
    const double h3p = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
    const double h4p = -12.0 * u2 + 28.0 * u3 - 15.0 * u4;
    const double h5p = 1.5 * u2 - 4.0 * u3 + 2.5 * u4;
    v = (h0p * a.pos + h3p * b.pos) / h
      + (h1p * a.vel + h4p * b.vel)
      + (h2p * a.acc + h5p * b.acc) * h;

    const double h0pp = -60.0 * u + 180.0 * u2 - 120.0 * u3;
    const double h1pp = -36.0 * u + 96.0 * u2 - 60.0 * u3;
    const double h2pp = 1.0 - 9.0 * u + 18.0 * u2 - 10.0 * u3;
    const double h3pp = 60.0 * u - 180.0 * u2 + 120.0 * u3;
    const double h4pp = -24.0 * u + 84.0 * u2 - 60.0 * u3;
    const double h5pp = 3.0 * u - 12.0 * u2 + 10.0 * u3;
    a_out = (h0pp * a.pos + h3pp * b.pos) / (h * h)
          + (h1pp * a.vel + h4pp * b.vel) / h
          + (h2pp * a.acc + h5pp * b.acc);
  } else {
    // Cubic Hermite — matches (pos, vel) at both endpoints. Velocity defaults
    // to 0 when the FJT message omits it (handle_accepted sets vel=0 then).
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;
    p = h00 * a.pos + h10 * (a.vel * h) + h01 * b.pos + h11 * (b.vel * h);

    const double h00p = 6.0 * u2 - 6.0 * u;
    const double h10p = 3.0 * u2 - 4.0 * u + 1.0;
    const double h01p = -6.0 * u2 + 6.0 * u;
    const double h11p = 3.0 * u2 - 2.0 * u;
    v = (h00p * a.pos + h01p * b.pos) / h + h10p * a.vel + h11p * b.vel;

    const double h00pp = 12.0 * u - 6.0;
    const double h10pp = 6.0 * u - 4.0;
    const double h01pp = -12.0 * u + 6.0;
    const double h11pp = 6.0 * u - 2.0;
    a_out = (h00pp * a.pos + h01pp * b.pos) / (h * h)
          + (h10pp * a.vel + h11pp * b.vel) / h;
  }
}

}  // namespace pendulum_pvt_control

PLUGINLIB_EXPORT_CLASS(
  pendulum_pvt_control::PendulumPVTController,
  controller_interface::ControllerInterface)
