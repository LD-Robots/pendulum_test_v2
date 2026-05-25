#ifndef PENDULUM_PVT_CONTROL__PENDULUM_PVT_CONTROLLER_HPP_
#define PENDULUM_PVT_CONTROL__PENDULUM_PVT_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "pendulum_safety/clamp.hpp"
#include "pendulum_safety/estop_subscriber.hpp"
#include "pendulum_safety/param_loader.hpp"
#include "pendulum_safety/rate_limiter.hpp"
#include "pendulum_safety/safety_limits.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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

  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;

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

  // A single waypoint in an action-driven trajectory. `has_acc` selects the
  // segment interpolant in sample_segment(): both endpoints with `has_acc=true`
  // → quintic-Hermite (matches pos+vel+acc at both ends); otherwise → cubic-
  // Hermite (matches pos+vel; vel defaults to 0 when the message omits it).
  struct Knot
  {
    double t{0.0};        // seconds from trajectory start (knots[0].t == 0)
    double pos{0.0};
    double vel{0.0};
    double acc{0.0};
    bool   has_acc{false};
  };

  struct Trajectory
  {
    std::vector<Knot> knots;   // monotonically increasing t, size >= 2
    double duration{0.0};      // == knots.back().t
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
    // true  -> real hardware: stream pos/vel/effort/kp/kd, drive runs the PD law.
    // false -> Gazebo sim: claim only effort, run the PD law in software.
    bool drive_side_pd{true};
    // Lag-governor (port of pvt_goto.py's --lag-free / --lag-pause / --alpha-slew).
    // Active only when an action-driven trajectory is being sampled.
    double lag_free{0.04};      // rad: below this lead the move runs full speed
    double lag_pause{0.14};     // rad: at this lead the move is fully paused
    double alpha_slew{2.0};     // 1/s: max rate of change of the time-scale
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

  // Action server callbacks — all run on the executor thread (action callbacks
  // are serialised by rclcpp_action, so active_goal_ needs no lock).
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FJT::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
  // 10 Hz timer body: publishes FJT feedback from the RT-published sampled state
  // and finalises the goal on completion / cancel / pre-emption.
  void on_feedback_tick();
  // Write nullptr to traj_buf_ and abort active_goal_ if any. Called from
  // setpoint_callback / hold_service / free_service / on_deactivate / e-stop
  // falling edge. RT loop falls through to the legacy setpoint path next tick.
  void preempt_goal(const std::string & why);
  // Publish a one-shot ~/setpoint with (position, 0, 0). Called from
  // on_feedback_tick on goal settlement; gives external observers a static
  // view of the resting target.
  void publish_settled_setpoint(double position);

  // Pure helpers — RT-safe. cubic-Hermite when either knot lacks acc; quintic-
  // Hermite when both endpoints carry acc.
  static void sample_segment(
    const Knot & a, const Knot & b, double t,
    double & p, double & v, double & a_out);
  static void sample_trajectory(
    const Trajectory & traj, double t,
    double & p, double & v, double & a);

  Params params_;
  // Cached from params_ in on_configure — command_interface_configuration() runs
  // before activation and must agree with the index order used in update().
  bool drive_side_pd_{true};

  // Centralised safety: limits loaded once in on_configure; the supervisor's
  // e-stop / Kp-derate signal read every update(); the rate limiter bounds the
  // commanded position slew + acceleration.
  pendulum_safety::SafetyLimits limits_;
  pendulum_safety::EstopSubscriber safety_;
  pendulum_safety::RateLimiter rate_limiter_;
  double estop_hold_pos_{0.0};   // joint position snapshotted when e-stop fires
  bool estop_was_active_{false};
  // After a reset (e-stop falling edge) we hold here until a fresh ~/setpoint
  // arrives — prevents the joint from lunging to a stale buffered point.
  double resume_hold_pos_{0.0};
  std::atomic<bool> waiting_for_setpoint_{false};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr setpoint_sub_;
  // Same topic the controller subscribes to. The action server publishes the
  // final endpoint here when a goal settles (success / cancel / abort), so
  // external observers see a one-shot "resting target" on ~/setpoint while
  // the realtime interpolator output stays on ~/active_setpoint.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr setpoint_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr free_srv_;

  realtime_tools::RealtimeBuffer<Setpoint> setpoint_buf_;
  std::atomic<Mode> mode_{Mode::FREE};

  // Action interface. Written by executor-thread callbacks, read by the RT
  // loop via traj_buf_ (single-writer / single-reader RealtimeBuffer).
  rclcpp_action::Server<FJT>::SharedPtr action_server_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<const Trajectory>> traj_buf_;
  // Latched by the RT loop when tv_ reaches duration, or by preempt_goal /
  // e-stop falling edge. The feedback timer turns it into a goal result.
  std::atomic<bool> traj_done_{false};
  // True when the RT loop latched traj_done_ from a clean completion (else the
  // termination came from pre-emption / e-stop and the goal is aborted).
  std::atomic<bool> traj_completed_clean_{false};

  // Governor state — touched only by the RT loop; reset by handle_accepted
  // *before* the trajectory shared_ptr is published, so the RT loop reads
  // consistent state on the first tick of a new goal.
  double tv_{0.0};
  double alpha_{1.0};
  double dq_sign_{1.0};

  // RT-published snapshot for the 10 Hz feedback timer: {ref_pos, ref_vel,
  // ref_acc, measured_q}. Written every tick while a trajectory is active.
  realtime_tools::RealtimeBuffer<std::array<double, 4>> sampled_state_;

  // Touched only on the executor thread (action callbacks + feedback timer
  // are all on the controller_manager executor, serialised).
  std::shared_ptr<GoalHandleFJT> active_goal_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // Diagnostic mirror of the resolved (ref_pos, ref_vel, ref_acc) — published
  // at 200 Hz on ~/active_setpoint regardless of the source (trajectory /
  // ~/setpoint / e-stop hold / post-reset hold). Lets PlotJuggler overlay
  // "what the controller is tracking" alongside /joint_states the same way it
  // does for pvt_goto.py's ~/setpoint stream.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
    active_setpoint_pub_;
  rclcpp::TimerBase::SharedPtr active_setpoint_timer_;
  void on_active_setpoint_tick();
};

}  // namespace pendulum_pvt_control

#endif  // PENDULUM_PVT_CONTROL__PENDULUM_PVT_CONTROLLER_HPP_
