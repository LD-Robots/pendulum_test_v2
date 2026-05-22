#!/usr/bin/env python3
"""Stream a closed-loop quintic point-to-point trajectory to the PVT controller.

  ros2 run pendulum_pvt_control pvt_goto.py <goal_rad> <duration_s> \
      [--rate HZ] [--lag-free RAD] [--lag-pause RAD] [--alpha-slew PER_S]

Point A is read from /joint_states (wherever the joint currently is); point B
is <goal_rad>. The quintic profile has zero velocity and acceleration at both
endpoints, so the position / velocity / acceleration in every
JointTrajectoryPoint are mutually consistent — exactly what the controller's
Kp, Kd and inertia-feedforward terms expect.

Unlike a pure open-loop stream, the trajectory clock is governed by the
measured position: /joint_states is monitored for the whole run and a
time-scale factor `alpha` modulates how fast the clock advances. If the joint
is physically held back, the reference is only ever allowed to lead the actual
position by a bounded amount — the clock smoothly slows to a stop. When the
joint is released the clock ramps back up and the path continues from where it
left off, with no position jump and no velocity spike. An undisturbed move is
unaffected: `alpha` stays 1 and the stream matches the plain quintic.

When the stream ends the controller keeps the last setpoint in its buffer and
holds it — there is no need to keep publishing.
"""

import argparse
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT = 'pendulum_joint'
SETPOINT_TOPIC = '/pendulum_pvt_controller/setpoint'

# Watchdog — abort the move if /joint_states goes silent this long. The topic
# runs far faster than this, so it tolerates a transient hiccup but still
# catches a real dropout (continuing open-loop blind is the bug being fixed).
JS_TIMEOUT = 0.5  # s


def make_quintic(q0, dq, duration):
    """Quintic point-to-point profile as closures over a virtual time `tv`.

    Returns (pos, vel, accel); each evaluates the profile at an arbitrary `tv`
    in seconds, with `tv` clamped to [0, duration] so sampling past the end
    yields the goal at rest. The s / sd / sdd math is identical to the streamed
    quintic in pvt_sim_gui.py: s(0)=0, s(1)=1, with s' = s'' = 0 at both ends.
    """
    def _tau(tv):
        return min(max(tv, 0.0), duration) / duration

    def pos(tv):
        tau = _tau(tv)
        s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        return q0 + dq * s

    def vel(tv):
        tau = _tau(tv)
        sd = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
        return dq * sd

    def accel(tv):
        tau = _tau(tv)
        sdd = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (duration * duration)
        return dq * sdd

    return pos, vel, accel


def parse_args(argv):
    p = argparse.ArgumentParser(
        prog='pvt_goto.py',
        description='Stream a closed-loop quintic point-to-point trajectory '
                    'to the PVT controller.')
    p.add_argument('goal', type=float, help='goal angle (rad)')
    p.add_argument('duration', type=float, help='nominal move duration (s)')
    p.add_argument('--rate', type=float, default=200.0,
                   help='setpoint stream rate (Hz, default 200)')
    # Governor tuning — `lead` is how far the reference is ahead of the joint,
    # in the travel direction. Below lag-free the move runs full speed; at
    # lag-pause it is fully stopped; in between the clock is scaled smoothly.
    p.add_argument('--lag-free', type=float, default=0.04,
                   help='lead (rad) below which the move runs at full speed '
                        '(default 0.04)')
    p.add_argument('--lag-pause', type=float, default=0.14,
                   help='lead (rad) at which the move is fully paused — also '
                        'bounds the on-release catch-up (default 0.14)')
    p.add_argument('--alpha-slew', type=float, default=2.0,
                   help='max rate of change of the time-scale factor (1/s); '
                        'lower = gentler resume (default 2.0)')
    return p.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])
    if args.duration <= 0.0 or args.rate <= 0.0:
        print('duration and rate must be > 0')
        return 1
    if not (args.lag_pause > args.lag_free > 0.0):
        print('require lag_pause > lag_free > 0')
        return 1
    if args.alpha_slew <= 0.0:
        print('alpha_slew must be > 0')
        return 1

    rclpy.init()
    node = rclpy.create_node('pvt_goto')
    pub = node.create_publisher(JointTrajectoryPoint, SETPOINT_TOPIC, 10)

    # /joint_states is monitored for the whole run — the governor reads the
    # latest measured position every tick, and `stamp` feeds the watchdog.
    state = {'q': None, 'stamp': None}

    def on_joint_state(msg):
        if JOINT in msg.name:
            state['q'] = msg.position[msg.name.index(JOINT)]
            state['stamp'] = time.monotonic()

    node.create_subscription(JointState, '/joint_states', on_joint_state, 10)

    # Point A — wait for the first /joint_states to learn where the joint is.
    node.get_logger().info('waiting for /joint_states ...')
    while state['q'] is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    if state['q'] is None:
        node.get_logger().error('no /joint_states received — is the controller up?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    q0 = state['q']
    dq = args.goal - q0
    node.get_logger().info(
        f'q0={q0:.4f} rad -> goal={args.goal:.4f} rad over {args.duration:.2f}s '
        f'@ {args.rate:.0f}Hz')

    if abs(dq) < 1e-9:
        # Already at the goal — publish one point to engage PVT hold and exit.
        pt = JointTrajectoryPoint()
        pt.positions, pt.velocities, pt.accelerations = [q0], [0.0], [0.0]
        pub.publish(pt)
        node.get_logger().info('already at goal — nothing to do')
        node.destroy_node()
        rclpy.shutdown()
        return 0

    pos_fn, vel_fn, accel_fn = make_quintic(q0, dq, args.duration)
    dq_sign = 1.0 if dq > 0.0 else -1.0
    duration = args.duration
    dt_nominal = 1.0 / args.rate

    tv = 0.0       # virtual trajectory time, 0 .. duration
    alpha = 1.0    # time-scale factor in [0, 1], slew-limited
    now0 = time.monotonic()
    t_prev = now0
    next_tick = now0

    rc = 0
    stopped_early = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)  # drain /joint_states
            now = time.monotonic()
            dt_real = now - t_prev                  # measured loop period
            t_prev = now

            if state['stamp'] is None or (now - state['stamp']) > JS_TIMEOUT:
                node.get_logger().error('lost /joint_states — aborting move')
                rc, stopped_early = 1, True
                break

            # Error-gated time-scaling: how far the reference leads the joint
            # in the travel direction sets how fast the clock may advance.
            lead = (pos_fn(tv) - state['q']) * dq_sign
            alpha_target = (args.lag_pause - lead) / (args.lag_pause - args.lag_free)
            alpha_target = min(1.0, max(0.0, alpha_target))

            # Slew-limit alpha so the time-scale — and hence the streamed
            # velocity — stays continuous through a pause/resume.
            max_step = args.alpha_slew * dt_real
            alpha += min(max_step, max(-max_step, alpha_target - alpha))
            alpha = min(1.0, max(0.0, alpha))
            if alpha < 0.05:
                node.get_logger().info('joint restrained — trajectory paused',
                                       throttle_duration_sec=2.0)

            tv = min(tv + alpha * dt_real, duration)

            # Publish a mutually-consistent point. vel/accel carry the chain
            # rule for the scaled clock: d(pos)/dt_wall = vel(tv)*alpha.
            pt = JointTrajectoryPoint()
            pt.positions     = [pos_fn(tv)]
            pt.velocities    = [vel_fn(tv) * alpha]
            pt.accelerations = [accel_fn(tv) * alpha * alpha]
            pub.publish(pt)

            if tv >= duration:
                break

            # Sleep on an absolute schedule so the loop period does not drift.
            next_tick += dt_nominal
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0.0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()        # fell behind — resync
    except KeyboardInterrupt:
        stopped_early = True
        node.get_logger().info('interrupted')

    if stopped_early:
        # Freeze at the last commanded position, at rest.
        pt = JointTrajectoryPoint()
        pt.positions, pt.velocities, pt.accelerations = [pos_fn(tv)], [0.0], [0.0]
        pub.publish(pt)
        node.get_logger().info('holding the last commanded point')
    else:
        # Final settle point — the exact goal, at rest.
        pt = JointTrajectoryPoint()
        pt.positions, pt.velocities, pt.accelerations = [args.goal], [0.0], [0.0]
        pub.publish(pt)
        node.get_logger().info('done — controller holds the final point')

    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
