#!/usr/bin/env python3
"""Stream a smooth quintic point-to-point trajectory to the PVT controller.

  ros2 run pendulum_pvt_control pvt_goto.py <goal_rad> <duration_s> [rate_hz]

Point A is read once from /joint_states (wherever the joint currently is);
point B is <goal_rad>. The quintic profile has zero velocity and acceleration
at both endpoints, so the position / velocity / acceleration in every
JointTrajectoryPoint are mutually consistent — exactly what the controller's
Kp, Kd and inertia-feedforward terms expect.

When the stream ends the controller keeps the last setpoint (B, zero velocity)
in its buffer and holds it — there is no need to keep publishing.
"""

import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT = 'pendulum_joint'
SETPOINT_TOPIC = '/pendulum_pvt_controller/setpoint'


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        print(__doc__)
        return 1
    goal = float(args[0])
    duration = float(args[1])
    rate = float(args[2]) if len(args) > 2 else 200.0
    if duration <= 0.0 or rate <= 0.0:
        print('duration and rate must be > 0')
        return 1

    rclpy.init()
    node = rclpy.create_node('pvt_goto')
    pub = node.create_publisher(JointTrajectoryPoint, SETPOINT_TOPIC, 10)

    # Point A — read the current joint angle once from /joint_states.
    q0 = None

    def on_joint_state(msg):
        nonlocal q0
        if JOINT in msg.name:
            q0 = msg.position[msg.name.index(JOINT)]

    sub = node.create_subscription(JointState, '/joint_states', on_joint_state, 10)
    node.get_logger().info('waiting for /joint_states ...')
    while q0 is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_subscription(sub)
    if q0 is None:
        node.get_logger().error('no /joint_states received — is the controller up?')
        rclpy.shutdown()
        return 1

    dq = goal - q0
    node.get_logger().info(
        f'q0={q0:.4f} rad -> goal={goal:.4f} rad over {duration:.2f}s @ {rate:.0f}Hz')

    dt = 1.0 / rate
    steps = max(1, int(duration * rate))
    for i in range(steps + 1):
        t = min(i * dt, duration)
        tau = t / duration
        # Quintic s(tau): s(0)=0, s(1)=1, with s' = s'' = 0 at both endpoints.
        s   = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        sd  = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
        sdd = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (duration * duration)
        pt = JointTrajectoryPoint()
        pt.positions     = [q0 + dq * s]
        pt.velocities    = [dq * sd]
        pt.accelerations = [dq * sdd]
        pub.publish(pt)
        time.sleep(dt)

    node.get_logger().info('done — controller holds the final point')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
