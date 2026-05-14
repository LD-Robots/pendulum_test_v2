#!/usr/bin/env python3
"""PD + feedforward setpoint node — variant 3.

Reads /joint_states, computes tau = Kp*(q_d - q) + Kd*(qd_d - qd) + FF,
publishes to /pendulum_effort_controller/commands. Setpoint via ~/setpoint
topic; ~/hold and ~/free services for the mode machine. Loops at 200 Hz —
not realtime-safe; use variant 1 if you need 1 kHz determinism.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


FREE, TUNE, PD = 0, 1, 2


class PDSetpointNode(Node):
    def __init__(self):
        super().__init__('pd_setpoint_node')

        self.declare_parameters(namespace='', parameters=[
            ('joint',          'pendulum_joint'),
            ('command_topic',  '/pendulum_effort_controller/commands'),
            ('Kp',             22.95),
            ('Kd',             2.14),
            ('tau_limit',      20.0),
            ('mgl',            4.10),
            ('J',              0.102),
            ('Fv',             0.05),
            ('comp_sign',      1.0),
            ('ff_gravity',     True),
            ('ff_inertia',     True),
            ('ff_viscous',     False),
            ('hold_position',  0.0),
            ('rate_hz',        200.0),
        ])

        self._lock = threading.Lock()
        self._mode = FREE
        self._sp = JointTrajectoryPoint()
        self._sp.positions = [self.get_parameter('hold_position').value]
        self._sp.velocities = [0.0]
        self._sp.accelerations = [0.0]
        self._q = 0.0
        self._qd = 0.0
        self._have_state = False

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(
            JointTrajectoryPoint, '~/setpoint', self._setpoint_cb, 10)

        self.create_service(Trigger, '~/hold', self._hold_srv)
        self.create_service(Trigger, '~/free', self._free_srv)

        cmd_topic = self.get_parameter('command_topic').value
        self._cmd_pub = self.create_publisher(Float64MultiArray, cmd_topic, 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"pd_setpoint_node up — joint={self.get_parameter('joint').value} "
            f"cmd_topic={cmd_topic} rate={rate:.0f} Hz")

    def _joint_state_cb(self, msg: JointState):
        joint = self.get_parameter('joint').value
        try:
            idx = msg.name.index(joint)
        except ValueError:
            return
        with self._lock:
            self._q = msg.position[idx] if idx < len(msg.position) else 0.0
            self._qd = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
            self._have_state = True

    def _setpoint_cb(self, msg: JointTrajectoryPoint):
        with self._lock:
            self._sp = msg
            self._mode = PD

    def _hold_srv(self, req, resp):
        with self._lock:
            self._sp.positions = [self._q]
            self._sp.velocities = [0.0]
            self._sp.accelerations = [0.0]
            self._mode = PD
            q_hold = self._q
        resp.success = True
        resp.message = f'Holding at q={q_hold:.4f}'
        return resp

    def _free_srv(self, req, resp):
        with self._lock:
            self._mode = FREE
        resp.success = True
        resp.message = 'FREE mode — zero effort'
        return resp

    def _tick(self):
        with self._lock:
            if not self._have_state:
                return
            mode = self._mode
            q, qd = self._q, self._qd
            sp_pos = self._sp.positions[0] if self._sp.positions else 0.0
            sp_vel = self._sp.velocities[0] if self._sp.velocities else 0.0
            sp_acc = self._sp.accelerations[0] if self._sp.accelerations else 0.0

        Kp = float(self.get_parameter('Kp').value)
        Kd = float(self.get_parameter('Kd').value)
        mgl = float(self.get_parameter('mgl').value)
        J = float(self.get_parameter('J').value)
        Fv = float(self.get_parameter('Fv').value)
        tau_limit = float(self.get_parameter('tau_limit').value)
        comp_sign = float(self.get_parameter('comp_sign').value)
        ff_g = bool(self.get_parameter('ff_gravity').value)
        ff_J = bool(self.get_parameter('ff_inertia').value)
        ff_v = bool(self.get_parameter('ff_viscous').value)

        if mode == FREE:
            tau = 0.0
        else:
            tau_g = mgl * math.sin(q) if ff_g else 0.0
            tau_Jt = J * sp_acc if ff_J else 0.0
            tau_v = Fv * qd if ff_v else 0.0
            tau_pd = Kp * (sp_pos - q) + Kd * (sp_vel - qd) if mode == PD else 0.0
            tau = comp_sign * (tau_g + tau_Jt + tau_v) + tau_pd
            if tau_limit > 0:
                tau = max(-tau_limit, min(tau_limit, tau))

        msg = Float64MultiArray()
        msg.data = [tau]
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PDSetpointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
