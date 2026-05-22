#!/usr/bin/env python3
"""Publish random targets in [2.0, 4.28] rad on /pendulum/target.

Default cadence: one random target every 1-2.5 seconds. Mirrors the training
event resample_target_interval so the policy sees the same kind of step
challenges in deploy as during training (pvt_v1 trained with 3-6 s resample —
pass --interval-min 3.0 --interval-max 6.0 to match it exactly).

Usage:
    ros2 run pendulum_pvt_policy random_target_publisher.py
    ros2 run pendulum_pvt_policy random_target_publisher.py --min 2.0 --max 4.28 \\
        --interval-min 3.0 --interval-max 6.0

Topic:
    /pendulum/target  (std_msgs/Float64)
"""

import argparse
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class RandomTargetPublisher(Node):
    def __init__(self, low: float, high: float,
                 interval_min: float, interval_max: float,
                 topic: str = "/pendulum/target") -> None:
        super().__init__("random_target_publisher")
        self.low = low
        self.high = high
        self.interval_min = interval_min
        self.interval_max = interval_max
        self.pub = self.create_publisher(Float64, topic, 10)
        self._schedule_next()

    def _schedule_next(self) -> None:
        delay = random.uniform(self.interval_min, self.interval_max)
        self.timer = self.create_timer(delay, self._on_tick)

    def _on_tick(self) -> None:
        # one-shot timer pattern — destroy and re-create with a fresh delay
        self.timer.cancel()
        self.destroy_timer(self.timer)

        target = random.uniform(self.low, self.high)
        msg = Float64()
        msg.data = target
        self.pub.publish(msg)
        self.get_logger().info(f"new target: {target:.3f} rad")

        self._schedule_next()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Random target publisher for /pendulum/target.")
    parser.add_argument("--min", type=float, default=2.0,
                        help="Lower bound of target range (rad).")
    parser.add_argument("--max", type=float, default=4.28,
                        help="Upper bound of target range (rad).")
    parser.add_argument("--interval-min", type=float, default=1.0,
                        help="Minimum seconds between new targets.")
    parser.add_argument("--interval-max", type=float, default=2.5,
                        help="Maximum seconds between new targets.")
    parser.add_argument("--topic", type=str, default="/pendulum/target",
                        help="Topic to publish on.")
    args = parser.parse_args()

    rclpy.init()
    node = RandomTargetPublisher(
        args.min, args.max, args.interval_min, args.interval_max, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
