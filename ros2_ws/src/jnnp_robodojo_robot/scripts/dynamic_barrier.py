#!/usr/bin/env python3
"""Random time-based dynamic barrier (game field area 2).

Opens and closes the boom gate at random intervals by publishing the target
hinge angle on /barrier/cmd (bridged to Gazebo's JointPositionController).
"""
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

OPEN, CLOSED = 1.5708, 0.0


class DynamicBarrier(Node):
    def __init__(self):
        super().__init__('dynamic_barrier')
        self.declare_parameter('min_closed', 4.0)
        self.declare_parameter('max_closed', 10.0)
        self.declare_parameter('min_open', 4.0)
        self.declare_parameter('max_open', 12.0)
        self.pub = self.create_publisher(Float64, 'barrier/cmd', 10)
        self.state = CLOSED
        self.timer = None
        self.schedule_next()
        # keep re-sending the current target so Gazebo always has it
        self.create_timer(0.5, self.publish_state)

    def publish_state(self):
        self.pub.publish(Float64(data=self.state))

    def schedule_next(self):
        if self.state == CLOSED:
            lo, hi = self.get_parameter('min_closed').value, self.get_parameter('max_closed').value
        else:
            lo, hi = self.get_parameter('min_open').value, self.get_parameter('max_open').value
        wait = random.uniform(lo, hi)
        self.get_logger().info(f"Barrier {'CLOSED' if self.state == CLOSED else 'OPEN'} for {wait:.1f} s")
        if self.timer is not None:
            self.destroy_timer(self.timer)
        self.timer = self.create_timer(wait, self.toggle)

    def toggle(self):
        self.state = OPEN if self.state == CLOSED else CLOSED
        self.publish_state()
        self.schedule_next()


def main():
    rclpy.init()
    node = DynamicBarrier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
