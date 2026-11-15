#!/usr/bin/env python3
"""Lightweight robot for RViz-only driving (no Gazebo).

Listens to  /cmd_vel   (geometry_msgs/Twist)   - drive commands from teleop
            /dump_cmd  (std_msgs/Float64)      - back cabin angle in rad (0 = down)
Publishes   TF odom -> base_footprint, /odom, /joint_states (4 wheels + back cabin)

The robot moves on a flat plane (no walls/physics) so you can watch it drive and dump in RViz.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

WHEELS = {  # joint name: side (+1 left, -1 right)
    'wheel_1_joint': -1, 'wheel_2_joint': -1, 'wheel_3_joint': +1, 'wheel_4_joint': +1}
CABIN = 'back_cabin_joint'


def step_toward(current, target, max_step):
    if abs(target - current) <= max_step:
        return target
    return current + math.copysign(max_step, target - current)


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_separation', 0.371)
        self.declare_parameter('cabin_speed', 0.6)       # rad/s the back cabin moves
        self.declare_parameter('cabin_max', 0.785)       # rad (45 deg, URDF limit)
        self.declare_parameter('cmd_timeout', 0.5)       # s - stop if teleop goes quiet
        self.r = self.get_parameter('wheel_radius').value
        self.sep = self.get_parameter('wheel_separation').value
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.last_cmd = self.get_clock().now()
        self.wheel_pos = {j: 0.0 for j in WHEELS}
        self.cabin = 0.0
        self.cabin_target = 0.0
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Float64, 'dump_cmd', self.on_dump, 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.dt = 1.0 / 50.0
        self.create_timer(self.dt, self.update)
        self.get_logger().info('Fake robot ready: drive with teleop, press d in teleop to dump')

    def on_cmd(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z
        self.last_cmd = self.get_clock().now()

    def on_dump(self, msg):
        self.cabin_target = max(0.0, min(float(msg.data), self.get_parameter('cabin_max').value))

    def update(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds * 1e-9 > self.get_parameter('cmd_timeout').value:
            self.v = self.w = 0.0
        dt = self.dt
        # unicycle integration
        self.th += self.w * dt
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        # wheel spin: left/right wheel speeds of a skid-steer base
        for j, side in WHEELS.items():
            wheel_v = self.v - side * self.w * self.sep / 2.0
            self.wheel_pos[j] += wheel_v / self.r * dt
        self.cabin = step_toward(self.cabin, self.cabin_target,
                                 self.get_parameter('cabin_speed').value * dt)
        stamp = now.to_msg()
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        self.odom_pub.publish(o)

        js = JointState()
        js.header.stamp = stamp
        js.name = list(WHEELS) + [CABIN]
        js.position = [self.wheel_pos[j] for j in WHEELS] + [self.cabin]
        self.js_pub.publish(js)


def main():
    rclpy.init()
    node = FakeRobot()
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
