#!/usr/bin/env python3
"""
JNNP RoboDojo - measure the real robot with its own LiDAR (no tape measure needed).

Run on the LAPTOP (robotengine running on the Pi, robotdrive open in another window):
    robotros
    python3 ~/robot_calibrate.py

1. Put the robot facing a WALL / field border, 1 - 2 m away, nothing moving in between.
2. Press Enter here (START).
3. In the teleop, hold UP and drive STRAIGHT towards the wall about 0.5 - 1 m, then SPACE.
   (Stop at least 25 cm before the wall.)
4. Press Enter here again (END).

It prints:
  * how far the robot REALLY moved (from the LiDAR: how much closer the wall got)
  * how far the WHEELS think it moved (/odom)
  * the corrected TICKS_PER_METER
  * where the LiDAR thinks "front" is (should be 0 deg)
"""
import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

TICKS_PER_METER_NOW = 1400.0   # the value currently in robodojo_ros.py


class Cal(Node):
    def __init__(self):
        super().__init__('robot_calibrate')
        self.scan = None
        self.odom = None
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', m), 10)


def snapshot(node):
    s, o = node.scan, node.odom
    if s is None or o is None:
        return None
    return list(s.ranges), s.angle_min, s.angle_increment, o.pose.pose.position.x, o.pose.pose.position.y


def smooth(r, i, n=3):
    vals = [r[(i + k) % len(r)] for k in range(-n, n + 1)]
    vals = [v for v in vals if math.isfinite(v)]
    return sorted(vals)[len(vals) // 2] if len(vals) >= n else float('nan')


def main():
    rclpy.init()
    node = Cal()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print('\nWaiting for /scan and /odom ...')
    while rclpy.ok() and (node.scan is None or node.odom is None):
        threading.Event().wait(0.2)
    print('OK - both received.\n')
    print('Robot facing a wall 1 - 2 m away?  Press Enter = START')
    input()
    a = snapshot(node)
    print('Now hold UP in the teleop, drive straight 0.5 - 1 m towards the wall, SPACE.')
    print('Then press Enter = END')
    input()
    threading.Event().wait(0.5)          # let one more scan arrive
    b = snapshot(node)

    r0, amin, inc, x0, y0 = a
    r1, _, _, x1, y1 = b
    odom_d = math.hypot(x1 - x0, y1 - y0)

    # the direction where the wall came closest = the robot's real "front" in the LiDAR
    best_i, best_drop = None, 0.0
    for i in range(len(r0)):
        d0, d1 = smooth(r0, i), smooth(r1, i)
        if math.isfinite(d0) and math.isfinite(d1) and d0 < 4.0:
            drop = d0 - d1
            if drop > best_drop:
                best_i, best_drop = i, drop

    print('\n================ RESULT ================')
    print(f'Wheels (/odom) say the robot moved : {odom_d:.3f} m')
    if best_i is None or best_drop < 0.1:
        print('LiDAR could not see a wall getting closer. Face a flat wall 1 - 2 m away and try again.')
    else:
        front_deg = math.degrees(amin + best_i * inc)
        front_deg = (front_deg + 180) % 360 - 180
        print(f'LiDAR says the robot really moved  : {best_drop:.3f} m')
        if odom_d > 0.05:
            new_tpm = TICKS_PER_METER_NOW * odom_d / best_drop
            print(f'TICKS_PER_METER: now {TICKS_PER_METER_NOW:.0f}  ->  set it to {new_tpm:.0f}')
        else:
            print('The wheels reported almost no movement - is robotengine running?')
        print(f'LiDAR "front" is at {front_deg:+.0f} deg   (0 = correct; within +/-10 is fine)')
    print('========================================')
    print('Send this result to Claude.\n')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
