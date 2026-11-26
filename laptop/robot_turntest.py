#!/usr/bin/env python3
"""
JNNP RoboDojo - TURN TEST: does the robot know how much it turned?

    robotturntest        (robotrun NAME must be READY / ALIGNED OK; robot in an open spot,
                          e.g. on its start marks; nobody touching it)

Why: in every mission run the robot gets LOST on the first leg, right after it turns
around at the screen. This test measures, with numbers instead of guesses:
  1. GYRO DRIFT : the robot stands still 10 s - does its heading creep?
  2. TURN       : it turns about 180 deg on the spot by itself.
                  WHEELS/GYRO say it turned X deg, the LiDAR (matched to the map) says Y deg.
If X and Y differ by more than a few degrees, the robot's own turn measurement is wrong -
that makes it lose its place on every turn, and THAT is what has to be fixed.
Writes the result to ~/turntest_log.txt as well.
"""
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

LOG = os.path.expanduser('~/turntest_log.txt')
TURN_SPEED = 0.5        # rad/s
TURN_GOAL = math.radians(180)


def out(t):
    print(t, flush=True)
    with open(LOG, 'a') as f:
        f.write(time.strftime('%H:%M:%S ') + t + '\n')


class T(Node):
    def __init__(self):
        super().__init__('robodojo_turntest')
        self.yaw = None
        self.unwrapped = 0.0
        self.prev = None
        self.create_subscription(Odometry, 'odom', self.cb, 20)
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)

    def cb(self, m):
        q = m.pose.pose.orientation
        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.prev is not None:
            d = (y - self.prev + math.pi) % (2 * math.pi) - math.pi
            self.unwrapped += d
        self.prev = y
        self.yaw = y


def lidar_heading():
    """Run the LiDAR-vs-map localization; return (heading_deg, fit_percent) or None."""
    p = subprocess.run(['python3', os.path.expanduser('~/robot_localize.py'), '--radius', '0.6'],
                       stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=90)
    m = re.search(r'Robot found: x=(-?[\d.]+) m, y=(-?[\d.]+) m, heading (-?[\d.]+) deg \((\d+)%', p.stdout)
    if not m or 'ALIGNED OK' not in p.stdout:
        return None
    return float(m.group(3)), int(m.group(4)), float(m.group(1)), float(m.group(2))


def main():
    rclpy.init()
    n = T()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    out('=== TURN TEST ===')
    t = time.time()
    while n.yaw is None and time.time() - t < 10:
        time.sleep(0.1)
    if n.yaw is None:
        out('No /odom from the robot - is robotrun running?'); os._exit(1)

    # 1. gyro drift
    out('1/3 Standing still 10 s (do not touch the robot) ...')
    a = n.unwrapped
    time.sleep(10)
    drift = math.degrees(n.unwrapped - a) * 6          # deg per minute
    out(f'    heading crept {drift:+.1f} deg per minute while standing still '
        + ('(OK)' if abs(drift) < 3 else '(TOO MUCH - gyro zero is off)'))

    # 2. LiDAR heading before
    out('2/3 LiDAR heading before the turn ...')
    h0 = lidar_heading()
    if h0 is None:
        out('    could not find the robot with the LiDAR - put it on its start marks and run robotrun again')
        os._exit(1)
    out(f'    LiDAR: heading {h0[0]:.0f} deg ({h0[1]}% fit) at x {h0[2]:.2f} y {h0[3]:.2f}')

    # 3. turn ~180 deg left on the spot
    out('3/3 Turning about 180 deg on the spot by itself ...')
    a = n.unwrapped
    tw = Twist(); tw.angular.z = TURN_SPEED
    t = time.time()
    while abs(n.unwrapped - a) < TURN_GOAL and time.time() - t < 20:
        n.cmd.publish(tw)
        time.sleep(0.05)
    for _ in range(10):
        n.cmd.publish(Twist()); time.sleep(0.05)
    time.sleep(2.0)
    odom_turn = math.degrees(n.unwrapped - a)
    took = time.time() - t
    h1 = lidar_heading()
    if h1 is None:
        out('    could not find the robot with the LiDAR after the turn')
        os._exit(1)
    lidar_turn = (h1[0] - h0[0] + 180) % 360 - 180
    if lidar_turn < -90 and odom_turn > 90:
        lidar_turn += 360
    moved = math.hypot(h1[2] - h0[2], h1[3] - h0[3])
    out(f'    took {took:.1f} s')
    out(f'    WHEELS/GYRO say it turned : {odom_turn:6.1f} deg')
    out(f'    LiDAR (truth) says        : {lidar_turn:6.1f} deg   ({h1[1]}% fit)')
    out(f'    centre moved {moved:.2f} m during the turn on the spot')
    err = odom_turn - lidar_turn
    out(f'RESULT: turn error {err:+.1f} deg '
        + ('-> OK, the robot measures its turns well.' if abs(err) <= 6 else
           '-> TOO BIG: the robot measures its own turns wrongly - this makes it get lost.'))
    if lidar_turn != 0:
        out(f'        (scale LiDAR/gyro = {lidar_turn / odom_turn:.3f})')
    out('Send all of this to Claude.')
    os._exit(0)


if __name__ == '__main__':
    main()
