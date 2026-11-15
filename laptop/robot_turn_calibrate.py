#!/usr/bin/env python3
"""
JNNP RoboDojo - TURN calibration for the real robot (the robot measures itself).

Run on the LAPTOP while robotengine is running (close robotslam / robotnav first):
    robotros
    python3 ~/robot_turn_calibrate.py

What happens:
  1. It reads the robot's current values from the Pi (Pi password).
  2. The robot TURNS LEFT on the spot by itself, about 90 degrees, and stops.
  3. It compares:  how much the WHEELS say it turned  <->  how much the LiDAR saw the room turn.
  4. It shows the corrected TRACK_WIDTH (and whether the LiDAR angles are mirrored)
     and, if you say yes, saves them on the Pi in ~/robodojo_calib.json.
  5. Restart robotengine to use them.

Put the robot in an open spot (40 cm free all around) where the LiDAR sees walls / objects
on several sides - NOT in an empty hall. Keep people still while it measures.
"""
import json
import math
import os
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PI = os.environ.get('PI', 'user@PI-IP')
TURN_SPEED = 0.6            # rad/s
TARGET_DEG = 90.0
DEFAULTS = {'TICKS_PER_METER': 1400.0, 'TRACK_WIDTH': 0.371,
            'LIDAR_CLOCKWISE': True, 'LIDAR_FRONT_ANGLE': 0.0}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_calib():
    r = subprocess.run(['ssh', PI, 'cat ~/robodojo_calib.json 2>/dev/null || echo {}'],
                       capture_output=True, text=True)
    cal = dict(DEFAULTS)
    try:
        cal.update(json.loads(r.stdout.strip() or '{}'))
    except ValueError:
        pass
    return cal


def save_calib(cal):
    r = subprocess.run(['ssh', PI, 'cat > ~/robodojo_calib.json'], input=json.dumps(cal, indent=1),
                       text=True)
    return r.returncode == 0


class Node2(Node):
    def __init__(self):
        super().__init__('robot_turn_calibrate')
        self.scan = None
        self.yaw = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Odometry, 'odom',
                                 lambda m: setattr(self, 'yaw', yaw_of(m.pose.pose.orientation)), 10)

    def drive(self, w):
        t = Twist()
        t.angular.z = w
        self.pub.publish(t)


def ranges_per_degree(scan):
    """360 values, one per degree (index 0 = angle -180 deg)."""
    out = [float('nan')] * 360
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and 0.15 < r < 6.0:
            deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            k = int(round(deg + 180)) % 360
            if not math.isfinite(out[k]) or r < out[k]:
                out[k] = r
    return out


def best_shift(r0, r1):
    """Shift s (deg) so that r1[i + s] matches r0[i]."""
    best = None
    for s in range(-180, 180):
        diffs = [min(abs(r0[i] - r1[(i + s) % 360]), 0.5)
                 for i in range(360) if math.isfinite(r0[i]) and math.isfinite(r1[(i + s) % 360])]
        if len(diffs) < 60:
            continue
        score = sum(diffs) / len(diffs)
        if best is None or score < best[1]:
            best = (s, score, len(diffs))
    return best


def wait_new_scan(node, after, timeout=3.0):
    t_end = time.time() + timeout
    while time.time() < t_end:
        s = node.scan
        if s is not None and (s.header.stamp.sec + s.header.stamp.nanosec * 1e-9) > after:
            return s
        time.sleep(0.05)
    return node.scan


def main():
    print('\nReading the robot values from the Pi (enter the Pi password if asked)...')
    cal = read_calib()
    print(f"  now: TRACK_WIDTH = {cal['TRACK_WIDTH']:.3f} m,  LIDAR_CLOCKWISE = {cal['LIDAR_CLOCKWISE']}")

    rclpy.init()
    node = Node2()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('Waiting for /scan and /odom ...')
    while node.scan is None or node.yaw is None:
        time.sleep(0.2)
    print('OK.\n')
    input('Robot in an open spot, 40 cm free all around?  Press Enter and it will TURN LEFT by itself...')

    time.sleep(1.0)
    now = node.get_clock().now().nanoseconds * 1e-9
    s0 = wait_new_scan(node, now)
    r0 = ranges_per_degree(s0)
    yaw0 = node.yaw

    print('Turning...')
    turned = 0.0
    t_end = time.time() + 10.0
    while time.time() < t_end:
        turned = math.degrees((node.yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi)
        if abs(turned) >= TARGET_DEG:
            break
        node.drive(TURN_SPEED)
        time.sleep(0.05)
    for _ in range(10):
        node.drive(0.0)
        time.sleep(0.05)
    print('Stopped. Measuring...')
    time.sleep(1.5)
    odom_deg = math.degrees((node.yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    now = node.get_clock().now().nanoseconds * 1e-9
    r1 = ranges_per_degree(wait_new_scan(node, now))

    best = best_shift(r0, r1)
    print('\n================ TURN RESULT ================')
    print(f'Wheels (/odom) say the robot turned : {odom_deg:+.0f} deg')
    if best is None:
        print('The LiDAR did not see enough around it. Move the robot near walls / objects and try again.')
    else:
        real_deg = -best[0]
        print(f'LiDAR says the robot really turned  : {real_deg:+.0f} deg   (match quality {best[1]:.3f} m)')
        if abs(real_deg) < 15 or abs(odom_deg) < 15:
            print('Hardly any real turn - wheels slipping or robot stuck? Check and try again.')
        else:
            new = dict(cal)
            if (real_deg > 0) != (odom_deg > 0):
                new['LIDAR_CLOCKWISE'] = not cal['LIDAR_CLOCKWISE']
                print('!! The LiDAR angles are MIRRORED -> LIDAR_CLOCKWISE will be switched to',
                      new['LIDAR_CLOCKWISE'])
            new['TRACK_WIDTH'] = round(cal['TRACK_WIDTH'] * abs(odom_deg) / abs(real_deg), 3)
            print(f"TRACK_WIDTH: {cal['TRACK_WIDTH']:.3f} m  ->  {new['TRACK_WIDTH']:.3f} m")
            if not 0.25 <= new['TRACK_WIDTH'] <= 1.5 or best[1] > 0.25:
                print('This result looks unusual - run the test again before saving.')
            print('=============================================')
            if input('Save these values on the robot? [y/n] ').strip().lower() == 'y':
                if save_calib(new):
                    print('Saved in ~/robodojo_calib.json on the Pi.')
                    print('Now restart the engine: Ctrl+C in the robotengine window, then type robotengine.')
                else:
                    print('Could not save - check the connection to the Pi.')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
