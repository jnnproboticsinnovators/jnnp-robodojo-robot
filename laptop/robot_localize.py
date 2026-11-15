#!/usr/bin/env python3
"""
JNNP RoboDojo - FIND THE ROBOT ON THE MAP automatically (instead of clicking 2D Pose Estimate).

Run on the LAPTOP while robotengine and robotnav <map> are running:
    robotros
    python3 ~/robot_localize.py                      # searches around the map's start point
    python3 ~/robot_localize.py ~/trial_6a_regions.yaml   # search around 'start' in this file

It compares what the LiDAR sees now with the map at thousands of positions/directions
(also with the LiDAR angles mirrored), picks the best fit and tells Nav2 (/initialpose).
If the MIRRORED version fits much better, it offers to fix LIDAR_CLOCKWISE on the robot.
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

PI = os.environ.get('PI', 'user@PI-IP')
SEARCH_RADIUS = 1.5      # m around the guess
YAW_STEP = 3             # deg
N_POINTS = 150           # LiDAR points used


class Loc(Node):
    def __init__(self):
        super().__init__('robot_localize')
        self.map = None
        self.scan = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map', m), qos)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)


def scan_points(scan, mirror):
    a = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    r = np.asarray(scan.ranges, dtype=float)
    ok = np.isfinite(r) & (r > 0.2) & (r < 8.0)
    a, r = a[ok], r[ok]
    if mirror:
        a = -a
    if len(r) > N_POINTS:
        pick = np.linspace(0, len(r) - 1, N_POINTS).astype(int)
        a, r = a[pick], r[pick]
    return r * np.cos(a), r * np.sin(a)


def match(occ, res, ox, oy, guess, px, py, yaws=None, radius=None, step_m=0.06):
    """Best (score, x, y, yaw) of the points (px, py) on the dilated wall grid occ."""
    h, w = occ.shape
    step = max(1, int(round(step_m / res)))
    rad = int((radius or SEARCH_RADIUS) / res)
    gcx, gcy = int((guess[0] - ox) / res), int((guess[1] - oy) / res)
    cxs = np.arange(gcx - rad, gcx + rad + 1, step)
    cys = np.arange(gcy - rad, gcy + rad + 1, step)
    best = (-1, 0, 0, 0)
    pad = 400
    big = np.zeros((h + 2 * pad, w + 2 * pad), np.int16)
    big[pad:pad + h, pad:pad + w] = occ
    for deg in (yaws if yaws is not None else range(0, 360, YAW_STEP)):
        t = math.radians(deg)
        dx = np.round((px * math.cos(t) - py * math.sin(t)) / res).astype(int)
        dy = np.round((px * math.sin(t) + py * math.cos(t)) / res).astype(int)
        keep = (np.abs(dx) < pad - rad - 2) & (np.abs(dy) < pad - rad - 2)
        acc = np.zeros((len(cys), len(cxs)), np.int32)
        for ddx, ddy in zip(dx[keep], dy[keep]):
            acc += big[np.ix_(cys + ddy + pad, cxs + ddx + pad)]
        j = np.unravel_index(np.argmax(acc), acc.shape)
        score = acc[j] / max(1, len(dx))
        if score > best[0]:
            best = (score, ox + (cxs[j[1]] + 0.5) * res, oy + (cys[j[0]] + 0.5) * res, t)
    return best


def save_clockwise(flip):
    r = subprocess.run(['ssh', PI, 'cat ~/robodojo_calib.json 2>/dev/null || echo {}'],
                       capture_output=True, text=True)
    try:
        cal = json.loads(r.stdout.strip() or '{}')
    except ValueError:
        cal = {}
    cal['LIDAR_CLOCKWISE'] = not bool(cal.get('LIDAR_CLOCKWISE', True)) if flip else cal.get('LIDAR_CLOCKWISE', True)
    r = subprocess.run(['ssh', PI, 'cat > ~/robodojo_calib.json'], input=json.dumps(cal, indent=1), text=True)
    return r.returncode == 0, cal['LIDAR_CLOCKWISE']


def main():
    rclpy.init()
    node = Loc()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('Waiting for the map (robotnav running?) and /scan ...')
    t_end = time.time() + 30
    while (node.map is None or node.scan is None) and time.time() < t_end:
        time.sleep(0.2)
    if node.map is None or node.scan is None:
        sys.exit('No map or no LiDAR scan received - is robotengine + robotnav running?')

    m = node.map
    res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
    grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    walls = (grid >= 65).astype(np.int16)
    occ = walls.copy()                       # thicken walls by 1 cell (small errors still count)
    for sy, sx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        occ |= np.roll(np.roll(walls, sy, 0), sx, 1)

    if len(sys.argv) > 1:
        reg = yaml.safe_load(open(os.path.expanduser(sys.argv[1])))
        s = reg.get('start') or reg.get('home')
        s = s[0] if isinstance(s, list) else s
        guess = (float(s['x']), float(s['y']))
    else:
        guess = (0.0, 0.0)
    print(f'Searching within {SEARCH_RADIUS} m of x={guess[0]:.2f}, y={guess[1]:.2f}  (takes ~10-30 s)...')

    results = {}
    for mirror in (False, True):
        px, py = scan_points(node.scan, mirror)
        coarse = match(occ, res, ox, oy, guess, px, py)
        d0 = math.degrees(coarse[3])
        fine = match(occ, res, ox, oy, coarse[1:3], px, py,             # refine: 1 deg, 3 cm
                     yaws=[d0 + k for k in range(-4, 5)], radius=0.12, step_m=res)
        results[mirror] = fine if fine[0] >= coarse[0] else coarse
        sc, x, y, t = results[mirror]
        print(f'  LiDAR {"MIRRORED" if mirror else "as now  "}: best fit {sc * 100:3.0f}% of points on walls'
              f'   at x={x:.2f} y={y:.2f} heading={math.degrees(t):.0f} deg')

    normal, mirrored = results[False], results[True]
    print('\n================ LOCALIZE RESULT ================')
    if max(normal[0], mirrored[0]) < 0.35:
        print('The LiDAR view does not match this map anywhere near the start.')
        print('-> Is this the same field, set up the same way as when the map was made?')
        print('-> Or move the robot to the start spot and run this again.')
    elif mirrored[0] > normal[0] + 0.08:
        print('The MIRRORED LiDAR fits the map much better: the LiDAR angles are reversed.')
        if input('Fix it on the robot now (LIDAR_CLOCKWISE)? [y/n] ').strip().lower() == 'y':
            ok, val = save_clockwise(True)
            print(f'Saved LIDAR_CLOCKWISE = {val}.' if ok else 'Could not save.')
            print('Restart robotengine (Ctrl+C, robotengine), then run this tool again.')
    else:
        sc, x, y, t = normal
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(t / 2), math.cos(t / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
        msg.pose.covariance[35] = math.radians(5) ** 2
        for _ in range(3):
            node.pub.publish(msg)
            time.sleep(0.3)
        print(f'Robot found: x={x:.2f} m, y={y:.2f} m, heading {math.degrees(t):.0f} deg '
              f'({sc * 100:.0f}% of LiDAR points on the walls).')
        print('Sent to Nav2. In RViz the red dots should now sit on the black walls.')
    print('=================================================')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
