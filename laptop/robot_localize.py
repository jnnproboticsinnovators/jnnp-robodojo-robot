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

Then it CHECKS the result (so a turned / shifted robot can never slip through unnoticed):
  * lets Nav2's position tracker (AMCL) settle on that spot without moving the robot
  * compares AMCL's position with the best fit; if they disagree it sends the fit again (3 tries)
  * prints  ALIGNED OK  or  NOT ALIGNED - DO NOT SEND GOALS  (and what to do)

Gazebo:  gzlocalize   (= python3 ~/robot_localize.py --sim --guess -2.05 -1.12)
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
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

PI = os.environ.get('PI', 'user@PI-IP')
SEARCH_RADIUS = 1.5      # m around the guess
YAW_STEP = 3             # deg (coarse search), then refined to 1 deg
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
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 lambda m: setattr(self, 'amcl', m), qos)
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')


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
    yy = (cys + pad)[:, None, None]
    xx = (cxs + pad)[None, :, None]
    for deg in (yaws if yaws is not None else range(0, 360, YAW_STEP)):
        t = math.radians(deg)
        dx = np.round((px * math.cos(t) - py * math.sin(t)) / res).astype(int)
        dy = np.round((px * math.sin(t) + py * math.cos(t)) / res).astype(int)
        keep = (np.abs(dx) < pad - rad - 2) & (np.abs(dy) < pad - rad - 2)
        # all candidate positions x all LiDAR points in one go (fast, no Python loop)
        acc = big[yy + dy[keep][None, None, :], xx + dx[keep][None, None, :]].sum(axis=2, dtype=np.int32)
        j = np.unravel_index(np.argmax(acc), acc.shape)
        score = acc[j] / max(1, len(dx))
        if score > best[0]:
            best = (score, ox + (cxs[j[1]] + 0.5) * res, oy + (cys[j[0]] + 0.5) * res, t)
    return best


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def send_pose(node, x, y, t):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'          # stamp 0 = "now" (works in real time AND Gazebo sim time)
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(t / 2), math.cos(t / 2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
    msg.pose.covariance[35] = math.radians(3) ** 2
    for _ in range(3):
        node.pub.publish(msg)
        time.sleep(0.3)


def settle(node, n=12):
    """Let AMCL compare LiDAR and map several times WITHOUT moving the robot."""
    if not node.nomotion.wait_for_service(timeout_sec=3.0):
        time.sleep(2.0)
        return
    for _ in range(n):
        node.nomotion.call_async(Empty.Request())
        time.sleep(0.35)


def send_and_check(node, sc, x, y, t, tries=3):
    """Send the fit, let AMCL settle, then check AMCL agrees (position 10 cm, heading 4 deg)."""
    for k in range(tries):
        node.amcl = None
        send_pose(node, x, y, t)
        settle(node)
        t_end = time.time() + 5
        while node.amcl is None and time.time() < t_end:
            settle(node, 2)
        a = node.amcl
        if a is None:
            print('  (no answer from AMCL yet - is Nav2 running?)')
            continue
        ax, ay = a.pose.pose.position.x, a.pose.pose.position.y
        at = yaw_of(a.pose.pose.orientation)
        dpos = math.hypot(ax - x, ay - y)
        dyaw = abs(math.degrees(math.atan2(math.sin(at - t), math.cos(at - t))))
        sd_xy = math.sqrt(max(a.pose.covariance[0], a.pose.covariance[7], 0.0))
        print(f'  check {k + 1}: AMCL at x={ax:.2f} y={ay:.2f} heading {math.degrees(at):.0f} deg '
              f'-> off by {dpos * 100:.0f} cm, {dyaw:.0f} deg; certainty +-{sd_xy * 100:.0f} cm')
        if sc >= 0.55 and dpos <= 0.10 and dyaw <= 4.0 and sd_xy <= 0.25:
            return True
    return False


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
    args = sys.argv[1:]
    sim = '--sim' in args
    args = [a for a in args if a != '--sim']
    global SEARCH_RADIUS
    if '--radius' in args:                  # search closer (used by robotcompete when lost)
        i = args.index('--radius')
        SEARCH_RADIUS = float(args[i + 1])
        del args[i:i + 2]
    guess_arg = None
    if '--guess' in args:
        i = args.index('--guess')
        guess_arg = (float(args[i + 1]), float(args[i + 2]))
        del args[i:i + 3]
    rclpy.init()
    node = Loc()
    if sim:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
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

    if guess_arg:
        guess = guess_arg
    elif args:
        reg = yaml.safe_load(open(os.path.expanduser(args[0])))
        s = reg.get('start') or reg.get('home')
        s = s[0] if isinstance(s, list) else s
        guess = (float(s['x']), float(s['y']))
    else:
        # 23 Nov: run in the middle of the field, searching around the START (0,0) found a wrong
        # spot that also fitted. Search around where navigation (AMCL) thinks the robot is now -
        # at the start that IS the start; later it is the (slightly drifted) current position.
        t_amcl = time.time() + 3
        while node.amcl is None and time.time() < t_amcl:
            time.sleep(0.1)
        if node.amcl is not None:
            a = node.amcl.pose.pose.position
            guess = (float(a.x), float(a.y))
            print('Searching around where navigation thinks the robot is now.')
        else:
            guess = (0.0, 0.0)
    node.occ, node.res, node.ox, node.oy = occ, res, ox, oy
    print(f'Searching within {SEARCH_RADIUS} m of x={guess[0]:.2f}, y={guess[1]:.2f}  (takes ~10-30 s)...')

    results = {}
    for mirror in (False, True):
        if mirror and results[False][0] >= 0.85:
            break                    # excellent fit already: no need to test the mirrored LiDAR
        print(f'  searching ({"mirrored LiDAR" if mirror else "LiDAR as now"}) ...', flush=True)
        px, py = scan_points(node.scan, mirror)
        coarse = match(occ, res, ox, oy, guess, px, py)
        d0 = math.degrees(coarse[3])
        fine = match(occ, res, ox, oy, coarse[1:3], px, py,             # refine: 1 deg, 3 cm
                     yaws=[d0 + k for k in range(-4, 5)], radius=0.12, step_m=res)
        results[mirror] = fine if fine[0] >= coarse[0] else coarse
        sc, x, y, t = results[mirror]
        print(f'  LiDAR {"MIRRORED" if mirror else "as now  "}: best fit {sc * 100:3.0f}% of points on walls'
              f'   at x={x:.2f} y={y:.2f} heading={math.degrees(t):.0f} deg')

    normal = results[False]
    mirrored = results.get(True, (0.0, 0.0, 0.0, 0.0))
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
        print(f'Robot found: x={x:.2f} m, y={y:.2f} m, heading {math.degrees(t):.0f} deg '
              f'({sc * 100:.0f}% of LiDAR points on the walls).')
        ok = send_and_check(node, sc, x, y, t)
        print('-------------------------------------------------')
        if ok:
            print('ALIGNED OK - Nav2 knows exactly where the robot is. You can send goals.')
        else:
            print('NOT ALIGNED - DO NOT SEND GOALS YET.')
            print('-> Put the robot exactly on its start mark, facing the same way as when the map was made,')
            print('   then run the localize word again (robotlocalize / gzlocalize).')
            print('-> Or in RViz: 2D Pose Estimate until the red LiDAR lines sit on the map walls.')
    print('=================================================', flush=True)
    # exit straight away: shutting ROS down while its listener thread still runs printed
    # "terminate called ... Aborted (core dumped)" - harmless, but it looked like a crash
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
