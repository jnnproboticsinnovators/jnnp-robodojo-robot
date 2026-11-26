#!/usr/bin/env python3
"""
JNNP RoboDojo - THE WHOLE COMPETITION RUN WITH ONE COMMAND.

    robotcompete NAME            (real robot: after  robotrun NAME  says ALIGNED OK + READY;
                                  it switches the camera + AI on and off by itself)
    gzcompete [plastic|metal|organic]   (Gazebo practice: after  gzrun fast  says READY)

Needs  ~/maps/NAME_points.yaml  made once with  robotpoints NAME  (Gazebo: gzpoints).

The run (nobody touches the robot after GO):
  1. drive to the SCREEN, look at it, read the waste type with the AI (majority vote)
  2. drive to the matching LOADING station, wait while it is loaded
  3. drive the marked route (grass, corridors, sand/ramps) to the GATE
  4. GATE open (LiDAR sees through it) -> gated path (10 pts); closed -> wait, then the detour (5 pts)
     (gate_check skipped with K when marking = no gate: always route + detour)
  5. drive to the matching OFFLOADING station = the FINISH: tip the bin up and down, stop there
Obstacles on the way (e.g. the obstacle zone) are seen by the LiDAR and avoided by Nav2 itself.
Every step is printed with its time, and saved in ~/compete_log.txt.
Ctrl+C = cancel and stop the robot at once.

Settings you may change: the CAPITAL values below, or on the command line:
    --load-wait 5     seconds at the loading station
    --gate-wait 5     seconds to wait for a closed gate before taking the detour
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
import urllib.request

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

LOAD_WAIT = 5.0        # s at the loading station (being loaded)
GATE_WAIT = 5.0        # s to wait for a closed gate, then take the detour
GATE_CLEAR = 0.8       # m - gate counts as OPEN when the LiDAR sees this far straight ahead
GATE_CONE = math.radians(10)   # +- this much around straight ahead
AI_LOOK = 4.0          # s of AI pictures for the vote at the screen
AI_MAX = 15.0          # s max at the screen before choosing the best guess
AI_MIN_CONF = 0.40
DUMP_UP = math.radians(40)
DUMP_TIME = 3.0        # s bin up
GOAL_TIMEOUT = 60.0    # s per try of a stage (one automatic retry) before going on
LOST_LIMIT = 0.90      # m  (24 Nov turn test: 0.5-0.7 happens during a NORMAL turn - not lost) - navigation's own uncertainty above this = robot is LOST: stop + re-find
MAX_REFINDS = 2        # re-finds per stage
AMCL = {'cert': None}  # latest certainty (m) from /amcl_pose
SIM = False
SHORT = False   # --short: screen, AI, loading, offloading + dump only (no route) - safe backup
LAST = {'code': 0}
# Nav2 codes meaning 'the robot is OFF the map / INSIDE a wall' in navigation's view = it is lost
LOST_CODES = {203, 205, 303, 305}
TYPES = ('plastic', 'organic', 'metal')
LOG = os.path.expanduser('~/compete_log.txt')
T0 = None


def say(text):
    t = '' if T0 is None else f'[{time.monotonic() - T0:6.1f} s] '
    line = f'{t}{text}'
    print(line, flush=True)
    try:
        with open(LOG, 'a') as f:
            f.write(time.strftime('%H:%M:%S ') + line + '\n')
    except OSError:
        pass


def log_only(text):
    try:
        with open(LOG, 'a') as f:
            t = '' if T0 is None else f'[{time.monotonic() - T0:6.1f} s] '
            f.write(time.strftime('%H:%M:%S ') + t + text + '\n')
    except OSError:
        pass


def to_pose(nav, p):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = nav.get_clock().now().to_msg()
    ps.pose.position.x, ps.pose.position.y = float(p['x']), float(p['y'])
    ps.pose.orientation.z = math.sin(float(p['yaw']) / 2)
    ps.pose.orientation.w = math.cos(float(p['yaw']) / 2)
    return ps


ODOM = {}
RAMP_RUNUP = 0.7        # m: start the straight drive this far BEFORE the ramp top
RAMP_SPEED = 0.15       # m/s straight over the ramp


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def odom_cb(m):
    q = m.pose.pose.orientation
    ODOM.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def ramp_drive(nav, cmd, guard_pub, top, bottom):
    """24 Nov finals: near the ramp the LiDAR sees the SLOPE as a wall, so Nav2 turned away and the
    wall guard refused to drive on. Here: Nav2 brings the robot to a run-up spot in front of the
    ramp, then the robot drives STRAIGHT over it (gyro keeps the heading), wall guard paused,
    then it finds itself on the map again and Nav2 takes over."""
    dx, dy = bottom['x'] - top['x'], bottom['y'] - top['y']
    n = math.hypot(dx, dy) or 1.0
    ux, uy = dx / n, dy / n
    head = math.atan2(uy, ux)
    run = {'x': top['x'] - RAMP_RUNUP * ux, 'y': top['y'] - RAMP_RUNUP * uy, 'yaw': head}
    length = RAMP_RUNUP + n + 0.10
    drive(nav, 'RAMP run-up spot', [run])

    def spin(t):
        end = time.monotonic() + t
        while time.monotonic() < end:
            rclpy.spin_once(nav, timeout_sec=0.02)

    def stop():
        for _ in range(10):
            cmd.publish(Twist())
            time.sleep(0.03)

    for _ in range(5):
        guard_pub.publish(Bool(data=False))
        spin(0.05)
    spin(0.5)
    if 'yaw' not in ODOM or 'yaw' not in AMCL:
        say('   RAMP: no odometry/position - skipping the straight drive')
    else:
        # 1. face straight up the ramp
        err = wrap(head - AMCL['yaw'])
        y0 = ODOM['yaw']
        t = time.monotonic()
        while abs(wrap(ODOM['yaw'] - y0) - err) > math.radians(4) and time.monotonic() - t < 10:
            tw = Twist()
            tw.angular.z = 0.5 if wrap(err - wrap(ODOM['yaw'] - y0)) > 0 else -0.5
            cmd.publish(tw)
            spin(0.05)
        stop()
        spin(0.3)
        # 2. straight over the ramp, heading held by the gyro
        x0, yy0, ref = ODOM['x'], ODOM['y'], ODOM['yaw']
        say(f'   RAMP: driving straight over it ({length:.1f} m)')
        t = time.monotonic()
        while math.hypot(ODOM['x'] - x0, ODOM['y'] - yy0) < length and time.monotonic() - t < 25:
            tw = Twist()
            tw.linear.x = RAMP_SPEED
            tw.angular.z = max(-0.3, min(0.3, 1.5 * wrap(ref - ODOM['yaw'])))
            cmd.publish(tw)
            spin(0.05)
        stop()
        say(f'   RAMP: done ({time.monotonic() - t:.0f} s)')
    for _ in range(5):
        guard_pub.publish(Bool(data=True))
        spin(0.05)
    try:
        nav.clearAllCostmaps()
    except Exception:
        pass
    refind(radius=1.0)          # the LiDAR was tilted on the ramp: find the robot again


def named_points(P, lst):
    """[(name, pose)] of a route list, named route_grass / route_ramp_top ... where known."""
    raw = P.get('_raw') or {}
    out = []
    for i, v in enumerate(P.get(lst) or []):
        if not isinstance(v, dict):
            continue
        nm = next((k for k, r in raw.items() if isinstance(r, dict) and r.get('x') == v.get('x')
                   and r.get('y') == v.get('y')), f'{lst} point {i + 1}')
        out.append((nm, v))
    return out


def drive(nav, label, poses):
    """Drive there; if Nav2 gives up, clear its obstacle memory and try ONE more time by itself
    (a person standing in the way, a wrongly remembered obstacle ...). True = arrived."""
    if not poses:
        return True
    t = time.monotonic()
    LAST['code'] = 0
    if drive_once(nav, label, poses):
        return True
    if LAST['code'] == -1:            # long way round: go on to the next point, no retry
        return False
    if LAST['code'] in LOST_CODES:
        # 24 Nov: code 203 = navigation thought the robot was OUTSIDE the map -> it was lost.
        say(f'   {label}: navigation lost the robot (code {LAST["code"]}) - finding it again')
        refind()
    if time.monotonic() - t < 3.0:
        # gave up at once: usually Nav2 is restarting itself (23 Nov log: "planner_server IS DOWN
        # ... Managed nodes have been reset", back ~8 s later). Wait for it instead of failing on.
        say(f'   {label}: Nav2 not ready - waiting for it (up to 20 s)')
        w = wait_nav2(nav, 20.0)
        say(f'   Nav2 ready again after {w:.0f} s' if nav2_active(nav)
            else '   Nav2 still not ready - trying anyway')
    say(f'   {label}: retrying once (obstacle memory cleared)')
    try:
        nav.clearAllCostmaps()
    except Exception:
        pass
    time.sleep(0.5)
    return drive_once(nav, label + ' (retry)', poses)


def refind(wait=True, radius=0.8):
    """Find the robot on the map again from the LiDAR (robot_localize.py, ~10-30 s)."""
    cmd = ['python3', os.path.expanduser('~/robot_localize.py'), '--radius', str(radius)] + (['--sim'] if SIM else [])
    p = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, env=dict(os.environ, PI=os.environ.get('PI', '')))
    if not wait:
        return p
    return refind_result(p)


def refind_result(p, timeout=60):
    try:
        out, _ = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.kill()
        say('   re-find: took too long')
        return False
    ok = 'ALIGNED OK' in out
    line = next((l for l in out.splitlines() if l.startswith('Robot found')), 'robot not found')
    say(f'   re-find: {"ALIGNED OK" if ok else "NOT aligned"} - {line}')
    return ok


def drive_once(nav, label, poses):
    say(f'-> {label}')
    if len(poses) == 1:
        accepted = nav.goToPose(to_pose(nav, poses[0]))
    else:
        accepted = nav.goThroughPoses([to_pose(nav, p) for p in poses])
    if accepted is False:
        say(f'   {label}: Nav2 REFUSED the goal (navigation not active? is robotrun READY?)')
        return False
    t = time.monotonic()
    refinds, lost_since, t_log = 0, None, 0.0
    while not nav.isTaskComplete():
        if time.monotonic() - t_log > 1.0 and 'x' in AMCL:
            t_log = time.monotonic()
            log_only(f'      pos x {AMCL["x"]:6.2f} y {AMCL["y"]:6.2f} '
                     f'heading {math.degrees(AMCL["yaw"]):5.0f}  unsure +-{AMCL["cert"] or 0:.2f} m')
        fb = nav.getFeedback()
        if fb is not None and hasattr(fb, 'distance_remaining'):
            print(f'     {fb.distance_remaining:5.2f} m to go   ', end='\r', flush=True)
            # 24 Nov finals: blocked in the obstacle zone, Nav2 chose the LONG way round (robot came
            # back). If the path is much longer than the direct distance, skip this point instead.
            if label.startswith('ROUTE') and len(poses) == 1 and 'x' in AMCL and time.monotonic() - t > 3.0:
                direct = math.hypot(poses[0]['x'] - AMCL['x'], poses[0]['y'] - AMCL['y'])
                if fb.distance_remaining > max(2.0, 2.5 * direct):
                    nav.cancelTask()
                    say(f'   {label}: Nav2 wants the long way round ({fb.distance_remaining:.1f} m for '
                        f'{direct:.1f} m) - skipping this point')
                    LAST['code'] = -1
                    return False
        # LOST? (23 Nov: robot lost its place on the route, drove into walls) -> stop, re-find, go on
        c = AMCL['cert']
        if c is not None and c > LOST_LIMIT:
            lost_since = lost_since or time.monotonic()
        else:
            lost_since = None
        if lost_since and time.monotonic() - lost_since > 5.0 and refinds < MAX_REFINDS:
            refinds += 1
            nav.cancelTask()
            say(f'   {label}: robot LOST (uncertain by {c:.2f} m) - stopping to find itself again')
            # 2nd time: search wider (24 Nov: lost by ~1.8 m on the grass, 0.8 m search found nothing)
            refind(radius=0.8 if refinds == 1 else 2.0)
            AMCL['cert'] = None
            lost_since = None
            if len(poses) == 1:
                nav.goToPose(to_pose(nav, poses[0]))
            else:
                nav.goThroughPoses([to_pose(nav, p) for p in poses])
            continue
        if time.monotonic() - t > GOAL_TIMEOUT:
            nav.cancelTask()
            say(f'   {label}: took longer than {GOAL_TIMEOUT:.0f} s - going on')
            return False
        time.sleep(0.1)
    ok = nav.getResult() == TaskResult.SUCCEEDED
    say(f'   {label}: {"REACHED" if ok else "NOT reached (going on)"}  ({time.monotonic() - t:.0f} s)'
        + ('' if ok else '  WHY: ' + nav_reason(nav)))
    return ok


def nav2_active(nav):
    """True when Nav2's navigate_to_pose action server answers (what the goals really use).
    (23 Nov: the lifecycle-service check said 'NOT active' although every goal then worked.)"""
    try:
        return nav.nav_to_pose_client.wait_for_server(timeout_sec=2.0)
    except Exception:
        return False


def wait_nav2(nav, timeout):
    t = time.monotonic()
    while time.monotonic() - t < timeout:
        if nav2_active(nav):
            time.sleep(1.0)           # let the costmaps fill once
            break
        time.sleep(0.5)
    return time.monotonic() - t


def nav_reason(nav):
    """Nav2's own reason for giving up (so we fix the real cause, not guess)."""
    try:
        r = nav.result_future.result().result
        code = int(getattr(r, 'error_code', 0))
        LAST['code'] = code
        msg = str(getattr(r, 'error_msg', '') or '').strip()
        return f'code {code} {msg}'.strip() if (code or msg) else 'no reason given (see robotnavlog)'
    except Exception:
        return 'no reason given (see robotnavlog)'


def check_marks_on_map(nav, P, timeout=5.0):
    """Warn about marks that sit ON or right next to a wall of the map (24 Nov: a route mark was
    saved on the top wall -> Nav2 could never plan to it)."""
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    got = {}
    q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)
    sub = nav.create_subscription(OccupancyGrid, 'map', lambda m: got.setdefault('m', m), q)
    t = time.monotonic()
    while 'm' not in got and time.monotonic() - t < timeout:
        rclpy.spin_once(nav, timeout_sec=0.2)
    nav.destroy_subscription(sub)
    if 'm' not in got:
        print('(map not received - mark check skipped)')
        return
    m = got['m']
    res, ox, oy, W, H = (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                         m.info.width, m.info.height)
    data = m.data
    raw = P.get('_raw') or {}
    if raw:      # named marks (route_grass, route_ramp_top ...) - the names robotremark uses
        marks = [(k, v) for k, v in raw.items() if isinstance(v, dict) and 'x' in v]
    else:
        marks = [(k, v) for k, v in P.items() if isinstance(v, dict) and 'x' in v]
        for lst in ('route_to_gate', 'detour'):
            marks += [(f'{lst}[{i + 1}]', v) for i, v in enumerate(P.get(lst) or []) if isinstance(v, dict)]
    bad = 0
    for k, v in marks:
        c, r = int((v['x'] - ox) / res), int((v['y'] - oy) / res)
        best = None
        n = int(0.4 / res)
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and data[rr * W + cc] >= 65:
                    d = math.hypot(dr, dc) * res
                    best = d if best is None or d < best else best
        if best is not None and best < 0.15:
            bad += 1
            print(f'   !! {k:18s} x {v["x"]:6.2f} y {v["y"]:6.2f}  is '
                  + ('ON A WALL' if best < 0.06 else f'only {best:.2f} m from a wall')
                  + f'  -> robotremark {P.get("map", "NAME")} {k.split("[")[0]}')
    print('Marks on the map: all clear of walls.' if not bad else
          f'{bad} mark(s) on/too close to a wall - the robot cannot reach those exactly.')


def where_am_i(nav, timeout=3.0):
    """Robot position on the map from AMCL (None if navigation does not know it)."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    got = {}
    q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)
    sub = nav.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                  lambda m: got.setdefault('p', m), q)
    t = time.monotonic()
    while 'p' not in got and time.monotonic() - t < timeout:
        rclpy.spin_once(nav, timeout_sec=0.2)
    nav.destroy_subscription(sub)
    if 'p' not in got:
        return None
    p = got['p'].pose
    return p.pose.position.x, p.pose.position.y, math.sqrt(max(p.covariance[0], 0) + max(p.covariance[7], 0))


def ai_items(pi_host):
    try:
        with urllib.request.urlopen(f'http://{pi_host}:8000/detections.json', timeout=2) as r:
            d = json.loads(r.read())
        if time.time() - float(d.get('time', 0)) > 3.0:      # old result: not live
            return []
        out = []
        for i in d.get('items', []):
            n = str(i.get('name', '')).lower()
            t = next((t for t in TYPES if t in n), None)      # 'Plastic', 'plastic_bottle' -> plastic
            if t:
                out.append((t, float(i.get('conf', 0))))
        return out
    except Exception:
        return None


def read_waste(pi_host):
    """Majority vote over the AI results while the robot looks at the screen."""
    votes, score, t = {k: 0 for k in TYPES}, {k: 0.0 for k in TYPES}, time.monotonic()
    while time.monotonic() - t < AI_MAX:
        items = ai_items(pi_host)
        if items is None:
            say('   AI not answering (camera off?)')
        for name, conf in items or []:
            if name in votes and conf >= AI_MIN_CONF:
                votes[name] += 1
                score[name] += conf
        best = max(TYPES, key=lambda k: (votes[k], score[k]))
        if time.monotonic() - t >= AI_LOOK and votes[best] >= 3:
            say(f'   AI: {best.upper()}  (votes {votes})')
            return best
        time.sleep(0.25)
    best = max(TYPES, key=lambda k: (votes[k], score[k]))
    if votes[best] == 0:
        say('   AI saw nothing sure - guessing PLASTIC')
        return 'plastic'
    say(f'   AI best guess: {best.upper()}  (votes {votes})')
    return best


class Gate:
    def __init__(self, nav):
        self.scan = None
        nav.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m),
                                qos_profile_sensor_data)

    def clear_ahead(self):
        s = self.scan
        if s is None:
            return None
        best = []
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= GATE_CONE and math.isfinite(r) and r > s.range_min:
                best.append(r)
        return min(best) if best else s.range_max


def main():
    global T0, LOAD_WAIT, GATE_WAIT
    args = sys.argv[1:]

    def opt(flag, default):
        if flag in args:
            i = args.index(flag)
            v = args[i + 1]
            del args[i:i + 2]
            return v
        return default
    sim = '--sim' in args
    global SHORT
    SHORT = '--short' in sys.argv
    args = [a for a in args if a not in ('--sim', '--short')]
    pi_host = opt('--pi', '')
    LOAD_WAIT = float(opt('--load-wait', LOAD_WAIT))
    GATE_WAIT = float(opt('--gate-wait', GATE_WAIT))
    waste_forced = opt('--waste', None)
    name = args[0] if args else 'robodojo_field'
    pfile = os.path.expanduser(f'~/maps/{name}_points.yaml')
    if not os.path.isfile(pfile):
        sys.exit(f'No places marked for map "{name}" ({pfile}).\n'
                 f'Mark them once:  {"gzpoints" if sim else "robotpoints " + name}')
    P = yaml.safe_load(open(pfile))
    need = ['screen'] + [f'{a}_{t}' for a in ('load', 'offload') for t in TYPES]
    missing = [k for k in need if k not in P]
    # The gate is OPTIONAL: if gate_check was skipped (K) the run never uses the gate and always
    # takes the marked route + detour straight to the offloading station (team's choice, 23 Nov).
    use_gate = 'gate_check' in P and 'after_gate' in P
    need += [k for k in ('gate_check', 'after_gate') if k in P]
    if not use_gate and not P.get('detour') and not P.get('route_to_gate'):
        print('NOTE: no gate, no detour and no route marked - the robot will drive straight from the '
              'loading station to the offloading station.')
    if missing:
        sys.exit(f'{pfile} is missing: {", ".join(missing)} - run the points word again.')

    rclpy.init()
    nav = BasicNavigator()
    if sim:
        nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    dump = nav.create_publisher(Float64, 'dump_cmd', 10)
    cmd = nav.create_publisher(Twist, 'cmd_vel', 10)
    guard_pub = nav.create_publisher(Bool, 'guard_enable', 10)
    nav.create_subscription(Odometry, 'odom', odom_cb, 20)
    gate = Gate(nav)
    global SIM
    SIM = sim
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    nav.create_subscription(
        PoseWithCovarianceStamped, 'amcl_pose',
        lambda m: AMCL.update(cert=math.sqrt(max(m.pose.covariance[0], 0) + max(m.pose.covariance[7], 0)),
                              x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                              yaw=2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))

    # ---------------- checks BEFORE GO (nothing moves yet)
    # robotcompete switches the camera + AI ON by itself for the run (Ngigi's choice for the
    # competition - no robotdrive / C key needed) and OFF again at the end.
    cam_pub = nav.create_publisher(Bool, 'camera_cmd', 10)
    laptop_ai = None
    if not sim and waste_forced is None:
        print('Switching the camera + AI on ...', flush=True)
        try:
            running = subprocess.run(['pgrep', '-f', 'waste_laptop.py'], capture_output=True).returncode == 0
            if not running:
                laptop_ai = subprocess.Popen(
                    ['python3', os.path.expanduser('~/waste_laptop.py'), pi_host],
                    stdin=subprocess.DEVNULL, stdout=open(os.path.expanduser('~/camera_log.txt'), 'w'),
                    stderr=subprocess.STDOUT, start_new_session=True)
        except Exception as e:
            print(f'  (laptop AI could not start: {e} - the Pi AI is used, slower)')
        t = time.monotonic()
        while ai_items(pi_host) is None and time.monotonic() - t < 60:
            cam_pub.publish(Bool(data=True))
            rclpy.spin_once(nav, timeout_sec=0.5)
            time.sleep(0.5)
        if ai_items(pi_host) is None:
            print('CAMERA / AI did not come on in 60 s (see ~/camera_log.txt). '
                  'Press Enter to run anyway (it will guess PLASTIC at the screen), or Ctrl+C.')
            input()
        else:
            print('Camera + AI: ON', flush=True)
    print(f'\nMap "{name}" places loaded.  Load wait {LOAD_WAIT:.0f} s, gate wait {GATE_WAIT:.0f} s.')
    if not nav2_active(nav):
        print('Waiting for navigation (Nav2) to be ACTIVE (is robotrun running?) ...', flush=True)
        wait_nav2(nav, 60.0)
        print('Nav2: ACTIVE' if nav2_active(nav) else 'WARNING: Nav2 is NOT active - goals will fail.')
    me = where_am_i(nav)
    if me is None:
        print('(robot position not received yet - fine if robotrun said ALIGNED OK)')
    else:
        print(f'Robot now at x {me[0]:.2f}  y {me[1]:.2f}  (certainty +-{me[2]:.2f} m)')
        far = [k for k in need if math.hypot(P[k]['x'] - me[0], P[k]['y'] - me[1]) > 6.0]
        for k in need:
            d = math.hypot(P[k]['x'] - me[0], P[k]['y'] - me[1])
            print(f'   {k:16s} x {P[k]["x"]:6.2f}  y {P[k]["y"]:6.2f}   {d:4.1f} m away'
                  + ('   <-- VERY FAR: marked wrong?' if k in far else ''))
    if me is not None and me[2] > 0.10:
        print(f'Position not sure enough (+-{me[2]:.2f} m) - finding the robot on the map first ...')
        refind()
    check_marks_on_map(nav, dict(P, map=name))
    input('Press Enter on the judge\'s GO ...')
    T0 = time.monotonic()
    say(f'===== RUN START  (map {name}) =====')
    try:
        # 1. screen + AI
        drive(nav, 'SCREEN', [P['screen']])
        if waste_forced:
            waste = waste_forced.lower()
            say(f'   (practice) waste type given: {waste.upper()}')
        else:
            waste = read_waste(pi_host)
            if not sim:
                # waste type known: switch the camera + AI off now, so the Pi has all its power
                # for driving (23 Nov: busy Pi -> late position data -> Nav2 restarted mid-run)
                cam_pub.publish(Bool(data=False))
                if laptop_ai is not None:
                    try:
                        os.killpg(laptop_ai.pid, signal.SIGTERM)
                    except Exception:
                        pass
                    laptop_ai = None
                say('   camera + AI off (Pi keeps its power for driving)')
        # 2. loading station
        drive(nav, f'{waste.upper()} LOADING station', [P[f'load_{waste}']])
        say(f'   waiting {LOAD_WAIT:.0f} s to be loaded ...')
        time.sleep(LOAD_WAIT)
        # 3. route to the gate
        if SHORT:
            say('   SHORT mission: route skipped - straight to the offloading station')
        elif not use_gate:
            # no gate marked: route (grass, obstacles, ramp) + detour, then straight to offloading
            # 24 Nov: the route used to be ONE task through all points - one blocked point made the
            # whole route fail. Now each point is its own goal (with its own retry); a point that
            # cannot be reached is skipped and the robot goes on to the next one.
            # 24 Nov finals: no gate used -> the detour point is not needed (it only added a leg that
            # failed after the ramp). Route = grass, obstacles, ramp; then straight to offloading.
            pts = named_points(P, 'route_to_gate')
            names = [nm for nm, _ in pts]
            ramp = 'route_ramp_top' in names and 'route_ramp_bottom' in names
            for nm, pose in pts:
                if ramp and nm == 'route_ramp_top':
                    ramp_drive(nav, cmd, guard_pub, pose, pts[names.index('route_ramp_bottom')][1])
                    continue
                if ramp and nm == 'route_ramp_bottom':
                    continue                     # already driven over by ramp_drive
                drive(nav, nm.replace('_', ' ').upper(), [pose])
        else:
            drive(nav, 'ROUTE to the gate', list(P.get('route_to_gate', [])) + [P['gate_check']])
        # 4. gate decision
        t = time.monotonic()
        d = gate.clear_ahead() if use_gate else None
        if SHORT:
            use_gate = False
        while use_gate and (d is None or d < GATE_CLEAR) and time.monotonic() - t < GATE_WAIT:
            time.sleep(0.2)
            d = gate.clear_ahead()
        if not use_gate:
            pass
        elif d is not None and d >= GATE_CLEAR:
            say(f'   GATE OPEN (clear {d:.2f} m ahead) -> gated path')
            drive(nav, 'through the GATE', [P['after_gate']])
        else:
            say(f'   GATE CLOSED after {time.monotonic() - t:.0f} s -> detour via the ramp')
            drive(nav, 'DETOUR via the ramp', list(P.get('detour', [])) + [P['after_gate']])
        # 5. offloading = finish
        drive(nav, f'{waste.upper()} OFFLOADING station (finish)', [P[f'offload_{waste}']])
        cmd.publish(Twist())
        say('   bin UP')
        dump.publish(Float64(data=DUMP_UP))
        time.sleep(DUMP_TIME)
        dump.publish(Float64(data=0.0))
        say('   bin DOWN')
        time.sleep(1.0)
        cmd.publish(Twist())
        say(f'===== RUN COMPLETE in {time.monotonic() - T0:.0f} s  (waste {waste.upper()}) =====')
    except KeyboardInterrupt:
        try:
            nav.cancelTask()
        except Exception:
            pass
        say('STOPPED by Ctrl+C')
    try:                                          # after Ctrl+C ROS may already be closed
        for _ in range(5):
            cmd.publish(Twist())
            time.sleep(0.05)
    except Exception:
        pass
    if not sim and waste_forced is None:          # camera + AI off again after the run
        try:
            cam_pub.publish(Bool(data=False))
        except Exception:
            pass
        if laptop_ai is not None:
            try:
                os.killpg(laptop_ai.pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(0.3)
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
