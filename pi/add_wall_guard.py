#!/usr/bin/env python3
"""
JNNP RoboDojo - add the WALL GUARD to the robot engine on the Pi (robodojo_ros.py).

Run ON THE PI (the installer does it):   python3 ~/add_wall_guard.py

What the guard does (inside the engine, for EVERY drive command - Nav2 or teleop):
  * It looks at the raw LiDAR points in a box just IN FRONT of the robot (and just BEHIND it).
  * Something there?  ->  driving forward (or back) into it is refused: the robot stops.
    Turning on the spot is still allowed, so Nav2 can back away / turn and plan a new way.
  * It uses the raw points, so it also sees walls very close to the robot (the normal
    obstacle map ignores everything nearer than 22 cm - that is why the robot hit wall ends).
It does not depend on where the robot THINKS it is on the map: it works even when lost.

Safe to run again: it only patches once. A backup is kept (robodojo_ros.py.before_guard).
Switch it off without uninstalling:  add  "WALL_GUARD": false  to ~/robodojo_calib.json
"""
import glob
import re
import os
import shutil
import sys

MARK = '# --- WALL GUARD (add_wall_guard.py) ---'

CONSTS = '''
''' + MARK + '''
WALL_GUARD = True          # refuse to drive INTO something right in front of / behind the robot
GUARD_HALF_LEN = 0.13      # m, robot centre to its front (and back) edge  (26 x 28 cm robot)
GUARD_HALF_WID = 0.14      # m, robot centre to its side
GUARD_GAP = 0.10           # m, look this far in front of the front edge (and behind the back edge)
GUARD_SKIP = 0.02          # m, ignore the first 2 cm right at the edge (bumper / cables of the robot)
GUARD_MIN_POINTS = 2       # LiDAR points needed in the box to count as "blocked"
GUARD_MAX_AGE = 0.6        # s, older LiDAR data does not block (LiDAR stopped -> normal behaviour)
try:
    WALL_GUARD = bool(_cal.get('WALL_GUARD', WALL_GUARD))
except NameError:
    pass
'''

INIT = '''        self.block_fwd = False       ''' + MARK + '''
        self.block_back = False
        self.guard_time = 0.0
        self.guard_logged = 0.0
        self.guard_on = True         # robotcompete switches it off only while driving over the RAMP
        self.create_subscription(Bool, 'guard_enable', lambda m: setattr(self, 'guard_on', bool(m.data)), 10)
'''

SCAN = '''                    ''' + MARK + '''
                    if WALL_GUARD:
                        nf = nb = 0
                        for a, d in turn:
                            if d <= 0.02 or d > 1.0:
                                continue
                            rel = a - LIDAR_FRONT_ANGLE
                            ang = math.radians(-rel if LIDAR_CLOCKWISE else rel)
                            bx = LASER_X + d * math.cos(ang)
                            by = LASER_Y + d * math.sin(ang)
                            if abs(by) > GUARD_HALF_WID + 0.02:
                                continue
                            if GUARD_HALF_LEN + GUARD_SKIP < bx < GUARD_HALF_LEN + GUARD_GAP:
                                nf += 1
                            elif -GUARD_HALF_LEN - GUARD_GAP < bx < -GUARD_HALF_LEN - GUARD_SKIP:
                                nb += 1
                        self.block_fwd = nf >= GUARD_MIN_POINTS
                        self.block_back = nb >= GUARD_MIN_POINTS
                        self.guard_time = time.time()
'''

CMD = '''        ''' + MARK + '''
        if WALL_GUARD and getattr(self, 'guard_on', True) and time.time() - self.guard_time < GUARD_MAX_AGE:
            if (v > 0.0 and self.block_fwd) or (v < 0.0 and self.block_back):
                v = 0.0                                    # no driving INTO it - turning still OK
                if time.time() - self.guard_logged > 2.0:
                    self.guard_logged = time.time()
                    self.get_logger().warn('WALL GUARD: something right '
                                           + ('in front' if self.block_fwd else 'behind')
                                           + ' - not driving into it')
'''


MARK2 = '# --- CORNER GUARD (add_wall_guard.py) ---'

CONSTS2 = '''
''' + MARK2 + '''
CORNER_GAP = 0.04          # m: something this close to a robot CORNER blocks turning that corner into it
CORNER_REACH = math.hypot(0.13, 0.14) + 0.05
'''

SCAN2 = '''                    ''' + MARK2 + '''
                    if WALL_GUARD:
                        nccw = ncw = 0
                        corners = ((GUARD_HALF_LEN, GUARD_HALF_WID), (GUARD_HALF_LEN, -GUARD_HALF_WID),
                                   (-GUARD_HALF_LEN, GUARD_HALF_WID), (-GUARD_HALF_LEN, -GUARD_HALF_WID))
                        for a, d in turn:
                            if d <= 0.02 or d > 0.4:
                                continue
                            rel = a - LIDAR_FRONT_ANGLE
                            ang = math.radians(-rel if LIDAR_CLOCKWISE else rel)
                            bx = LASER_X + d * math.cos(ang)
                            by = LASER_Y + d * math.sin(ang)
                            if abs(bx) <= GUARD_HALF_LEN + 0.005 and abs(by) <= GUARD_HALF_WID + 0.005:
                                continue                       # the robot's own body
                            if math.hypot(bx, by) > CORNER_REACH:
                                continue
                            cx, cy = min(corners, key=lambda c: (bx - c[0]) ** 2 + (by - c[1]) ** 2)
                            if (bx - cx) ** 2 + (by - cy) ** 2 > CORNER_GAP ** 2:
                                continue
                            dot = (bx - cx) * (-cy) + (by - cy) * cx   # > 0: in the way of a LEFT turn
                            if dot > 0:
                                nccw += 1
                            elif dot < 0:
                                ncw += 1
                        self.block_ccw = nccw >= GUARD_MIN_POINTS
                        self.block_cw = ncw >= GUARD_MIN_POINTS
'''

CMD2 = '''        ''' + MARK2 + '''
        if WALL_GUARD and getattr(self, 'guard_on', True) and time.time() - self.guard_time < GUARD_MAX_AGE:
            if abs(v) < 0.05 and ((w > 0.0 and getattr(self, 'block_ccw', False)) or
                                  (w < 0.0 and getattr(self, 'block_cw', False))):   # turning on the spot only
                w = 0.0                                    # do not swing a corner into the wall
                if time.time() - self.guard_logged > 2.0:
                    self.guard_logged = time.time()
                    self.get_logger().warn('CORNER GUARD: a corner is close to something - not turning into it')
'''


def add_corner(src):
    """Add the corner guard (once). Needs the wall guard already in src."""
    if MARK2 in src:
        return src, False
    a1 = "                    s = LaserScan()\n"
    a2 = "        v, w = msg.linear.x, msg.angular.z\n"
    a3 = "GUARD_MAX_AGE = "
    if src.count(a1) != 1 or src.count(a2) != 1 or src.count(a3) != 1:
        return src, False
    j = src.index('\n', src.index(a3)) + 1
    src = src[:j] + CONSTS2 + src[j:]
    src = src.replace(a1, SCAN2 + a1, 1)
    src = src.replace(a2, a2 + CMD2, 1)
    return src, True


def find_engine():
    for p in [os.path.expanduser('~/robodojo_ros.py')] + sorted(
            glob.glob(os.path.expanduser('~/**/robodojo_ros.py'), recursive=True)):
        if os.path.isfile(p) and 'def on_cmd_vel' in open(p).read():
            return p
    return None


def main():
    path = find_engine()
    if not path:
        sys.exit('robodojo_ros.py (robot engine) not found on this Pi - nothing changed.')
    src = open(path).read()
    if MARK in src:
        new = re.sub(r'^GUARD_GAP = [0-9.]+', 'GUARD_GAP = 0.10', src, flags=re.M)
        new, added = add_corner(new)
        if added:
            print('Corner guard: ADDED - the robot will not swing a corner into a wall when turning')
        if "'guard_enable'" not in new:          # add the ramp on/off switch
            a1 = "        self.guard_logged = 0.0\n"
            a2 = "        if WALL_GUARD and time.time() - self.guard_time < GUARD_MAX_AGE:"
            if new.count(a1) == 1 and new.count(a2) == 1:
                new = new.replace(a1, a1 + "        self.guard_on = True\n        self.create_subscription(Bool, 'guard_enable', "
                                  "lambda m: setattr(self, 'guard_on', bool(m.data)), 10)\n", 1)
                new = new.replace(a2, "        if WALL_GUARD and getattr(self, 'guard_on', True) and "
                                  "time.time() - self.guard_time < GUARD_MAX_AGE:", 1)
                print('Wall guard  : ramp on/off switch added')
        if new != src:
            compile(new, path, 'exec')
            open(path, 'w').write(new)
            print(f'Wall guard  : UPDATED in {path} - now stops 10 cm before anything in front/behind')
        else:
            print(f'Wall guard  : already in {path} (stops 10 cm before anything in front/behind)')
        return
    anchors = [
        ("CMD_TIMEOUT = ", None),
        ("        self.target = (0, 0)\n        self.last_cmd = 0.0\n", INIT),
        ("                    s = LaserScan()\n", SCAN),
        ("        v, w = msg.linear.x, msg.angular.z\n", CMD),
    ]
    for a, _ in anchors:
        if src.count(a) != 1:
            sys.exit(f'Engine looks different than expected (anchor {a.strip()!r}) - nothing changed. '
                     'Send this message to Claude.')
    shutil.copy(path, path + '.before_guard')
    # constants: after the CMD_TIMEOUT line
    i = src.index('CMD_TIMEOUT = ')
    j = src.index('\n', i) + 1
    src = src[:j] + CONSTS + src[j:]
    a2 = "        self.target = (0, 0)\n        self.last_cmd = 0.0\n"
    src = src.replace(a2, INIT + a2, 1)
    src = src.replace("                    s = LaserScan()\n", SCAN + "                    s = LaserScan()\n", 1)
    src = src.replace("        v, w = msg.linear.x, msg.angular.z\n",
                      "        v, w = msg.linear.x, msg.angular.z\n" + CMD, 1)
    src, _ = add_corner(src)
    compile(src, path, 'exec')                    # never write a broken engine
    open(path, 'w').write(src)
    print(f'Wall guard  : ADDED to {path} - stops before driving into anything within 15 cm '
          f'of the front/back (backup: {os.path.basename(path)}.before_guard)')


if __name__ == '__main__':
    main()
