#!/usr/bin/env python3
"""
JNNP RoboDojo - ARROW-KEY teleop for ROS 2 (Gazebo and real robot, via /cmd_vel).   Version 7

Keys:
    ↑  forward   (hold)          ↓  backward  (hold)
    ←  turn LEFT  one step       →  turn RIGHT one step
       (one tap = one clean turn, measured with /odom, then the robot stops)
    SPACE    stop now (also cancels a turn)
    +  / -   faster / slower (forward speed AND turn speed)
    [  / ]   smaller / bigger turn step (15, 30, 45, 60, 90 degrees) - starts at 60
    U        dump UP   (back cabin to 40 deg - robot stops first)
    N        dump DOWN (back cabin to 0 deg)
    M / K / B  mark this place / skip it / UNDO the last mark (then drive there and press M again)
    C        camera ON / OFF
               Gazebo     : opens a viewer window with the simulated camera
               real robot : (started with robotdrive) switches the Pi camera + waste AI on/off
                            AND opens/closes a laptop window with the live camera + fast laptop
                            AI (waste_laptop.py). Browser http://<PI-IP>:8000 still works too.
    q        quit

Run (after sourcing ROS):
    python3 arrow_teleop.py
"""
import curses
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64
import yaml
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

FORWARD_SPEED = 0.15     # m/s
MAX_SPEED = 0.6
STEP = 0.05              # + / - change of forward speed
TURN_SPEED = 1.0         # rad/s top turning speed (60 deg in about 1.8 s, smooth)
TURN_MIN = 0.12          # rad/s at the very end of a turn
SLOW_ZONE = math.radians(30)   # start slowing down this far before the end of the turn
TURN_ACCEL = 2.5         # rad/s per second - smooth speed-up AND slow-down (no jerk)
TURN_STEPS = [15, 30, 45, 60, 90]
TURN_STEP_INDEX = 3      # start with 60 degree turns
TURN_TOLERANCE = math.radians(0.5)
HOLD_TIME = 0.8          # s after each turn: robot actively HOLDS its heading (stops the swing back)
HOLD_GAIN = 2.0          # how hard it resists being swung
HOLD_MAX = 0.25          # rad/s - gentle correction only
HOLD_DEADBAND = math.radians(1.0)
RELEASE_TIMEOUT = 0.55   # s - a single TAP of ↑/↓ drives this long (the keyboard only starts
                         #     repeating a held key after ~0.5 s, so we must wait that long once)
RELEASE_FAST = 0.12      # s - once the key is repeating (held), the robot stops 0.12 s after you
                         #     let go (was 0.6 s every time - the "slow to stop" feeling)
DUMP_UP = math.radians(40)      # same as the real robot: up 40 deg
DUMP_DOWN = 0.0                 # down 0 deg
CAMERA_CMD = ["ros2", "run", "rqt_image_view", "rqt_image_view", "/camera/image_raw"]
REAL_ROBOT = os.environ.get("ROBODOJO_REAL") == "1"          # set by the robotdrive shortcut
PI_IP = os.environ.get("PI", "user@PI-IP").split("@")[-1]

# ---- REAL ROBOT turns: clean "stop-and-done" turns, no swinging back ----
# The real robot's wheels need a strong push to spin (minimum power), so slowing down gently
# at the end of a turn and "holding" the heading afterwards make it jerk back and forth (the swing).
# Instead it turns at one steady speed and cuts the power a little EARLY, so that it coasts
# exactly onto the target angle, then brakes and stays still.  After every turn it measures how
# far it really went and corrects the "early" amount for the next turn (it learns, per side).
REAL_TURN_SPEED = 0.8        # rad/s steady turning speed on the real robot
REAL_TURN_ACCEL = 5.0        # rad/s per second - quick, firm start
REAL_LEAD_START = 0.15       # s - first guess of how long the robot keeps turning after "stop"
REAL_LEAD_MIN, REAL_LEAD_MAX = 0.0, 0.6
REAL_LEARN = 0.6             # how much of each turn's error is corrected next time
REAL_SETTLE = 0.6            # s to wait after a turn before measuring where it ended
GZ_SETTLE = 0.2              # Gazebo: exact position, so the next ← / → is accepted after 0.2 s
CLEAN_TURNS = True           # use these clean turns in Gazebo too (False = old Gazebo slow-down + hold)
# HOLDING ← / → : the robot keeps turning smoothly for as long as the key is held, and stops
# cleanly (brake, no swing back) as soon as it is let go. A short tap still makes one step turn.
KEY_RELEASE = 0.25           # s without a repeated ← / → = the key was let go
HOLD_REPEATS = 3             # repeated key events needed to count as "held"
HOLD_MIN_TIME = 0.3          # s - keyboards start repeating a held key after ~0.3-0.5 s
TURN_LOG = os.path.expanduser("~/turn_log.txt")
# ---- MARKING the competition places WHILE MAPPING (key M) ----
# Drive to each place in this order and press M there (facing the way the robot should face).
# K = skip the current place.  Saved to MARKS_FILE; robotsavemap NAME / savemap keeps them with the map.
MARKS_FILE = os.path.expanduser("~/.robodojo/marks_current.yaml")
MARK_ORDER = [
    ('screen', 'SCREEN (yellow) - stop in front of it, FACING the screen'),
    ('load_plastic', 'PLASTIC loading station (blue)'),
    ('load_organic', 'ORGANIC loading station (green)'),
    ('load_metal', 'METAL loading station (red)'),
    ('route_grass', 'middle of the GRASS zone'),
    ('route_obstacles', 'middle of the OBSTACLE zone'),
    ('route_ramp_top', 'TOP of the RAMP'),
    ('route_ramp_bottom', 'BOTTOM of the RAMP'),
    ('gate_check', 'just BEFORE the RAMP GATE, facing through it'),
    ('after_gate', 'just AFTER the ramp gate'),
    ('detour', 'DETOUR way round (only if there is one - else press K)'),
    ('offload_plastic', 'PLASTIC offloading station (blue) = finish'),
    ('offload_organic', 'ORGANIC offloading station (green) = finish'),
    ('offload_metal', 'METAL offloading station (red) = finish'),
]


def load_marks():
    try:
        return yaml.safe_load(open(MARKS_FILE)) or {}
    except Exception:
        return {}


def save_marks(marks):
    """Write the marks in the format robotcompete reads (route + detour as lists)."""
    os.makedirs(os.path.dirname(MARKS_FILE), exist_ok=True)
    out = {k: v for k, v in marks.items() if not k.startswith('route_') and k != 'detour'}
    out['route_to_gate'] = [marks[k] for k in ('route_grass', 'route_obstacles', 'route_ramp_top',
                                               'route_ramp_bottom') if k in marks]
    out['detour'] = [marks['detour']] if 'detour' in marks else []
    out['_raw'] = marks
    with open(MARKS_FILE, 'w') as f:
        f.write('# Marked with key M in the teleop while mapping. Map frame, metres, yaw radians.\n')
        yaml.safe_dump(out, f, sort_keys=False)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def stop_camera(cam):
    if cam is not None and cam.poll() is None:
        try:
            os.killpg(cam.pid, signal.SIGTERM)
            cam.wait(timeout=3)
        except Exception:
            pass


def main(screen):
    rclpy.init()
    node = rclpy.create_node("arrow_teleop")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    dump_pub = node.create_publisher(Float64, "dump_cmd", 10)
    odom = {'yaw': None}
    real_cam = {'on': None}                   # real robot camera state (from the Pi)
    cam_cmd_pub = node.create_publisher(Bool, "camera_cmd", 10)
    node.create_subscription(Bool, "camera_on", lambda m: real_cam.__setitem__('on', m.data), 10)
    # Gazebo on a laptop often runs SLOWER than real time (e.g. 0.4x). Turn timing (time limit,
    # waiting for the robot to settle, learning) therefore uses Gazebo's own clock (/clock).
    # The real robot has no /clock and simply uses the normal clock.
    simclock = {'t': None}
    if not REAL_ROBOT:
        node.create_subscription(Clock, "clock",
                                 lambda m: simclock.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9), 10)

    def robot_time():
        return simclock['t'] if simclock['t'] is not None else time.time()

    tf_buf = Buffer()
    TransformListener(tf_buf, node)
    marks = (load_marks().get('_raw') or {})

    def map_pose():
        t = tf_buf.lookup_transform('map', 'base_footprint', Time())
        q = t.transform.rotation
        return {'x': round(t.transform.translation.x, 3), 'y': round(t.transform.translation.y, 3),
                'yaw': round(yaw_of(q), 3)}

    def next_mark():
        return next(((k, t) for k, t in MARK_ORDER if k not in marks and k + '_skip' not in marks), None)

    node.create_subscription(Odometry, "odom",
                             lambda m: odom.__setitem__('yaw', yaw_of(m.pose.pose.orientation)), 10)

    curses.curs_set(0)
    screen.nodelay(True)
    screen.keypad(True)

    speed, step_i = FORWARD_SPEED, TURN_STEP_INDEX
    turn_speed = REAL_TURN_SPEED if REAL_ROBOT else TURN_SPEED
    lead = {1: REAL_LEAD_START, -1: REAL_LEAD_START}   # learned "stop early" time per side
    settle = None             # real robot: measure the end of the last turn
    last_turn_sign, last_turn_stop, last_turn_held = 0, -10.0, False
    last_turn_text = "-"
    lin = ang = 0.0
    action = "STOP"
    last_key = 0.0
    drive_key = {'k': None, 'reps': 0}   # ↑/↓ key repeat tracking (fast stop on release)
    last_loop = time.time()
    turning = None            # dict while a turn step is running
    settle_until = 0.0
    hold = None               # heading to hold right after a turn
    dump_state = "DOWN (0 deg)"
    camera = None             # the camera viewer program
    note = ""
    quit_now = False
    moving_until = 0.0        # keep sending until this time (then go quiet)

    while True:
        rclpy.spin_once(node, timeout_sec=0)
        keys = []
        while True:
            k = screen.getch()
            if k == -1:
                break
            keys.append(k)
        now = time.time()
        dt = min(0.1, now - last_loop)
        last_loop = now
        if ord(" ") in keys:
            keys = [ord(" ")]

        for key in keys[-1:]:
            if key == ord(" "):
                lin, ang, action, turning, hold = 0.0, 0.0, "STOP", None, None
                drive_key.update(k=None, reps=0)
                pub.publish(Twist())              # stop instantly
                moving_until = now + 0.5          # SPACE always sends STOP (also stops Nav2 moves)
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT) and (turning or now < settle_until):
                sign = 1 if key == curses.KEY_LEFT else -1
                side = 'LEFT' if sign > 0 else 'RIGHT'
                if turning and turning['sign'] == sign:
                    turning['reps'] += 1
                    turning['last_rep'] = now
                    # HELD only when the keyboard keeps repeating the key (3+ repeats, after the
                    # normal repeat delay) - one stray extra key event can no longer cause a long turn
                    if turning['reps'] >= HOLD_REPEATS and now - turning['t0'] > HOLD_MIN_TIME:
                        turning.update(held=True, was_held=True, end_time=robot_time() + 2.0)
                        action = f"TURN {side} (key held)"
                elif (not turning and last_turn_held and last_turn_sign == sign
                      and now - last_turn_stop < 0.15):
                    # still held right after a step turn ended: carry on turning, no pause
                    turning = {'sign': sign, 'start': odom['yaw'], 'goal': 0.0, 't0': now,
                               'acc': 0.0, 'prev': odom['yaw'], 'held': True, 'was_held': True,
                               'reps': HOLD_REPEATS, 'last_rep': now, 'end_time': robot_time() + 2.0}
                    settle, hold, lin = None, None, 0.0
                    action = f"TURN {side} (key held)"
                # anything else (other arrow while turning, repeats while settling) is ignored
            elif key in (curses.KEY_UP, curses.KEY_DOWN):
                if drive_key['k'] == key and now - last_key < RELEASE_TIMEOUT:
                    drive_key['reps'] += 1           # the keyboard is repeating = key is held
                else:
                    drive_key.update(k=key, reps=0)
                fwd = key == curses.KEY_UP
                lin, ang, turning, hold = (speed if fwd else -speed), 0.0, None, None
                action = "FORWARD" if fwd else "BACKWARD"
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
                sign = 1 if key == curses.KEY_LEFT else -1
                step = math.radians(TURN_STEPS[step_i])
                settle = None          # a new turn before the last one was measured: skip that one
                turning = {'sign': sign, 'start': odom['yaw'], 'goal': step, 't0': now,
                           'acc': 0.0, 'prev': odom['yaw'], 'held': False, 'was_held': False,
                           'reps': 0, 'last_rep': now,
                           'end_time': robot_time() + step / turn_speed * 1.6 + 1.0}
                lin, ang, hold = 0.0, 0.0, None
                action = f"TURN {'LEFT' if sign > 0 else 'RIGHT'} {TURN_STEPS[step_i]} deg"
            elif key in (ord("+"), ord("=")):
                speed = min(MAX_SPEED, round(speed + STEP, 2))
                turn_speed = min(2.5 if not REAL_ROBOT else 1.6, round(turn_speed + 0.2, 2))
            elif key in (ord("-"), ord("_")):
                speed = max(0.05, round(speed - STEP, 2))
                turn_speed = max(0.4, round(turn_speed - 0.2, 2))
                lead = {1: REAL_LEAD_START, -1: REAL_LEAD_START}   # new speed: learn again
            elif key == ord("["):
                step_i = max(0, step_i - 1)
            elif key == ord("]"):
                step_i = min(len(TURN_STEPS) - 1, step_i + 1)
            elif key in (ord("u"), ord("U")):
                lin, ang, action, turning = 0.0, 0.0, "STOP", None
                pub.publish(Twist())               # stop before dumping
                dump_pub.publish(Float64(data=DUMP_UP))
                dump_state = "UP (40 deg)"
                if dump_pub.get_subscription_count() == 0:
                    dump_state += "  - NOT SENT: robot engine not running (start robotengine / robotrun)"
            elif key in (ord("n"), ord("N")):
                dump_pub.publish(Float64(data=DUMP_DOWN))
                dump_state = "DOWN (0 deg)"
                if dump_pub.get_subscription_count() == 0:
                    dump_state += "  - NOT SENT: robot engine not running (start robotengine / robotrun)"
            elif key in (ord("c"), ord("C")) and REAL_ROBOT:
                turn_on = not real_cam['on']
                cam_cmd_pub.publish(Bool(data=turn_on))
                real_cam['on'] = turn_on
                stop_camera(camera)
                camera = None
                if turn_on:              # open the laptop camera window (fast laptop AI)
                    try:
                        log = open(os.path.expanduser("~/camera_log.txt"), "w")
                        camera = subprocess.Popen(
                            ["python3", os.path.expanduser("~/waste_laptop.py"), PI_IP],
                            stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
                        note = "Camera ON - its window opens in a few s. CLICK BACK HERE to drive."
                    except Exception as e:
                        note = f"Camera ON, but the window could not open: {e}"
                else:
                    note = "Camera + waste AI OFF"
            elif key in (ord("c"), ord("C")):
                if camera is not None and camera.poll() is None:
                    stop_camera(camera)
                    camera, note = None, ""
                else:
                    try:
                        camera = subprocess.Popen(CAMERA_CMD, stdin=subprocess.DEVNULL,
                                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                                  start_new_session=True)
                        note = "Camera window opening... then CLICK BACK INSIDE THIS WINDOW to drive"
                    except Exception as e:
                        camera, note = None, f"Camera viewer could not start: {e}"
            elif key in (ord("m"), ord("M")):
                nm = next_mark()
                if nm is None:
                    note = "All places are marked. (Delete ~/.robodojo/marks_current.yaml to start again.)"
                else:
                    try:
                        marks[nm[0]] = map_pose()
                        save_marks(marks)
                        p = marks[nm[0]]
                        note = f"MARKED {nm[0]}  (x {p['x']:.2f}, y {p['y']:.2f})"
                    except Exception:
                        note = "Cannot mark: no map position yet (is robotmap / robotrun / gzmap running?)"
            elif key in (ord("k"), ord("K")):
                nm = next_mark()
                if nm is not None:
                    marks[nm[0] + '_skip'] = True
                    save_marks(marks)
                    note = f"skipped {nm[0]}"
            elif key in (ord("b"), ord("B"), curses.KEY_BACKSPACE, 127, 8):
                last = next((k for k, _ in reversed(MARK_ORDER)
                             if k in marks or k + '_skip' in marks), None)
                if last is None:
                    note = "Nothing marked yet - nothing to undo."
                else:
                    marks.pop(last, None)
                    marks.pop(last + '_skip', None)
                    save_marks(marks)
                    note = f"UNDONE {last} - drive there and press M again"
            elif key in (ord("q"), ord("Q")):
                quit_now = True
            last_key = now
        if quit_now:
            break

        if turning:
            done = False
            if turning['start'] is None:
                turning['start'] = turning['prev'] = odom['yaw']
            have_odom = turning['start'] is not None and odom['yaw'] is not None
            if have_odom:
                # add up the turn bit by bit (works past 180 deg when the key is held)
                turning['acc'] += abs(angle_diff(odom['yaw'], turning['prev']))
                turning['prev'] = odom['yaw']
                left = turning['goal'] - turning['acc']
            if turning['held'] and now - turning['last_rep'] > KEY_RELEASE:
                turning['held'] = False              # key let go: finish (at least the step) and stop
            if REAL_ROBOT or CLEAN_TURNS:
                # steady speed, cut the power early by the learned coasting time
                target = turn_speed if have_odom else turn_speed * 0.6
                if turning['held']:
                    done = False                     # keep turning while the key is held
                elif turning['was_held']:
                    done = not have_odom or left <= abs(ang) * lead[turning['sign']]
                elif have_odom:
                    done = left <= abs(ang) * lead[turning['sign']]
                accel = REAL_TURN_ACCEL
            else:
                if have_odom:
                    done = left <= TURN_TOLERANCE
                    target = max(TURN_MIN, min(turn_speed, turn_speed * left / SLOW_ZONE))
                else:
                    target = turn_speed * 0.6            # no /odom: gentle timed turn
                accel = TURN_ACCEL
            # smooth speed-up: never jump straight to full turning speed
            target_ang = turning['sign'] * target
            max_change = accel * dt
            ang += max(-max_change, min(max_change, target_ang - ang))
            rt = robot_time()
            if done or rt > turning['end_time']:
                last_turn_sign, last_turn_stop = turning['sign'], now
                last_turn_held = turning['was_held']
                if (REAL_ROBOT or CLEAN_TURNS) and have_odom:
                    # measure where it ends (tap turns: also learn from it)
                    settle = {'sign': turning['sign'], 'start': turning['start'],
                              'goal': turning['goal'], 'rate': max(0.2, abs(ang)),
                              'at': rt + REAL_SETTLE, 'learn': not turning['was_held'],
                              'reps': turning['reps'], 'at_stop': turning['acc'],
                              'lead': lead[turning['sign']], 'timeout': rt > turning['end_time']}
                turning, lin, ang, action = None, 0.0, 0.0, "STOP"
                clean = REAL_ROBOT or CLEAN_TURNS
                settle_until = now + (REAL_SETTLE if REAL_ROBOT else GZ_SETTLE if clean else HOLD_TIME)
                if odom['yaw'] is not None and not clean:        # old Gazebo mode: lock heading
                    hold = {'yaw': odom['yaw'], 'until': now + HOLD_TIME}
                curses.flushinp()                        # throw away repeated ← / → presses
        elif hold:
            # "active brake" for turning: if the robot starts to swing back, push gently against it
            err = angle_diff(hold['yaw'], odom['yaw']) if odom['yaw'] is not None else 0.0
            ang = 0.0 if abs(err) < HOLD_DEADBAND else max(-HOLD_MAX, min(HOLD_MAX, HOLD_GAIN * err))
            lin = 0.0
            if now > hold['until']:
                hold, ang = None, 0.0
        elif not keys and action not in ("STOP",) and \
                now - last_key > (RELEASE_FAST if drive_key['reps'] >= 2 else RELEASE_TIMEOUT):
            lin, ang, action = 0.0, 0.0, "STOP"
            drive_key.update(k=None, reps=0)
            pub.publish(Twist())                     # stop NOW (not on the next loop)

        if settle and robot_time() >= settle['at'] and odom['yaw'] is not None:
            final = abs(angle_diff(odom['yaw'], settle['start']))
            err = final - settle['goal']                 # + = went too far, - = stopped short
            k = settle['sign']
            if settle['learn']:
                lead[k] = max(REAL_LEAD_MIN, min(REAL_LEAD_MAX, lead[k] + REAL_LEARN * err / settle['rate']))
                last_turn_text = (f"{math.degrees(final):.0f} deg (asked {math.degrees(settle['goal']):.0f}, "
                                  f"{'+' if err >= 0 else ''}{math.degrees(err):.0f})")
            else:
                last_turn_text = f"{math.degrees(final):.0f} deg (key held)"
            # every turn is written to ~/turn_log.txt so a problem can be seen in numbers
            try:
                with open(TURN_LOG, 'a') as f:
                    f.write(f"{time.strftime('%H:%M:%S')}  {'RIGHT' if k < 0 else 'LEFT ':5}  "
                            f"{'HELD' if not settle['learn'] else 'tap ':4}  asked {math.degrees(settle['goal']):5.1f}  "
                            f"at_stop {math.degrees(settle['at_stop']):6.1f}  final {math.degrees(final):6.1f}  "
                            f"coast {math.degrees(final - settle['at_stop']):5.1f}  key_events {settle['reps'] + 1:3d}  "
                            f"lead {settle['lead']:.2f}s  speed {settle['rate']:.2f}"
                            f"{'  TIMEOUT' if settle['timeout'] else ''}  {'REAL' if REAL_ROBOT else 'GAZEBO'}\n")
            except OSError:
                pass
            settle = None

        # Only talk while driving (and for 0.5 s after stopping, so the stop surely arrives).
        # When idle it stays QUIET, so Nav2 can drive the robot without this teleop fighting it.
        if lin != 0.0 or ang != 0.0:
            moving_until = now + 0.5
        if now < moving_until:
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            pub.publish(msg)

        if camera is not None and camera.poll() is not None:
            camera = None
            note = ("Camera window closed (problem? see ~/camera_log.txt). C = camera off, C again = reopen."
                    if REAL_ROBOT else "Camera window closed. Press C to open it again.")

        if REAL_ROBOT:
            cam_text = ("waiting for the robot..." if real_cam['on'] is None else
                        ("ON  (window open, laptop AI)" if camera is not None else f"ON  (http://{PI_IP}:8000)")
                        if real_cam['on'] else "OFF  (press C to turn on)")
        else:
            cam_text = "ON (separate window)" if camera is not None else "OFF"
        try:
            screen.erase()
            screen.addstr(0, 0, "JNNP RoboDojo - arrow teleop (v7) - " + ("REAL ROBOT" if REAL_ROBOT else "GAZEBO"))
            screen.addstr(2, 0, "  ^   forward (hold)       v   backward (hold)")
            screen.addstr(3, 0, "  <-  turn left one step   ->  turn right one step")
            screen.addstr(4, 0, "  SPACE stop   + faster   - slower   [ ] turn step")
            screen.addstr(5, 0, "  U dump up   N dump down   C camera on/off   q quit")
            nm = next_mark()
            screen.addstr(13, 0, ("  M mark place -> next: " + nm[1] + "   (K skip, B undo last)") if nm
                          else "  All competition places marked.")
            screen.addstr(7, 0, f"  Speed: {speed:.2f} m/s   Turn speed: {turn_speed:.1f} rad/s   "
                                f"Turn step: {TURN_STEPS[step_i]} deg")
            screen.addstr(8, 0, f"  Now:    {action}")
            screen.addstr(9, 0, f"  Dump:   {dump_state}")
            screen.addstr(10, 0, f"  Camera: {cam_text}")
            screen.addstr(11, 0, "  /odom:  " + ("OK (exact turns)" if odom['yaw'] is not None
                                                 else "not received - turns are timed"))
            if REAL_ROBOT or CLEAN_TURNS:
                screen.addstr(12, 0, f"  Last turn: {last_turn_text}")
            if note:
                screen.addstr(15, 0, "  " + note)
            screen.refresh()
        except curses.error:
            pass                     # window too small - keep driving anyway
        time.sleep(0.02)

    stop_camera(camera)
    pub.publish(Twist())          # stop the robot on exit
    time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:          # Ctrl+C: stop the robot and close quietly
        try:
            n = rclpy.create_node("arrow_teleop_stop") if rclpy.ok() else None
            if n is not None:
                p = n.create_publisher(Twist, "cmd_vel", 10)
                for _ in range(5):
                    p.publish(Twist())
                    time.sleep(0.05)
        except Exception:
            pass
        print("Teleop closed (robot stopped).")
