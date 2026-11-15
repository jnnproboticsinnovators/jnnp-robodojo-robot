#!/usr/bin/env python3
"""
JNNP RoboDojo - ARROW-KEY teleop for ROS 2 (Gazebo, via /cmd_vel).   Version 4

Keys:
    ↑  forward   (hold)          ↓  backward  (hold)
    ←  turn LEFT  one step       →  turn RIGHT one step
       (one tap = one clean turn, measured with /odom, then the robot stops)
    SPACE    stop now (also cancels a turn)
    +  / -   faster / slower (forward speed AND turn speed)
    [  / ]   smaller / bigger turn step (15, 30, 45, 60, 90 degrees) - starts at 60
    U        dump UP   (back cabin to 40 deg - robot stops first)
    N        dump DOWN (back cabin to 0 deg)
    C        camera ON / OFF
               Gazebo     : opens a viewer window with the simulated camera
               real robot : (started with robotdrive) switches the Pi camera + waste AI on/off,
                            watch it at http://<PI-IP>:8000
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
from std_msgs.msg import Bool, Float64

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
RELEASE_TIMEOUT = 0.6    # s without ↑/↓ key = robot stops
DUMP_UP = math.radians(40)      # same as the real robot: up 40 deg
DUMP_DOWN = 0.0                 # down 0 deg
CAMERA_CMD = ["ros2", "run", "rqt_image_view", "rqt_image_view", "/camera/image_raw"]
REAL_ROBOT = os.environ.get("ROBODOJO_REAL") == "1"          # set by the robotdrive shortcut
PI_IP = os.environ.get("PI", "user@PI-IP").split("@")[-1]


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
    node.create_subscription(Odometry, "odom",
                             lambda m: odom.__setitem__('yaw', yaw_of(m.pose.pose.orientation)), 10)

    curses.curs_set(0)
    screen.nodelay(True)
    screen.keypad(True)

    speed, turn_speed, step_i = FORWARD_SPEED, TURN_SPEED, TURN_STEP_INDEX
    lin = ang = 0.0
    action = "STOP"
    last_key = 0.0
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
                moving_until = now + 0.5          # SPACE always sends STOP (also stops Nav2 moves)
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT) and (turning or now < settle_until):
                pass                       # ignore key auto-repeat while turning / settling
            elif key == curses.KEY_UP:
                lin, ang, action, turning, hold = speed, 0.0, "FORWARD", None, None
            elif key == curses.KEY_DOWN:
                lin, ang, action, turning, hold = -speed, 0.0, "BACKWARD", None, None
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
                sign = 1 if key == curses.KEY_LEFT else -1
                step = math.radians(TURN_STEPS[step_i])
                turning = {'sign': sign, 'start': odom['yaw'], 'goal': step,
                           'end_time': now + step / turn_speed * 1.6 + 1.0}
                lin, ang, hold = 0.0, 0.0, None
                action = f"TURN {'LEFT' if sign > 0 else 'RIGHT'} {TURN_STEPS[step_i]} deg"
            elif key in (ord("+"), ord("=")):
                speed = min(MAX_SPEED, round(speed + STEP, 2))
                turn_speed = min(2.5, round(turn_speed + 0.2, 2))
            elif key in (ord("-"), ord("_")):
                speed = max(0.05, round(speed - STEP, 2))
                turn_speed = max(0.4, round(turn_speed - 0.2, 2))
            elif key == ord("["):
                step_i = max(0, step_i - 1)
            elif key == ord("]"):
                step_i = min(len(TURN_STEPS) - 1, step_i + 1)
            elif key in (ord("u"), ord("U")):
                lin, ang, action, turning = 0.0, 0.0, "STOP", None
                pub.publish(Twist())               # stop before dumping
                dump_pub.publish(Float64(data=DUMP_UP))
                dump_state = "UP (40 deg)"
            elif key in (ord("n"), ord("N")):
                dump_pub.publish(Float64(data=DUMP_DOWN))
                dump_state = "DOWN (0 deg)"
            elif key in (ord("c"), ord("C")) and REAL_ROBOT:
                turn_on = not real_cam['on']
                cam_cmd_pub.publish(Bool(data=turn_on))
                real_cam['on'] = turn_on
                note = (f"Camera + waste AI ON  -> open http://{PI_IP}:8000 in Chrome" if turn_on
                        else "Camera + waste AI OFF")
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
            elif key in (ord("q"), ord("Q")):
                quit_now = True
            last_key = now
        if quit_now:
            break

        if turning:
            done = False
            if turning['start'] is None:
                turning['start'] = odom['yaw']
            if turning['start'] is not None and odom['yaw'] is not None:
                turned = abs(angle_diff(odom['yaw'], turning['start']))
                left = turning['goal'] - turned
                done = left <= TURN_TOLERANCE
                target = max(TURN_MIN, min(turn_speed, turn_speed * left / SLOW_ZONE))
            else:
                target = turn_speed * 0.6                # no /odom: gentle timed turn
            # smooth speed-up: never jump straight to full turning speed
            target_ang = turning['sign'] * target
            max_change = TURN_ACCEL * dt
            ang += max(-max_change, min(max_change, target_ang - ang))
            if done or now > turning['end_time']:
                turning, lin, ang, action = None, 0.0, 0.0, "STOP"
                settle_until = now + HOLD_TIME
                if odom['yaw'] is not None:              # lock this heading for a moment
                    hold = {'yaw': odom['yaw'], 'until': now + HOLD_TIME}
                curses.flushinp()                        # throw away repeated ← / → presses
        elif hold:
            # "active brake" for turning: if the robot starts to swing back, push gently against it
            err = angle_diff(hold['yaw'], odom['yaw']) if odom['yaw'] is not None else 0.0
            ang = 0.0 if abs(err) < HOLD_DEADBAND else max(-HOLD_MAX, min(HOLD_MAX, HOLD_GAIN * err))
            lin = 0.0
            if now > hold['until']:
                hold, ang = None, 0.0
        elif not keys and action not in ("STOP",) and now - last_key > RELEASE_TIMEOUT:
            lin, ang, action = 0.0, 0.0, "STOP"

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
            note = "Camera window closed. Press C to open it again."

        if REAL_ROBOT:
            cam_text = ("waiting for the robot..." if real_cam['on'] is None else
                        f"ON  (http://{PI_IP}:8000)" if real_cam['on'] else "OFF  (press C to turn on)")
        else:
            cam_text = "ON (separate window)" if camera is not None else "OFF"
        try:
            screen.erase()
            screen.addstr(0, 0, "JNNP RoboDojo - arrow teleop (v6) - " + ("REAL ROBOT" if REAL_ROBOT else "GAZEBO"))
            screen.addstr(2, 0, "  ^   forward (hold)       v   backward (hold)")
            screen.addstr(3, 0, "  <-  turn left one step   ->  turn right one step")
            screen.addstr(4, 0, "  SPACE stop   + faster   - slower   [ ] turn step")
            screen.addstr(5, 0, "  U dump up   N dump down   C camera on/off   q quit")
            screen.addstr(7, 0, f"  Speed: {speed:.2f} m/s   Turn speed: {turn_speed:.1f} rad/s   "
                                f"Turn step: {TURN_STEPS[step_i]} deg")
            screen.addstr(8, 0, f"  Now:    {action}")
            screen.addstr(9, 0, f"  Dump:   {dump_state}")
            screen.addstr(10, 0, f"  Camera: {cam_text}")
            screen.addstr(11, 0, "  /odom:  " + ("OK (exact turns)" if odom['yaw'] is not None
                                                 else "not received - turns are timed"))
            if note:
                screen.addstr(13, 0, "  " + note)
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
    curses.wrapper(main)
