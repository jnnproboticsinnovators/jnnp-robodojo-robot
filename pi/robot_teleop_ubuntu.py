"""
RoboDojo teleop for the Pi on UBUNTU 24.04 - runs on the Raspberry Pi (start it over SSH).

  * KEYBOARD  -> Arduino Mega (/dev/ttyACM*): drive the robot with the ARROW keys ONLY
  * LiDAR     (/dev/ttyUSB0): automatic stop if something is too close in FRONT (needs lidar_a1.py)
  * CAMERA    is NOT used here: the waste-detect service already shows the camera
              (with AI boxes) at  http://<pi-address>:8000

Keys: UP arrow forward         DOWN arrow back
      LEFT arrow turn left       RIGHT arrow turn right
      SPACE stop   + faster  - slower   U servo bin up (dump)  N servo bin down
      R reset encoders   O LiDAR safety ON/OFF   C camera AI ON/OFF   A autonomous ON/OFF   Q quit
      (the Pi turns each arrow into a code letter for the Arduino - you never type letters)
"""
import curses
import glob
import json
import socket
import subprocess
import threading
import time
import urllib.request

import serial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
try:
    from lidar_a1 import A1Lidar, front_distance   # our own RPLIDAR A1 reader (lidar_a1.py, same folder)
except ImportError:
    A1Lidar = None

# ---------------- settings ----------------
ARDUINO_PORT = (sorted(glob.glob('/dev/ttyACM*')) or ['/dev/ttyACM0'])[0]   # first Arduino found
LIDAR_PORT = '/dev/ttyUSB0'
FRONT_ANGLE = 0        # LiDAR angle that points to the robot's FRONT (check: put a box in front)
FRONT_WIDTH = 15       # look +/- this many degrees around the front
STOP_DISTANCE = 0.20   # metres: refuse to drive forward if something is closer than this
                       # (the A1 cannot measure closer than about 0.15 m)
IGNORE_CLOSER = 0.18   # metres: ignore hits closer than this - the LiDAR sees the robot's own
                       # parts at about 0.12 m
CAMERA_PORT = 8000
# ---- autonomous mode (key A) ----
AUTO_OBSTACLE = 0.40   # metres: obstacle ahead closer than this -> turn away
AUTO_RESUME = 0.60     # metres: keep turning until the front is clearer than this
WASTE_REACHED = 0.45   # waste box taller than 45% of the picture = close enough -> stop
LIDAR_CLOCKWISE = True # RPLIDAR A1 angles grow clockwise (seen from above): 90 = right, 270 = left
DETECTIONS_URL = 'http://127.0.0.1:8000/detections.json'   # from the camera AI (key C)
LIDAR_VIEW_PORT = 8001  # live LiDAR picture in the browser: http://<PI-IP>:8001

# ---------------- shared state ----------------
running = True
front_dist = None
lidar_status = 'starting...'
lidar_view = {'points': [], 'turn': 0, 'status': 'starting...'}   # for the browser view
last_turn, last_turn_time = [], 0.0   # newest full LiDAR turn, for autonomous mode
detections = None                     # newest camera detections, for autonomous mode


# ---------------- LiDAR thread ----------------
def lidar_loop():
    global front_dist, lidar_status, last_turn, last_turn_time
    if A1Lidar is None:
        lidar_status = 'lidar_a1.py missing (safety stop OFF)'
        return
    while running:
        lidar = A1Lidar(LIDAR_PORT)
        try:
            lidar_status = 'starting (3 s)...'
            if not lidar.start():
                raise IOError('no reply')
            for n, turn in enumerate(lidar.scans(), 1):
                if not running:
                    break
                front_dist = front_distance(turn, FRONT_ANGLE, FRONT_WIDTH, IGNORE_CLOSER)
                last_turn, last_turn_time = turn, time.time()
                lidar_status = 'OK'
                lidar_view.update(points=[[round(a, 1), round(d, 3)] for a, d in turn], turn=n,
                                  status=f'OK (auto-restarts {lidar.restarts})')
        except Exception as e:
            lidar_status = f'error ({e}) - retrying'
            lidar_view['status'] = lidar_status
            front_dist = None
            time.sleep(1)
        finally:
            lidar.close()


# ---------------- LiDAR view in the browser (same page as lidar_view.py) ----------------
try:
    from lidar_view import PAGE as LIDAR_PAGE
except Exception:
    LIDAR_PAGE = b"<html><body>Copy lidar_view.py next to the teleop to see the LiDAR picture.</body></html>"


class LidarViewHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/scan.json':
            body, ctype = json.dumps(lidar_view).encode(), 'application/json'
        elif self.path == '/':
            body, ctype = LIDAR_PAGE, 'text/html'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


def start_lidar_view():
    try:
        server = ThreadingHTTPServer(('0.0.0.0', LIDAR_VIEW_PORT), LidarViewHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
    except OSError:
        pass    # port busy (lidar_view.py already running?) - teleop still works


# ---------------- autonomous mode ----------------
def detections_loop():
    """Ask the camera AI for its newest detections 3 times a second (works only when the camera is ON)."""
    global detections
    while running:
        try:
            with urllib.request.urlopen(DETECTIONS_URL, timeout=0.5) as r:
                detections = json.loads(r.read())
        except Exception:
            detections = None
        time.sleep(0.3)


def rel_angle(a):
    """LiDAR angle -> robot angle: 0 = front, 90 = right, 180 = back, 270 = left."""
    r = (a - FRONT_ANGLE) % 360
    return r if LIDAR_CLOCKWISE else (-r) % 360


def space_between(turn, lo, hi):
    """Typical free distance in a sector (median, so one odd point does not fool it)."""
    d = sorted(dist for a, dist in turn if dist > IGNORE_CLOSER and lo <= rel_angle(a) < hi)
    return d[len(d) // 2] if d else 8.0


class AutoPilot:
    """Reactive autonomy: explore, avoid obstacles with the LiDAR, drive to waste the camera sees."""

    def __init__(self):
        self.avoid = None          # 'a' or 'd' while turning away from an obstacle
        self.pulse_cmd, self.pulse_end, self.wait_end = None, 0.0, 0.0

    def decide(self, now):
        """Returns (command letter, reason shown on screen)."""
        if now - last_turn_time > 1.0:
            return 'x', 'waiting for LiDAR'
        fd = front_dist
        # 1. obstacle ahead -> spin towards the side with more space, until the front is clear
        if self.avoid:
            if fd is None or fd > AUTO_RESUME:
                self.avoid = None
            else:
                return self.avoid, f'avoiding obstacle ({fd:.2f} m)'
        if fd is not None and fd < AUTO_OBSTACLE:
            left = space_between(last_turn, 270, 340)
            right = space_between(last_turn, 20, 90)
            self.avoid = 'a' if left > right else 'd'
            return self.avoid, f'obstacle {fd:.2f} m - turning {"left" if self.avoid == "a" else "right"}'
        # 2. waste seen by the camera -> aim and approach
        if now < self.pulse_end:
            return self.pulse_cmd, 'aiming at waste'
        det = detections
        if det and det.get('items') and now - det.get('time', 0) < 2.0:
            if det['time'] < self.pulse_end:              # picture taken before our last turn: wait for a new one
                return ('x', 'looking at waste...') if now < self.wait_end + 1.5 else ('w', 'exploring')
            best = max(det['items'], key=lambda i: i['height'])
            name = best['name']
            if best['height'] >= WASTE_REACHED:
                return 'x', f'REACHED {name} - press U to dump / arrows to drive'
            if best['side'] in ('LEFT', 'RIGHT'):
                self.pulse_cmd = 'a' if best['side'] == 'LEFT' else 'd'
                self.pulse_end = now + 0.15                # short turn, then look again
                self.wait_end = self.pulse_end + 0.6
                return self.pulse_cmd, f'{name} on the {best["side"].lower()} - turning'
            return 'w', f'driving to {name} ({best["conf"]:.0%})'
        # 3. nothing special -> explore forward
        return 'w', 'exploring'


CAMERA_SERVICE = 'waste-detect'


def camera_is_on():
    try:
        return subprocess.run(['systemctl', 'is-active', '--quiet', CAMERA_SERVICE]).returncode == 0
    except Exception:
        return False


def camera_switch(on):
    """Start/stop the camera AI. Needs the one-time sudo rule (see instructions)."""
    subprocess.Popen(['sudo', '-n', 'systemctl', 'start' if on else 'stop', CAMERA_SERVICE],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def pi_address():
    try:
        return subprocess.check_output(['hostname', '-I'], text=True).split()[0]
    except Exception:
        return socket.gethostname() + '.local'


# ---------------- keyboard + Arduino (main thread) ----------------
NAMES = {'w': 'FORWARD', 's': 'BACKWARD', 'a': 'TURN LEFT', 'd': 'TURN RIGHT', 'x': 'STOPPED'}


def main(screen, ser, url):
    screen.nodelay(True)
    current, deck = 'x', 'DOWN (0 deg)'
    last_send = 0
    last_command = ''
    buf = ''
    enc_l = enc_r = speed = '?'
    safety_on = True
    cam_on = camera_is_on()
    cam_msg = ''
    cam_target = None          # what we asked for (True/False) while waiting
    cam_asked = 0
    last_cam_check = time.time()
    arrows = {curses.KEY_UP: 'w', curses.KEY_DOWN: 's',
              curses.KEY_LEFT: 'a', curses.KEY_RIGHT: 'd'}
    auto_on, pilot, auto_reason = False, AutoPilot(), ''

    while True:
        # Read ALL waiting keys; SPACE beats everything so stop is instant
        keys = []
        while True:
            k = screen.getch()
            if k == -1:
                break
            keys.append(k)
        if ord(' ') in keys:
            keys = [ord(' ')]
        quit_now = False
        for key in keys[-1:]:
            if key in arrows:
                current = arrows[key]
                auto_on = False                     # any arrow = take back control
            elif key in (ord('o'), ord('O')):
                safety_on = not safety_on
            elif key < 256:
                ch = chr(key).lower()
                if ch == 'q':
                    quit_now = True
                elif ch == ' ':
                    current = 'x'
                    auto_on = False                 # SPACE = stop AND leave autonomous mode
                elif ch == 'a':
                    auto_on = not auto_on
                    pilot = AutoPilot()
                    current = 'x'
                elif ch in '+-r':
                    ser.write(ch.encode())
                elif ch == 'u':
                    ser.write(b'u'); deck = 'UP (40 deg, dumping)'
                elif ch == 'n':
                    ser.write(b'n'); deck = 'DOWN (0 deg)'
                elif ch == 'c':
                    cam_target = not cam_on
                    camera_switch(cam_target)
                    cam_asked = time.time()
                    cam_msg = 'turning ON (takes ~5 s)...' if cam_target else 'turning OFF...'
        if quit_now:
            break

        if auto_on:
            if lidar_status != 'OK':
                current, auto_reason = 'x', 'LiDAR not ready - autonomous paused'
            else:
                current, auto_reason = pilot.decide(time.time())

        # LiDAR safety: block FORWARD if something is too close in front
        blocked = (safety_on and current == 'w'            # only driving forward is refused; turning away stays allowed
                   and front_dist is not None and front_dist < STOP_DISTANCE)
        command = 'x' if blocked else current

        if command != last_command or time.time() - last_send > 0.1:   # change -> send NOW
            ser.write(command.encode())
            last_send = time.time()
            last_command = command

        buf += ser.read(ser.in_waiting or 1).decode(errors='ignore')
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            p = line.split()
            if len(p) == 5 and p[0] == 'ENC':
                enc_l, enc_r, speed = p[1], p[2], p[4]

        fd = f"{front_dist:.2f} m" if front_dist is not None else "--"
        screen.erase()
        screen.addstr(0, 0, "ROBODOJO TELEOP  (Arduino + LiDAR)")
        screen.addstr(2, 0, "UP forward  DOWN back  LEFT turn left  RIGHT turn right  SPACE stop")
        screen.addstr(3, 0, "+/- speed  U servo UP  N servo DOWN  R reset  O safety  C camera  A autonomous  Q quit")
        screen.addstr(5, 0, f"Motion  : {NAMES[current]}" + ("   << BLOCKED: obstacle ahead!" if blocked else ""))
        screen.addstr(6, 0, f"Speed   : {speed}/255")
        screen.addstr(10, 0, f"Servo   : bin {deck}   (U = up / dump,  N = down)")
        screen.addstr(7, 0, f"Encoders: left {enc_l}   right {enc_r}")
        screen.addstr(8, 0, f"LiDAR   : {lidar_status}   front distance {fd}  "
                      + (f"(stop below {STOP_DISTANCE} m)" if safety_on else "SAFETY OFF - be careful!"))
        if time.time() - last_cam_check > 1:           # check the camera every second
            cam_on = camera_is_on()
            last_cam_check = time.time()
            if cam_target is not None:
                if cam_on == cam_target:
                    cam_target, cam_msg = None, ''
                elif time.time() - cam_asked > 15:
                    cam_target, cam_msg = None, 'FAILED - do the one-time sudo setup'
        cam_text = (f"ON  - video at {url}" if cam_on else "OFF (press C to turn on)")
        screen.addstr(9, 0, f"Camera  : {cam_text}  {cam_msg}")
        screen.addstr(11, 0, f"LiDAR view in browser: {url.rsplit(':', 1)[0]}:{LIDAR_VIEW_PORT}")
        screen.addstr(13, 0, (f"MODE    : AUTONOMOUS - {auto_reason}   (arrows or SPACE = take control)"
                              if auto_on else "MODE    : MANUAL   (press A for autonomous)"),
                      curses.A_BOLD if auto_on else 0)
        screen.refresh()
        time.sleep(0.02)


if __name__ == '__main__':
    print(f"Connecting to Arduino on {ARDUINO_PORT}...")
    ser = serial.Serial(ARDUINO_PORT, 115200, timeout=0)
    time.sleep(2)                       # the Mega restarts when the port opens

    threading.Thread(target=lidar_loop, daemon=True).start()
    start_lidar_view()
    threading.Thread(target=detections_loop, daemon=True).start()
    url = f"http://{pi_address()}:{CAMERA_PORT}"

    try:
        curses.wrapper(main, ser, url)
    finally:
        running = False
        try:
            ser.write(b'x')             # always stop the motors
            ser.close()
        except Exception:
            print("Arduino was disconnected - check its USB cable.")
        time.sleep(1.5)                 # let the LiDAR stop cleanly
        print("Stopped. Bye!")
