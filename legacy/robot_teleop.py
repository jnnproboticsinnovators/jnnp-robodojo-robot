"""
RoboDojo all-in-one teleop - runs on the Raspberry Pi (start it over SSH).

  * KEYBOARD  -> Arduino Mega (/dev/ttyACM0): drive the robot with W A S D
  * LiDAR     (/dev/ttyUSB0): automatic stop if something is too close in FRONT
  * CAMERA    (ribbon): live video at  http://<pi-address>:8000  in the laptop browser

Keys: W or RIGHT-arrow forward   S or LEFT-arrow back
      A or UP-arrow turn left      D or DOWN-arrow turn right   (smooth, like a car)
      SPACE stop
      + faster  - slower  U deck up  N deck down  R reset encoders
      O = LiDAR safety stop ON/OFF   Q quit
"""
import curses
import glob
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import serial
from picamera2 import Picamera2
from rplidar import RPLidar

# ---------------- settings ----------------
ARDUINO_PORT = (sorted(glob.glob('/dev/ttyACM*')) or ['/dev/ttyACM0'])[0]   # first Arduino found
LIDAR_PORT = '/dev/ttyUSB0'
FRONT_ANGLE = 0        # LiDAR angle that points to the robot's FRONT (check with lidar_view.py)
FRONT_WIDTH = 20       # look +/- this many degrees around the front
STOP_DISTANCE = 0.03   # metres: refuse to drive forward if something is closer than this
IGNORE_CLOSER = 0.0    # metres: ignore LiDAR hits closer than this (parts of the robot itself),
                       # e.g. 0.22 if the LiDAR keeps seeing the robot's own frame at ~0.19 m
CAMERA_PORT = 8000

# ---------------- shared state ----------------
running = True
front_dist = None
lidar_status = 'starting...'
camera_status = 'starting...'
latest_jpeg = None
jpeg_lock = threading.Lock()


# ---------------- LiDAR thread ----------------
def lidar_loop():
    global front_dist, lidar_status
    while running:
        lidar = None
        try:
            lidar = RPLidar(LIDAR_PORT, baudrate=115200)
            for scan in lidar.iter_scans(max_buf_meas=500):
                if not running:
                    break
                ahead = [d for (_, a, d) in scan
                         if d / 1000 > IGNORE_CLOSER and abs((a - FRONT_ANGLE + 180) % 360 - 180) <= FRONT_WIDTH]
                front_dist = min(ahead) / 1000 if ahead else None
                lidar_status = 'OK'
        except Exception as e:
            lidar_status = f'error ({e}) - retrying'
            front_dist = None
            time.sleep(1)
        finally:
            if lidar is not None:
                try:
                    lidar.stop(); lidar.stop_motor(); lidar.disconnect()
                except Exception:
                    pass


# ---------------- camera thread ----------------
def camera_loop():
    global latest_jpeg, camera_status
    try:
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"}))
        cam.start()
        camera_status = 'OK'
        while running:
            frame = cam.capture_array()
            label = f"front: {front_dist:.2f} m" if front_dist is not None else "front: --"
            cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with jpeg_lock:
                    latest_jpeg = jpg.tobytes()
            time.sleep(0.08)          # about 10 pictures per second
        cam.stop()
    except Exception as e:
        camera_status = f'error ({e})'


class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):   # keep the terminal clean
        pass

    def do_GET(self):
        if self.path == '/':
            page = (b"<html><body style='background:#111;color:#eee;font-family:sans-serif'>"
                    b"<h2>RoboDojo camera</h2><img src='/stream' style='max-width:100%'></body></html>")
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while running:
                    with jpeg_lock:
                        jpg = latest_jpeg
                    if jpg:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)


def pi_address():
    try:
        return subprocess.check_output(['hostname', '-I'], text=True).split()[0]
    except Exception:
        return socket.gethostname() + '.local'


# ---------------- keyboard + Arduino (main thread) ----------------
NAMES = {'w': 'FORWARD', 's': 'BACKWARD', 'a': 'TURN LEFT', 'd': 'TURN RIGHT', 'x': 'STOPPED'}


def main(screen, ser, url):
    screen.nodelay(True)
    current, deck = 'x', 'DOWN'
    last_send = 0
    buf = ''
    enc_l = enc_r = speed = '?'
    safety_on = True
    arrows = {curses.KEY_RIGHT: 'w', curses.KEY_LEFT: 's',
              curses.KEY_UP: 'a', curses.KEY_DOWN: 'd'}

    while True:
        key = screen.getch()
        if key in arrows:
            current = arrows[key]
        elif key in (ord('o'), ord('O')):
            safety_on = not safety_on
        elif key != -1 and key < 256:
            ch = chr(key).lower()
            if ch == 'q':
                break
            if ch == ' ':
                ch = 'x'
            if ch in 'wasdx':
                current = ch
            elif ch in '+-r':
                ser.write(ch.encode())
            elif ch == 'u':
                ser.write(b'u'); deck = 'UP (dumping)'
            elif ch == 'n':
                ser.write(b'n'); deck = 'DOWN'

        # LiDAR safety: block FORWARD if something is too close in front
        blocked = (safety_on and current in 'wad'          # all forward-moving commands
                   and front_dist is not None and front_dist < STOP_DISTANCE)
        command = 'x' if blocked else current

        if time.time() - last_send > 0.1:          # keep the Arduino's watchdog fed
            ser.write(command.encode())
            last_send = time.time()

        buf += ser.read(ser.in_waiting or 1).decode(errors='ignore')
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            p = line.split()
            if len(p) == 5 and p[0] == 'ENC':
                enc_l, enc_r, speed = p[1], p[2], p[4]

        fd = f"{front_dist:.2f} m" if front_dist is not None else "--"
        screen.erase()
        screen.addstr(0, 0, "ROBODOJO TELEOP  (Arduino + LiDAR + Camera)")
        screen.addstr(2, 0, "RIGHT forward  LEFT back  UP turn left  DOWN turn right  SPACE stop")
        screen.addstr(3, 0, "+/- speed  U/N deck  R reset  O safety  Q quit")
        screen.addstr(5, 0, f"Motion  : {NAMES[current]}" + ("   << BLOCKED: obstacle ahead!" if blocked else ""))
        screen.addstr(6, 0, f"Speed   : {speed}/255      Deck: {deck}")
        screen.addstr(7, 0, f"Encoders: left {enc_l}   right {enc_r}")
        screen.addstr(8, 0, f"LiDAR   : {lidar_status}   front distance {fd}  "
                      + (f"(stop below {STOP_DISTANCE} m)" if safety_on else "SAFETY OFF - be careful!"))
        screen.addstr(9, 0, f"Camera  : {camera_status}   view at {url}")
        screen.refresh()
        time.sleep(0.02)


if __name__ == '__main__':
    print(f"Connecting to Arduino on {ARDUINO_PORT}...")
    ser = serial.Serial(ARDUINO_PORT, 115200, timeout=0)
    time.sleep(2)                       # the Mega restarts when the port opens

    threading.Thread(target=lidar_loop, daemon=True).start()
    threading.Thread(target=camera_loop, daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', CAMERA_PORT), StreamHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
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
        server.shutdown()
        time.sleep(1.5)                 # let the LiDAR and camera stop cleanly
        print("Stopped. Bye!")
