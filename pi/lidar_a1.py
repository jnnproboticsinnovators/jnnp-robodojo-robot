#!/usr/bin/env python3
"""
JNNP RoboDojo - simple, robust RPLIDAR A1 reader (no extra library needed, only pyserial).

Test on the Pi:
    python3 lidar_a1.py
It prints, once per turn of the LiDAR:  points, nearest object (distance + angle), front distance.
Hold a box ~30 cm in FRONT of the robot: the angle it shows is your FRONT_ANGLE.
Ctrl+C to stop.
"""
import time
import serial

PORT = '/dev/ttyUSB0'
BAUD = 115200


class A1Lidar:
    def __init__(self, port=PORT, baud=BAUD):
        self.port, self.baud = port, baud
        self.ser = None

    def start(self):
        """Open the port, spin the motor, (re)start scanning. Returns True when data flows."""
        self.close()
        self.ser = serial.Serial(self.port, self.baud, timeout=0.3)
        self.ser.setDTR(False)                 # A1 adapter: DTR low = motor ON
        time.sleep(3.0)                        # the A1 sends NO data until its motor is at full speed
                                               # (opening the port stops the motor briefly; 1 s was too short)
        for _ in range(3):
            self.ser.write(b'\xA5\x25')        # STOP any old scan
            time.sleep(0.05)
            self.ser.reset_input_buffer()
            self.ser.write(b'\xA5\x20')        # SCAN
            head = self._read_exact(7)
            if head[:2] == b'\xA5\x5A' and head[2] == 5 and head[6] == 0x81:
                return True
            time.sleep(0.3)
        return False

    def _read_exact(self, n):
        data = b''
        t_end = time.time() + 2
        while len(data) < n and time.time() < t_end:
            data += self.ser.read(n - len(data))
        return data

    def _restart_scan(self):
        """Motor is already running: just send STOP + SCAN again and check the reply header."""
        self.ser.write(b'\xA5\x25')
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.write(b'\xA5\x20')
        head = self._read_exact(7)
        return head[:2] == b'\xA5\x5A' and head[2] == 5 and head[6] == 0x81

    def scans(self, stall_time=1.5, give_up=20):
        """Yield one full turn at a time: list of (angle_deg, distance_m).
        If the data pauses for stall_time seconds, the scan is restarted automatically.
        Only after give_up seconds with no data at all does it stop with an error."""
        turn, buf = [], b''
        last_data = time.time()
        self.restarts = 0
        while True:
            chunk = self.ser.read(max(5, self.ser.in_waiting))
            if not chunk:
                quiet = time.time() - last_data
                if quiet > give_up:
                    raise IOError(f'LiDAR sent no data for {give_up} s')
                if quiet > stall_time:
                    self.restarts += 1
                    self._restart_scan()
                    buf, turn = b'', []
                continue
            last_data = time.time()
            buf += chunk
            i = 0
            while len(buf) - i >= 5:
                b0, b1, b2, b3, b4 = buf[i:i + 5]
                s, not_s, check = b0 & 1, (b0 >> 1) & 1, b1 & 1
                if s == not_s or check != 1:   # not a valid packet start -> resync by 1 byte
                    i += 1
                    continue
                i += 5
                if s and turn:                 # start of a new turn
                    yield turn
                    turn = []
                quality = b0 >> 2
                angle = ((b2 << 7) | (b1 >> 1)) / 64.0
                dist = ((b4 << 8) | b3) / 4000.0   # metres
                if quality > 0 and dist > 0:
                    turn.append((angle % 360, dist))
            buf = buf[i:]

    def close(self):
        if self.ser is not None:
            try:
                self.ser.write(b'\xA5\x25')    # STOP scan
                time.sleep(0.05)
                self.ser.setDTR(True)          # motor OFF
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def front_distance(turn, front_angle=0, width=20, ignore_closer=0.0):
    near = [d for a, d in turn
            if d > ignore_closer and abs((a - front_angle + 180) % 360 - 180) <= width]
    return min(near) if near else None


if __name__ == '__main__':
    lidar = A1Lidar()
    print('Starting LiDAR...')
    if not lidar.start():
        print('LiDAR did not start scanning - check the 7-pin cable is pushed in firmly.')
        raise SystemExit(1)
    print('Scanning. Ctrl+C to stop.')
    try:
        for n, turn in enumerate(lidar.scans(), 1):
            if not turn:
                continue
            a, d = min(turn, key=lambda p: p[1])
            fd = front_distance(turn)
            print(f'turn {n:4d}: {len(turn):3d} points   nearest {d:5.2f} m at {a:5.1f} deg   '
                  f'front(0 deg) {fd if fd is None else round(fd, 2)} m   (auto-restarts: {lidar.restarts})')
    except KeyboardInterrupt:
        pass
    except IOError as e:
        print(e, '- check the LiDAR cable and power.')
    finally:
        lidar.close()
        print('Stopped.')
