"""
RoboDojo keyboard control - run on the Raspberry Pi (over SSH).
W/A/S/D drive, SPACE stop, +/- speed, U/N deck up/down, R reset counts, Q quit.
"""
import curses
import time
import serial

PORT = '/dev/ttyACM0'    # the Arduino Mega on the Pi
ser = serial.Serial(PORT, 115200, timeout=0)
time.sleep(2)            # the Mega restarts when the port opens

NAMES = {'w': 'FORWARD', 's': 'BACKWARD', 'a': 'SPIN LEFT', 'd': 'SPIN RIGHT', 'x': 'STOPPED'}


def main(screen):
    screen.nodelay(True)
    current, deck = 'x', 'DOWN'
    last_send = 0
    buf = ''
    enc_l = enc_r = speed = '?'

    while True:
        key = screen.getch()
        if key != -1 and key < 256:
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

        # keep sending the motion command; the Mega stops by itself if this stops
        if time.time() - last_send > 0.1:
            ser.write(current.encode())
            last_send = time.time()

        # read reports from the Mega
        buf += ser.read(ser.in_waiting or 1).decode(errors='ignore')
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            p = line.split()
            if len(p) == 5 and p[0] == 'ENC':
                enc_l, enc_r, speed = p[1], p[2], p[4]

        screen.erase()
        screen.addstr(0, 0, "ROBODOJO KEYBOARD CONTROL")
        screen.addstr(2, 0, "W forward   S backward   A spin left   D spin right   SPACE stop")
        screen.addstr(3, 0, "+ faster    - slower     U deck up     N deck down    R reset   Q quit")
        screen.addstr(5, 0, f"Motion : {NAMES[current]}")
        screen.addstr(6, 0, f"Speed  : {speed}/255")
        screen.addstr(7, 0, f"Deck   : {deck}")
        screen.addstr(8, 0, f"Encoder: left {enc_l}   right {enc_r}")
        screen.refresh()
        time.sleep(0.02)


try:
    curses.wrapper(main)
finally:
    ser.write(b'x')      # always stop the motors on exit
    ser.close()
