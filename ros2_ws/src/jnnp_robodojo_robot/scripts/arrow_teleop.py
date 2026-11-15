#!/usr/bin/env python3
"""Arrow-key teleop for the JNNP RoboDojo robot.

  Right arrow : forward          Space : stop
  Left arrow  : backward         + / - : faster / slower
  Up arrow    : turn left        q     : quit
  Down arrow  : turn right
  d           : dump back cabin (press again to lower it)

Hold an arrow key to keep moving; let go and the robot stops.
Publishes geometry_msgs/Twist on /cmd_vel.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.node import Node

DUMP_ANGLE = 0.70      # rad the back cabin tips up when dumping (~40 deg)
HOLD_TIMEOUT = 0.6     # s without a key press before stopping (covers the keyboard repeat delay)
RATE = 20.0            # Hz

HELP = """
---------------------------------------------
  JNNP RoboDojo arrow-key teleop
---------------------------------------------
  [Right]   forward
  [Left]    backward
  [Up]      turn left
  [Down]    turn right
  d         dump / lower the back cabin

  Space : stop      + / - : faster / slower
  q     : quit
  Hold a key to keep moving; release to stop.
---------------------------------------------
"""


_pending = b''


def read_key(timeout):
    """Return 'up', 'down', 'left', 'right', or the typed character, or None.

    Reads raw bytes with os.read so arrow-key escape codes (ESC [ A..D) arrive whole.
    """
    global _pending
    fd = sys.stdin.fileno()
    if not _pending:
        ready, _, _ = select.select([fd], [], [], timeout)
        if not ready:
            return None
        _pending = os.read(fd, 64)
    if _pending[:1] == b'\x1b':
        while len(_pending) < 3:                       # rest of the escape code still arriving
            r, _, _ = select.select([fd], [], [], 0.03)
            if not r:
                break
            _pending += os.read(fd, 64)
        seq, _pending = _pending[1:3], _pending[3:]
        return {b'[A': 'up', b'[B': 'down', b'[C': 'right', b'[D': 'left',
                b'OA': 'up', b'OB': 'down', b'OC': 'right', b'OD': 'left'}.get(seq)
    ch, _pending = _pending[:1], _pending[1:]
    return ch.decode(errors='ignore')


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.declare_parameter('speed', 0.2)    # m/s
        self.declare_parameter('turn', 0.3)     # rad/s (slow turns keep the SLAM map clean)
        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dump_pub = self.create_publisher(Float64, 'dump_cmd', 10)
        self.dumping = False

    def toggle_dump(self):
        self.dumping = not self.dumping
        self.dump_pub.publish(Float64(data=DUMP_ANGLE if self.dumping else 0.0))

    def send(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArrowTeleop()
    settings = termios.tcgetattr(sys.stdin)
    lin = ang = 0.0
    moving = False
    last_key = 0.0
    try:
      try:
        tty.setcbreak(sys.stdin.fileno())
        print(HELP)
        while rclpy.ok():
            key = read_key(1.0 / RATE)
            now = time.monotonic()
            if key in ('up', 'down', 'left', 'right'):
                # key layout chosen by the user: Right = forward, Left = backward,
                # Up = turn left, Down = turn right
                lin = {'right': node.speed, 'left': -node.speed}.get(key, 0.0)
                ang = {'up': node.turn, 'down': -node.turn}.get(key, 0.0)
                last_key = now
            elif key == ' ':
                lin = ang = 0.0
                last_key = 0.0
            elif key in ('+', '='):
                node.speed = min(node.speed * 1.1, 1.0)
                node.turn = min(node.turn * 1.1, 3.0)
            elif key in ('-', '_'):
                node.speed = max(node.speed / 1.1, 0.02)
                node.turn = max(node.turn / 1.1, 0.05)
            elif key in ('d', 'D'):
                node.toggle_dump()
            elif key in ('q', 'Q', '\x03'):
                break

            if now - last_key > HOLD_TIMEOUT:
                lin = ang = 0.0

            if lin or ang:
                node.send(lin, ang)
                moving = True
            elif moving:                 # send one stop, then stay quiet
                node.send(0.0, 0.0)
                moving = False

            state = 'FORWARD ' if lin > 0 else 'BACKWARD' if lin < 0 else \
                    'LEFT    ' if ang > 0 else 'RIGHT   ' if ang < 0 else 'STOPPED '
            cabin = 'DUMPING' if node.dumping else 'down   '
            sys.stdout.write(f'\r  {state}   speed {node.speed:.2f} m/s   turn {node.turn:.2f} rad/s   cabin {cabin}  ')
            sys.stdout.flush()
            rclpy.spin_once(node, timeout_sec=0.0)
      except KeyboardInterrupt:      # Ctrl+C: quit quietly, no traceback
        pass
    finally:
        node.send(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\nStopped.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
