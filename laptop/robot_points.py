#!/usr/bin/env python3
"""
JNNP RoboDojo - MARK THE COMPETITION PLACES ON A MAP (once per map, on a practice day).

    robotpoints NAME      (real robot, after  robotrun NAME  says READY)
    gzpoints              (Gazebo, after  gzrun fast  says READY)

In RViz use the  "Publish Point"  button (top bar) and click on the map where this program asks.
For each place you click TWO points:
    1) where the robot should STOP (its centre)
    2) a point in the direction the robot should FACE there (e.g. the screen itself)
For the ROUTE (grass, corridors, ramps ... up to the gate) you click ONE point per via-point, in
driving order, and press Enter in this window when the route is complete.

Saves  ~/maps/NAME_points.yaml  (used by  robotcompete NAME / gzcompete).
Ctrl+C = stop without saving.
"""
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

PLACES = [   # (key, what to tell the user)
    ('screen',          'the SCREEN (yellow): where the robot stops in front of it'),
    ('load_plastic',    'the PLASTIC loading station (blue, left)'),
    ('load_organic',    'the ORGANIC loading station (green, left)'),
    ('load_metal',      'the METAL loading station (red, left)'),
    ('gate_check',      'the spot just BEFORE THE RAMP GATE, facing through the gate'),
    ('after_gate',      'the spot just AFTER the gate (on the gated path)'),
    ('offload_plastic', 'the PLASTIC offloading station (blue, middle)'),
    ('offload_organic', 'the ORGANIC offloading station (green, bottom)'),
    ('offload_metal',   'the METAL offloading station (red, bottom)'),
]


class Clicks(Node):
    def __init__(self, sim):
        super().__init__('robodojo_points')
        if sim:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pts = []
        self.lock = threading.Lock()
        self.create_subscription(PointStamped, 'clicked_point', self.cb, 10)

    def cb(self, m):
        with self.lock:
            self.pts.append((round(m.point.x, 3), round(m.point.y, 3)))
        print(f'      clicked  x={m.point.x:.2f}  y={m.point.y:.2f}', flush=True)

    def next_click(self):
        with self.lock:
            n = len(self.pts)
        while True:
            with self.lock:
                if len(self.pts) > n:
                    return self.pts[n]
            time.sleep(0.05)


def ask_place(node, key, text):
    print(f'\n>>> {text}')
    print('    click 1: where the robot should STOP')
    x, y = node.next_click()
    print('    click 2: a point in the direction the robot should FACE')
    fx, fy = node.next_click()
    yaw = math.atan2(fy - y, fx - x)
    print(f'    saved {key}: x={x:.2f} y={y:.2f} facing {math.degrees(yaw):.0f} deg')
    return {'x': x, 'y': y, 'yaw': round(yaw, 3)}


def ask_route(node, title, note):
    print(f'\n>>> {title}')
    print(f'    {note}')
    print('    Click one point per via-point IN DRIVING ORDER. Press Enter here when done '
          '(Enter straight away = no via-points).')
    pts, done = [], threading.Event()
    threading.Thread(target=lambda: (sys.stdin.readline(), done.set()), daemon=True).start()
    with node.lock:
        start = len(node.pts)
    while not done.is_set():
        time.sleep(0.1)
    with node.lock:
        new = node.pts[start:]
    for i, (x, y) in enumerate(new):
        nx, ny = new[i + 1] if i + 1 < len(new) else (x + 1.0, y)
        pts.append({'x': x, 'y': y, 'yaw': round(math.atan2(ny - y, nx - x), 3)})
    print(f'    saved {len(pts)} via-point(s)')
    return pts


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    sim = '--sim' in sys.argv
    name = args[0] if args else 'robodojo_field'
    out = os.path.expanduser(f'~/maps/{name}_points.yaml')
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rclpy.init()
    node = Clicks(sim)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f'MARKING competition places for map "{name}".')
    print('In RViz: click the  "Publish Point"  button, then click on the map. '
          '(Press the button again before every click.)')
    data = {'map': name}
    try:
        for key, text in PLACES[:4]:
            data[key] = ask_place(node, key, text)
        data['route_to_gate'] = ask_route(
            node, 'ROUTE from the loading stations to the gate',
            'GRASS (top) -> OBSTACLES zone (top right) -> RAMP (right side) -> towards the ramp gate.\n'
            '    One click in each zone is enough - Nav2 finds the way and avoids the obstacles itself.')
        data['gate_check'] = ask_place(node, 'gate_check', PLACES[4][1])
        data['after_gate'] = ask_place(node, 'after_gate', PLACES[5][1])
        data['detour'] = ask_route(
            node, 'DETOUR via the ramp (used only when the gate is CLOSED)',
            'from the gate spot, around via the ramp, to where it meets the gated path again.')
        for key, text in PLACES[6:]:
            data[key] = ask_place(node, key, text)
        # the offloading station is also the finish (judges' rule) - no separate finish point
    except KeyboardInterrupt:
        print('\nStopped - nothing saved.')
        os._exit(1)
    with open(out, 'w') as f:
        f.write('# Written by robotpoints / gzpoints. Map frame, metres, yaw in radians.\n')
        yaml.safe_dump(data, f, sort_keys=False)
    print(f'\nSAVED: {out}\nNow the run:  robotcompete {name}   (Gazebo: gzcompete)')
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
