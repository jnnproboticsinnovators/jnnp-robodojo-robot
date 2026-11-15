#!/usr/bin/env python3
"""
JNNP RoboDojo - AUTONOMOUS MISSION with Nav2 (runs on the Pi).

    python3 ~/robodojo_mission.py ~/maps/trial_6a_regions.yaml

Needs running: robotengine  +  robotnav <same map>
Put the robot at the START spot (same place and direction as when the map was made) first.

What it does, for every region 1, 2, 3 ... in the regions file:
    home -> load_N  (waits LOAD_WAIT s: the robot is being loaded)
         -> offload_N (stops, dump UP, waits, dump DOWN)
         -> home (turns back to the home direction)
Ctrl+C at any time = cancel the goal and stop the robot.
"""
import math
import re
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Float64

LOAD_WAIT = 5.0          # s waiting at each load point
DUMP_UP = math.radians(40)
DUMP_TIME = 3.0          # s the bin stays up at the offload point
GOAL_TIMEOUT = 120.0     # s - give up on a goal after this long


def pose(nav, p):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = nav.get_clock().now().to_msg()
    ps.pose.position.x = float(p['x'])
    ps.pose.position.y = float(p['y'])
    ps.pose.orientation.z = math.sin(float(p['yaw']) / 2)
    ps.pose.orientation.w = math.cos(float(p['yaw']) / 2)
    return ps


def first(v):
    return v[0] if isinstance(v, list) else v


def go(nav, name, p):
    print(f'-> going to {name}  (x {p["x"]:.2f}, y {p["y"]:.2f})')
    nav.goToPose(pose(nav, p))
    t0 = time.time()
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None:
            print(f'   {fb.distance_remaining:5.2f} m to go   ', end='\r')
        if time.time() - t0 > GOAL_TIMEOUT:
            nav.cancelTask()
            print(f'\n   {name}: took too long - cancelled')
            return False
        time.sleep(0.2)
    ok = nav.getResult() == TaskResult.SUCCEEDED
    print(f'\n   {name}: {"REACHED" if ok else "FAILED"}')
    return ok


def main():
    if len(sys.argv) < 2:
        sys.exit('Usage: python3 robodojo_mission.py ~/maps/<map>_regions.yaml')
    with open(sys.argv[1]) as f:
        reg = yaml.safe_load(f)
    start = first(reg.get('start') or reg['home'])
    home = first(reg.get('home') or reg['start'])
    numbers = sorted(int(m.group(1)) for k in reg if (m := re.fullmatch(r'load_(\d+)', k)))
    print(f'Regions found: {numbers}')

    rclpy.init()
    nav = BasicNavigator()
    dump = nav.create_publisher(Float64, 'dump_cmd', 10)
    stop = nav.create_publisher(Twist, 'cmd_vel', 10)

    print('Setting the start position on the map...')
    nav.setInitialPose(pose(nav, start))
    print('Waiting for Nav2 to be ready (robotnav must be running)...')
    nav.waitUntilNav2Active()
    time.sleep(2.0)

    try:
        for n in numbers:
            print(f'\n========== REGION {n} ==========')
            if not go(nav, f'load_{n}', first(reg[f'load_{n}'])):
                print('Skipping this region.')
                go(nav, 'home', home)
                continue
            print(f'   loading... ({LOAD_WAIT:.0f} s)')
            time.sleep(LOAD_WAIT)
            if go(nav, f'offload_{n}', first(reg[f'offload_{n}'])):
                print('   dumping: bin UP')
                dump.publish(Float64(data=DUMP_UP))
                time.sleep(DUMP_TIME)
                print('   bin DOWN')
                dump.publish(Float64(data=0.0))
                time.sleep(2.0)
            go(nav, 'home', home)
        print('\nMISSION COMPLETE.')
    except KeyboardInterrupt:
        print('\nStopped by you.')
        nav.cancelTask()
    finally:
        for _ in range(5):
            stop.publish(Twist())
            time.sleep(0.05)
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
