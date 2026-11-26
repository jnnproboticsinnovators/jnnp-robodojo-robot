#!/usr/bin/env python3
"""
JNNP RoboDojo - make the Nav2 settings file for the REAL robot (run once on the Pi).

    python3 ~/make_nav2_params.py

Reads the standard Nav2 settings and writes  ~/nav2_params_real.yaml  with our robot's values:
  * real time (no simulation clock)
  * robot shape: its real RECTANGLE (0.34 m long x 0.43 m wide, wheels included) instead of a
    big circle, and keep only 0.10 m extra away from walls - so it can move in tight spots
    like the start box (with a 0.26 m circle + 0.30 m it saw the whole start box as "blocked")
  * slow, safe speeds (0.20 m/s, 0.8 rad/s)
  * a simple, light path follower (Regulated Pure Pursuit) - the default one is too heavy for a Pi 4
  * LiDAR range 12 m (RPLIDAR A1)
  * AMCL trusts the LiDAR more than the wheels (skid-steer wheels slip when turning)
"""
import os
import sys

import yaml

SRC = '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
DST = os.path.expanduser('~/nav2_params_real.yaml')

ROBOT_RADIUS = 0.26      # only used if no footprint is given
HALF_LENGTH = 0.13       # m, centre to front/back  (outline 26 cm long)
HALF_WIDTH = 0.14        # m, centre to the outside of the wheels (outline 28 cm wide)
FOOTPRINT = (f'[[{HALF_LENGTH}, {HALF_WIDTH}], [{HALF_LENGTH}, -{HALF_WIDTH}], '
             f'[-{HALF_LENGTH}, -{HALF_WIDTH}], [-{HALF_LENGTH}, {HALF_WIDTH}]]')
INFLATION = 0.30      # >= the robot's inscribed radius (0.18) as Nav2 requires; costs fall
                      # away fast (COST_SCALING) so only the first ~10 cm really 'keep away'
SELF_IGNORE = 0.22      # m - LiDAR hits closer than this are the robot's OWN body (27 x 29 cm robot; the
                        #     dump bin, camera mast): never treat them as walls
COST_SCALING = 12.0
# The PATH is planned on the global costmap: there a WIDE but SOFT "keep-away" makes the planner
# prefer the middle of free areas (with a small one it hugged the walls - shortest way). It never
# blocks a gap: only the robot's own size blocks; this only makes cells near walls "more expensive".
GLOBAL_INFLATION = 0.55
GLOBAL_COST_SCALING = 1.8     # 24 Nov finals: cost stays high near walls -> paths run down the
                              # MIDDLE of corridors (narrow passages still allowed, just disliked)
# PLANNING size: the global planner treats the robot as a 0.15 m circle, so it can always find a
# way out of the narrow start box by itself (with the full 0.34 x 0.43 rectangle the box looked
# "occupied" and Nav2 aborted). The DRIVING part still checks the real rectangle against walls.
PLAN_RADIUS = 0.15
MAX_SPEED = 0.12         # m/s slow (24 Nov finals). Below ~0.10 the wheels stall.
MAX_TURN = 0.8           # rad/s
YAW_TOLERANCE = 0.4      # rad (23 deg) - 'facing the goal' is good enough within this
TF_TOLERANCE = 0.6       # s - accept position data this late (busy Pi) instead of aborting

FOLLOW_PATH = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': MAX_SPEED,
    # 24 Nov: 0.25 m was tried (less corner cutting) - from then on the robot spun/overshot on the
    # first leg in EVERY run (0 of 5 reached the loading station; with 0.4 m: 2 of 2 reached).
    # Back to 0.4 m. Wall ends are now handled by the wall guard in the engine instead.
    'lookahead_dist': 0.4,
    'min_lookahead_dist': 0.25,
    'max_lookahead_dist': 0.7,
    'lookahead_time': 1.5,
    'rotate_to_heading_angular_vel': 0.45,  # slow, steady turns on the spot (24 Nov finals)
    'transform_tolerance': 0.6,
    'use_velocity_scaled_lookahead_dist': False,
    # The real motors need a firm push: below ~0.12 m/s the robot hardly moves (stalls; on the
    # GRASS even more - 23 Nov it stuck at the grass edge), so
    # Nav2 must never ask for less than that while driving (it caused "Failed to make progress").
    'min_approach_linear_velocity': 0.12,
    'regulated_linear_scaling_min_speed': 0.12,
    'use_cost_regulated_linear_velocity_scaling': False,   # do not crawl just because walls are near
    'approach_velocity_scaling_dist': 0.5,
    # 21 Nov log: 933x "detected collision ahead!" - this check tests the robot outline against the
    # MAP's 5 cm squares; in the narrow corridors the outline 'touched' wall squares while the real
    # robot had room, so it stopped again and again (30 recoveries in 40 s). Off: the planner keeps
    # the path in the middle, the local costmap still avoids walls/objects, and on the REAL robot
    # the collision monitor (real LiDAR points, not map squares) still stops it before any hit.
    'use_collision_detection': False,
    'max_allowed_time_to_collision_up_to_carrot': 0.5,   # look 0.5 s ahead (1.0 refused moves near walls)
    'use_regulated_linear_velocity_scaling': True,
    'regulated_linear_scaling_min_radius': 0.6,
    'use_rotate_to_heading': True,
    'allow_reversing': False,
    'rotate_to_heading_min_angle': 0.785,   # turn on the spot only for > 45 deg (less spinning in tight spots)
    'max_angular_accel': 2.0,
    'max_robot_pose_search_dist': 10.0,
}


# AMCL (finding the robot on the map): trust the LiDAR more than the wheels,
# because a skid-steer robot's wheels slip when it turns.
AMCL_TUNING = {
    'alpha1': 0.1,   # turning error caused by turning. 24 Nov turn test: the gyro measures a
                     # 180 deg turn within 2% -> trust it (0.4 made the position 'unsure' grow
                     # from 0.07 to 1.0 m during slow turns in the loading area)
    'alpha2': 0.2,   # turning error caused by driving
    'alpha3': 0.5,   # driving error caused by driving. 24 Nov: on the GRASS the wheels slip, so the
                     # wheels over-count the distance (re-find found the robot 1.8 m further than it
                     # thought). 0.5 = do not trust wheel distance much; let the LiDAR decide.
    'alpha4': 0.2,   # driving error caused by turning
    'update_min_a': 0.1,   # re-check position after every 0.1 rad of turning (default 0.2)
    'update_min_d': 0.1,   # ... and every 10 cm
    'max_particles': 3000,
    'max_beams': 180,      # use 3x more LiDAR points (default 60): the corridor END walls are then
                           # seen, so the position ALONG a long corridor stays correct
    # 23 Nov run: the robot lost its place on the route and never found it again. With these,
    # AMCL keeps testing a few random positions when the LiDAR stops matching the map, so it
    # can find itself again (Nav2's defaults 0.0 switch this off).
    'recovery_alpha_slow': 0.0,   # off again: random test positions made the estimate jump;
    'recovery_alpha_fast': 0.0,   # robotcompete re-finds the robot from the LiDAR instead
    # Start Nav2 already knowing the robot is at the map's start spot (where SLAM began).
    # Without this, the map->odom link is missing until someone clicks 2D Pose Estimate,
    # and the navigation part of Nav2 times out and stays "inactive".
    'set_initial_pose': True,
    'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
}


# ---- FAST REACTIONS: Nav2's "behaviour tree" (what to do when blocked) ----
BT_SRC = '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
BT_DST = os.path.expanduser('~/robodojo_fast_bt.xml')


def make_fast_bt():
    """Copy Nav2's standard plan: re-plan 3x per second (was 1x) so a new way round an object is
    found at once; when stuck: back up 15 cm / turn 45 deg / wait 1 s (was 30 cm / 90 deg / 5 s);
    allow 10 tries (was 6) before giving up. Returns the new file, or None."""
    import re
    if not os.path.isfile(BT_SRC):
        return None
    x = open(BT_SRC).read()
    x = re.sub(r'(<RateController hz=")[0-9.]+(")', r'\g<1>3.0\g<2>', x)
    x = re.sub(r'(<Wait wait_duration=")[0-9.]+(")', r'\g<1>1.0\g<2>', x)
    x = re.sub(r'(<Spin spin_dist=")[0-9.]+(")', r'\g<1>0.79\g<2>', x)
    # 24 Nov finals: back up when blocked, but only 15 cm and SLOWLY (8 cm/s; Arduino v16 caps it too)
    x = re.sub(r'<BackUp [^>]*/>', '<BackUp backup_dist="0.15" backup_speed="0.08"/>', x)
    x = re.sub(r'(number_of_retries=")6(" name="NavigateRecovery")', r'\g<1>10\g<2>', x)
    open(BT_DST, 'w').write(x)
    return BT_DST


def walk(node):
    """Change every matching setting anywhere in the file."""
    if isinstance(node, dict):
        for k in list(node):
            v = node[k]
            if k == 'use_sim_time':
                node[k] = False
            elif k == 'robot_radius':
                node[k] = ROBOT_RADIUS
                node['footprint'] = FOOTPRINT        # the real rectangle is used instead
            elif k == 'inflation_radius':
                node[k] = INFLATION
                node['cost_scaling_factor'] = COST_SCALING
            elif k == 'yaw_goal_tolerance':
                # 24 Nov video: at a goal (screen) the robot wiggled left-right for a long time.
                # A skid-steer turn cannot be done in tiny steps (the motors need a strong push to
                # turn at all) -> it overshoots a narrow +-14 deg window again and again.
                # +-23 deg is reached in one turn; the AI and the stations do not need more.
                node[k] = YAW_TOLERANCE
            elif k == 'transform_tolerance' and isinstance(v, (int, float)):
                # 23 Nov log: odom arrived up to ~1 s late while the Pi was busy ->
                # "extrapolation into the future" and Nav2 aborted. Accept late data up to 0.6 s.
                node[k] = max(float(v), TF_TOLERANCE)
            elif k == 'laser_max_range':
                node[k] = 12.0
            elif k == 'max_accel' and isinstance(v, list) and len(v) == 3:
                node[k] = [0.4, 0.0, 1.5]      # 24 Nov: gentle speed-up (was 2.5 m/s2 -> jerky starts)
            elif k == 'max_decel' and isinstance(v, list) and len(v) == 3:
                node[k] = [-0.6, 0.0, -2.0]    # gentle slow-down, still stops within ~5 cm at 0.2 m/s
            elif k == 'max_velocity' and isinstance(v, list) and len(v) == 3:
                node[k] = [MAX_SPEED, 0.0, MAX_TURN]
            elif k == 'min_velocity' and isinstance(v, list) and len(v) == 3:
                node[k] = [-MAX_SPEED, 0.0, -MAX_TURN]
            elif k == 'scan' and isinstance(v, dict) and 'data_type' in v:
                v['obstacle_min_range'] = SELF_IGNORE   # costmap: ignore the robot's own body
                walk(v)
            elif k == 'progress_checker' and isinstance(v, dict) and 'required_movement_radius' in v:
                # 21 Nov log: "Failed to make progress" while the robot was TURNING in a tight
                # junction - the simple checker only counts driving, not turning. This one counts both:
                v['plugin'] = 'nav2_controller::PoseProgressChecker'
                v['required_movement_angle'] = 0.35    # ... OR turn 20 deg ...
                v['required_movement_radius'] = 0.20   # must move 20 cm ...
                v['movement_time_allowance'] = 10.0    # ... within 10 s (stuck is still noticed fast)
            elif k == 'FollowPath' and isinstance(v, dict) and 'plugin' in v:
                node[k] = dict(FOLLOW_PATH)
            else:
                walk(v)
    elif isinstance(node, list):
        for item in node:
            walk(item)


def main():
    if not os.path.isfile(SRC):
        sys.exit(f'Nav2 is not installed ({SRC} missing):  sudo apt install ros-jazzy-nav2-bringup')
    with open(SRC) as f:
        params = yaml.safe_load(f)
    walk(params)
    params['amcl']['ros__parameters'].update(AMCL_TUNING)
    for cm in ('local_costmap', 'global_costmap'):
        params[cm][cm]['ros__parameters']['footprint_padding'] = 0.0   # exact size, no extra 1 cm
        params[cm][cm]['ros__parameters']['transform_tolerance'] = TF_TOLERANCE
    g = params['global_costmap']['global_costmap']['ros__parameters']
    g.pop('footprint', None)
    g['robot_radius'] = PLAN_RADIUS
    for layer in g.values():
        if isinstance(layer, dict) and 'inflation_radius' in layer:
            layer['inflation_radius'] = GLOBAL_INFLATION
            layer['cost_scaling_factor'] = GLOBAL_COST_SCALING
    planner = params['planner_server']['ros__parameters']
    for v in planner.values():
        if isinstance(v, dict) and 'plugin' in v:
            v['tolerance'] = 0.5          # goal a bit inside a wall zone: plan to the nearest free spot
            v['allow_unknown'] = True
    bt = make_fast_bt()
    if bt:
        params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt
    with open(DST, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    print('Written:', DST)
    fp = params['controller_server']['ros__parameters']['FollowPath']['plugin']
    print('Path follower :', fp.split('::')[-1])
    print(f'Robot shape   : rectangle {2 * HALF_LENGTH:.2f} m long x {2 * HALF_WIDTH:.2f} m wide,  keep-away {INFLATION} m')
    print(f'Path planning : keeps to the middle of free areas (soft keep-away {GLOBAL_INFLATION} m)')
    print(f'Own body      : LiDAR hits closer than {SELF_IGNORE} m are ignored by the costmap')
    print(f'Max speed     : {MAX_SPEED} m/s,  turn {MAX_TURN} rad/s,  never slower than 0.12 m/s when driving')
    print('Progress check: must move 20 cm OR turn 20 deg within 10 s (turning counts as progress)')
    print('Blocked path  : ' + ('re-plans 3x per second; recover = back 15 cm / turn 45 deg / wait 1 s ('
                                + str(bt) + ')' if bt else 'standard Nav2 behaviour (file not found)'))
    print(f'Path planning : robot treated as a {PLAN_RADIUS} m circle -> it can always plan out of the start box')
    print('Driving checks: real rectangle, 0.5 s look-ahead, turns on the spot only above 45 deg')
    print('AMCL          : trusts the gyro for turns (measured 2% error) - stays sure while turning')
    print('Path following: looks 0.4 m ahead (the setting with which the loading stations were reached)')
    print('Reversing     : only when blocked, 15 cm, slowly (8 cm/s)')
    print('Paths         : keep away from walls, but still go through the obstacle zone')
    print('Smooth        : speeds up at 0.4 m/s2 and slows at 0.6 m/s2 (no jerks)')
    print('At a goal     : facing within +-23 deg is enough, turns on the spot at 0.6 rad/s (no wiggling)')
    print(f'Late data     : position data up to {TF_TOLERANCE} s late is accepted (busy Pi) - no abort')
    print('Start pose    : robot starts at the map start spot (x 0, y 0, facing as when mapping began)')


if __name__ == '__main__':
    main()
