#!/usr/bin/env python3
"""
JNNP RoboDojo - make the Nav2 settings file for the REAL robot (run once on the Pi).

    python3 ~/make_nav2_params.py

Reads the standard Nav2 settings and writes  ~/nav2_params_real.yaml  with our robot's values:
  * real time (no simulation clock)
  * robot size: radius 0.26 m, keep 0.30 m away from walls
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

ROBOT_RADIUS = 0.26
INFLATION = 0.30
MAX_SPEED = 0.20         # m/s
MAX_TURN = 0.8           # rad/s

FOLLOW_PATH = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': MAX_SPEED,
    'lookahead_dist': 0.4,
    'min_lookahead_dist': 0.25,
    'max_lookahead_dist': 0.7,
    'lookahead_time': 1.5,
    'rotate_to_heading_angular_vel': MAX_TURN,
    'transform_tolerance': 0.3,
    'use_velocity_scaled_lookahead_dist': False,
    'min_approach_linear_velocity': 0.08,
    'approach_velocity_scaling_dist': 0.5,
    'use_collision_detection': True,
    'max_allowed_time_to_collision_up_to_carrot': 1.0,
    'use_regulated_linear_velocity_scaling': True,
    'use_cost_regulated_linear_velocity_scaling': False,
    'regulated_linear_scaling_min_radius': 0.6,
    'regulated_linear_scaling_min_speed': 0.08,
    'use_rotate_to_heading': True,
    'allow_reversing': False,
    'rotate_to_heading_min_angle': 0.6,
    'max_angular_accel': 2.0,
    'max_robot_pose_search_dist': 10.0,
}


# AMCL (finding the robot on the map): trust the LiDAR more than the wheels,
# because a skid-steer robot's wheels slip when it turns.
AMCL_TUNING = {
    'alpha1': 0.4,   # turning error caused by turning (default 0.2)
    'alpha2': 0.2,   # turning error caused by driving
    'alpha3': 0.2,   # driving error caused by driving
    'alpha4': 0.4,   # driving error caused by turning
    'update_min_a': 0.1,   # re-check position after every 0.1 rad of turning (default 0.2)
    'update_min_d': 0.1,   # ... and every 10 cm
    'max_particles': 3000,
}


def walk(node):
    """Change every matching setting anywhere in the file."""
    if isinstance(node, dict):
        for k in list(node):
            v = node[k]
            if k == 'use_sim_time':
                node[k] = False
            elif k == 'robot_radius':
                node[k] = ROBOT_RADIUS
            elif k == 'inflation_radius':
                node[k] = INFLATION
            elif k == 'laser_max_range':
                node[k] = 12.0
            elif k == 'max_velocity' and isinstance(v, list) and len(v) == 3:
                node[k] = [MAX_SPEED, 0.0, MAX_TURN]
            elif k == 'min_velocity' and isinstance(v, list) and len(v) == 3:
                node[k] = [-MAX_SPEED, 0.0, -MAX_TURN]
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
    with open(DST, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    print('Written:', DST)
    fp = params['controller_server']['ros__parameters']['FollowPath']['plugin']
    print('Path follower :', fp.split('::')[-1])
    print(f'Robot radius  : {ROBOT_RADIUS} m,  keep-away {INFLATION} m')
    print(f'Max speed     : {MAX_SPEED} m/s,  turn {MAX_TURN} rad/s')
    print('AMCL          : trusts the LiDAR more than the wheels')


if __name__ == '__main__':
    main()
