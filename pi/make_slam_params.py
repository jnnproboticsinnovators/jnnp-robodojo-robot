#!/usr/bin/env python3
"""
JNNP RoboDojo - SLAM settings for the REAL robot (run once on the Pi):

    python3 ~/make_slam_params.py

Writes ~/slam_params_real.yaml (robotslam / robotmap use it automatically when it exists).

Why: on the big field a whole part of the map was drawn TURNED ~30 deg in one block. SLAM's
scan matcher searches +-20 deg by default and, entering an open area with similar plywood walls,
it picked a wrong match (or "closed a loop" with a place that only looked the same). The robot
now has a gyro (IMU) that knows the heading well, so SLAM is told to:
  * search only +-5 deg around the IMU heading (the IMU is calibrated and good)
  * but +-30 cm around the position, and trust the LiDAR for position: the wheel encoders
    under-count distance (test: wheels 44 cm, real 72 cm)
  * accept a "loop closure" only when the match is very sure and the robot is close by
  * use LiDAR hits up to 8 m only (the A1 is unreliable further away)
  * add a map update every 20 cm / ~11 deg (more, smaller steps = fewer big mistakes)
"""
import os
import yaml

SRC = '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml'
OUT = os.path.expanduser('~/slam_params_real.yaml')

SETTINGS = {
    'use_sim_time': False,
    'resolution': 0.05,
    'max_laser_range': 8.0,
    'minimum_travel_distance': 0.2,
    'minimum_travel_heading': 0.2,
    # scan matching: small search window around the IMU/wheel position
    'correlation_search_space_dimension': 0.6,  # +-30 cm: the wheels under-count distance, so
                                                # the LiDAR must be free to fix the POSITION
    'correlation_search_space_resolution': 0.01,
    'coarse_search_angle_offset': 0.087,        # +-5 deg (default 0.349 = +-20 deg)
    'coarse_angle_resolution': 0.0175,          # 1 deg
    'fine_search_angle_offset': 0.00349,
    'distance_variance_penalty': 0.3,           # default 0.5: trust the LiDAR for position (wheels slip)
    'angle_variance_penalty': 2.0,              # default 1.0: trust the IMU heading more
    'minimum_angle_penalty': 0.9,
    'minimum_distance_penalty': 0.5,
    'link_match_minimum_response_fine': 0.15,
    # loop closing: only when very sure
    'do_loop_closing': True,
    'loop_search_maximum_distance': 2.0,        # default 3.0
    'loop_match_minimum_chain_size': 15,        # default 10
    'loop_match_minimum_response_coarse': 0.45,
    'loop_match_minimum_response_fine': 0.6,    # default 0.45
    'loop_search_space_dimension': 4.0,
}


def main():
    d = yaml.safe_load(open(SRC))
    p = d['slam_toolbox']['ros__parameters']
    p.update(SETTINGS)
    yaml.safe_dump(d, open(OUT, 'w'), sort_keys=False)
    print('Written:', OUT)
    print('SLAM now: heading from the IMU (+-5 deg), position from the LiDAR (+-30 cm), strict loop closing, LiDAR up to 8 m')


if __name__ == '__main__':
    main()
