#!/usr/bin/env python3
"""
JNNP RoboDojo - Gazebo Nav2: stop the robot seeing ITSELF as a wall (run once on the laptop):

    python3 ~/fix_gazebo_nav_self.py

Why (1): the robot stood inside a "too close to a wall" zone (all the CYAN around it in RViz):
the Gazebo settings used a big circle for the robot + a wide keep-away, and the start box is
small - so Nav2 thought the robot was already touching the walls and refused to move.
Now Gazebo uses the real robot's rectangle and the small keep-away that works on the real robot.

Why (2): in Gazebo the LiDAR also hits the robot's own body (cabin, bin, camera mast). Nav2 then draws
a wall all round the robot (the red box in RViz), thinks it is boxed in, tries 5 recoveries and
gives up - the robot never moves. The real robot already ignores hits closer than 0.30 m;
this gives the Gazebo settings (~/ros2_ws/nav2_params_robodojo.yaml) the same rule.
A backup is kept as nav2_params_robodojo.yaml.bak

Why (3) - from gznavcheck: "Failed to make progress" = Nav2 SENT drive commands but the robot did
not move. Between Nav2 and the wheels sits Nav2's "collision monitor": it cuts the speed to 0
when LiDAR points are inside the robot's outline - and in Gazebo the LiDAR sees the robot's
own body / the start-box walls touching it. The real robot's engine throws away hits closer than
0.18 m before anything sees them; Gazebo did not. Now:
  * the Gazebo LiDAR ignores everything closer than 0.25 m (like the real robot)  -> URDF + rebuild
  * the collision monitor is switched off in Gazebo (the costmaps still avoid walls)
Why (4) - from gznavcheck: position unsure (+-49 cm). In Gazebo the robot's position is EXACT,
so AMCL is told to trust it (small movement-noise values) and it starts at the spawn spot.
"""
import os
import re
import shutil
import subprocess
import sys

import yaml

PATH = os.path.expanduser('~/ros2_ws/nav2_params_robodojo.yaml')
URDF = os.path.expanduser('~/ros2_ws/src/jnnp_robodojo_robot/urdf/jnnp_robodojo_robot.urdf')
LIDAR_MIN = 0.25       # m - Gazebo LiDAR: nearer hits are the robot itself (real robot: 0.18)
SELF_IGNORE = 0.30     # m - LiDAR hits closer than this are the robot's own body
# Same robot shape + keep-away as the REAL robot (that one drives out of the start box fine):
HALF_LENGTH, HALF_WIDTH = 0.13, 0.14   # robot outline 26 x 28 cm
FOOTPRINT = (f'[[{HALF_LENGTH}, {HALF_WIDTH}], [{HALF_LENGTH}, -{HALF_WIDTH}], '
             f'[-{HALF_LENGTH}, -{HALF_WIDTH}], [-{HALF_LENGTH}, {HALF_WIDTH}]]')
PLAN_RADIUS = 0.15     # planner treats the robot as a 0.15 m circle -> always plans out of the start box
LOCAL_INFLATION, LOCAL_SCALING = 0.30, 12.0     # >= inscribed radius (Nav2 needs it); steep fall-off
GLOBAL_INFLATION, GLOBAL_SCALING = 0.55, 2.5    # path: prefer the middle, but never blocks a gap

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
    x = re.sub(r'(<BackUp backup_dist=")[0-9.]+(")', r'\g<1>0.15\g<2>', x)
    x = re.sub(r'(number_of_retries=")6(" name="NavigateRecovery")', r'\g<1>10\g<2>', x)
    open(BT_DST, 'w').write(x)
    return BT_DST


MAX_SPEED, MAX_TURN = 0.25, 0.8
FOLLOW_PATH = {                      # identical to the real robot (make_nav2_params.py)
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': MAX_SPEED,
    'lookahead_dist': 0.4, 'min_lookahead_dist': 0.25, 'max_lookahead_dist': 0.7,
    'lookahead_time': 1.5,
    'rotate_to_heading_angular_vel': MAX_TURN,
    'transform_tolerance': 0.3,
    'use_velocity_scaled_lookahead_dist': False,
    'min_approach_linear_velocity': 0.12,
    'regulated_linear_scaling_min_speed': 0.12,
    'use_cost_regulated_linear_velocity_scaling': False,
    'approach_velocity_scaling_dist': 0.5,
    # 21 Nov log: 933x "detected collision ahead!" - this check tests the robot outline against the
    # MAP's 5 cm squares; in the narrow corridors the outline 'touched' wall squares while the real
    # robot had room, so it stopped again and again (30 recoveries in 40 s). Off: the planner keeps
    # the path in the middle, the local costmap still avoids walls/objects, and on the REAL robot
    # the collision monitor (real LiDAR points, not map squares) still stops it before any hit.
    'use_collision_detection': False,
    'max_allowed_time_to_collision_up_to_carrot': 0.5,
    'use_regulated_linear_velocity_scaling': True,
    'regulated_linear_scaling_min_radius': 0.6,
    'use_rotate_to_heading': True,
    'allow_reversing': False,
    'rotate_to_heading_min_angle': 0.785,
    'max_angular_accel': 2.0,
    'max_robot_pose_search_dist': 10.0,
}


if not os.path.isfile(PATH):
    sys.exit(f'Not found: {PATH}')
shutil.copy(PATH, PATH + '.bak')
d = yaml.safe_load(open(PATH))
n = 0


def walk(node):
    global n
    if isinstance(node, dict):
        for k, v in node.items():
            if isinstance(v, dict) and 'data_type' in v and 'topic' in v:      # a costmap LiDAR source
                v['obstacle_min_range'] = SELF_IGNORE
                v['raytrace_min_range'] = 0.0
                n += 1
            walk(v)
    elif isinstance(node, list):
        for item in node:
            walk(item)


walk(d)
for name, infl, sc in (('local_costmap', LOCAL_INFLATION, LOCAL_SCALING),
                       ('global_costmap', GLOBAL_INFLATION, GLOBAL_SCALING)):
    try:
        params = d[name][name]['ros__parameters']
    except (KeyError, TypeError):
        continue
    params['footprint_padding'] = 0.0
    if name == 'global_costmap':                 # planning: small circle (never "start occupied")
        params.pop('footprint', None)
        params['robot_radius'] = PLAN_RADIUS
    else:                                        # driving: the real rectangle
        params['footprint'] = FOOTPRINT
    for layer in params.values():
        if isinstance(layer, dict) and 'inflation_radius' in layer:
            layer['inflation_radius'] = infl
            layer['cost_scaling_factor'] = sc
bt = make_fast_bt()
try:
    if bt:
        d['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt
    pc = d['controller_server']['ros__parameters'].get('progress_checker', {})
    if 'required_movement_radius' in pc:
        pc['plugin'] = 'nav2_controller::PoseProgressChecker'   # turning counts as progress too
        pc['required_movement_angle'] = 0.35
        pc['required_movement_radius'] = 0.20
        pc['movement_time_allowance'] = 10.0
except (KeyError, TypeError, AttributeError):
    pass
try:
    for v in d['planner_server']['ros__parameters'].values():
        if isinstance(v, dict) and 'plugin' in v:
            v['tolerance'] = 0.5
            v['allow_unknown'] = True
    # Gazebo now drives with the SAME light path follower and values as the real robot
    # (from your log: "Failed to make progress" x20 - the heavy default follower kept stalling
    # on this laptop; Gazebo practice must behave like the competition robot anyway)
    cs = d['controller_server']['ros__parameters']
    old = str(cs.get('FollowPath', {}).get('plugin', '?')).split('::')[-1]
    cs['FollowPath'] = dict(FOLLOW_PATH)
    cs['controller_frequency'] = 10.0
    print(f'Path follower: {old} -> RegulatedPurePursuitController (same as the real robot)')
    vs = d.get('velocity_smoother', {}).get('ros__parameters', {})
    if 'max_velocity' in vs:
        vs['max_velocity'] = [MAX_SPEED, 0.0, MAX_TURN]
        vs['min_velocity'] = [-MAX_SPEED, 0.0, -MAX_TURN]
except (KeyError, TypeError, AttributeError):
    pass
try:
    a = d['amcl']['ros__parameters']
    a['laser_min_range'] = SELF_IGNORE
    for k in ('alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5'):
        a[k] = 0.02                     # Gazebo position is exact: trust it
    a['set_initial_pose'] = True
    a['initial_pose'] = {'x': -2.05, 'y': -1.12, 'z': 0.0, 'yaw': 1.5708}   # spawn spot
except (KeyError, TypeError):
    pass
cm_off = 0
try:
    cm = d['collision_monitor']['ros__parameters']
    for name in cm.get('polygons', []):
        if isinstance(cm.get(name), dict):
            cm[name]['enabled'] = False
            cm_off += 1
except (KeyError, TypeError):
    pass
yaml.safe_dump(d, open(PATH, 'w'), sort_keys=False)
print(f'Done: {n} costmap LiDAR sources now ignore hits closer than {SELF_IGNORE} m (+ AMCL).')
print(f'Robot shape: rectangle {2*HALF_LENGTH:.2f} x {2*HALF_WIDTH:.2f} m; keep-away {LOCAL_INFLATION} m '
      f'(driving), soft {GLOBAL_INFLATION} m (path) - same as the real robot.')
print(f'Saved {PATH}  (backup: {PATH}.bak)')
print(f'Collision monitor: {cm_off} zone(s) switched off (Gazebo only).')
print(f'Path planning: robot treated as a {PLAN_RADIUS} m circle -> plans out of the start box by itself.')
print('Blocked path: ' + ('re-plans 3x per second; recover = back 15 cm / turn 45 deg / wait 1 s'
                          if bt else 'standard Nav2 behaviour (behaviour-tree file not found)'))
print('AMCL: trusts the exact Gazebo position, starts at the spawn spot.')

# ---- Gazebo LiDAR: ignore hits closer than LIDAR_MIN (the robot's own body) ----
if not os.path.isfile(URDF):
    print(f'LiDAR: robot file not found ({URDF}) - skipped.')
else:
    u = open(URDF).read()
    changed = False

    def fix_sensor(m):
        global changed
        block = m.group(0)
        if not re.search(r'type="(gpu_)?(lidar|ray)"', block):
            return block
        new = re.sub(r'(<range>\s*<min>)\s*[0-9.]+\s*(</min>)', rf'\g<1>{LIDAR_MIN}\g<2>', block, count=1)
        changed = changed or new != block
        return new
    u2 = re.sub(r'<sensor\b.*?</sensor>', fix_sensor, u, flags=re.S)
    if changed:
        if not os.path.exists(URDF + '.before_selffix'):
            shutil.copy(URDF, URDF + '.before_selffix')
        open(URDF, 'w').write(u2)
        print(f'LiDAR: Gazebo LiDAR now ignores hits closer than {LIDAR_MIN} m. Rebuilding ...')
        r = subprocess.run('source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && '
                           'colcon build --packages-select jnnp_robodojo_robot',
                           shell=True, executable='/bin/bash')
        if r.returncode != 0:
            sys.exit('Build failed - send Claude the lines above.')
    elif re.search(rf'<min>\s*{LIDAR_MIN}\s*</min>', u):
        print('LiDAR: already set.')
    else:
        print('LiDAR: could not find the LiDAR range in the robot file - send Claude:  '
              'grep -n -A12 "<sensor" ' + URDF)

print('\nDONE. Close gzrun (Ctrl+C) and start it again. Do NOT click 2D Pose Estimate -')
print('the robot already starts at the right spot. Then Nav2 Goal.')
