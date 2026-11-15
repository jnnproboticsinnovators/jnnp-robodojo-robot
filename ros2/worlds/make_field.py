# Generates robodojo_field.sdf (approximate layout from the competition figure).
import math

L, W = 6.70356, 4.868    # from Figure 1: 6703.56 mm x 4868 mm
WALL_H, WALL_T = 0.30, 0.03
FLOOR = (0.80, 0.62, 0.30)

out = []
def color(r, g, b):
    return f"<ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"

def box(name, x, y, z, sx, sy, sz, rgb, static=True, roll=0, pitch=0, yaw=0, mass=0.2, collide=True):
    col = f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>" if collide else ""
    inertia = "" if static else (
        f"<inertial><mass>{mass}</mass><inertia>"
        f"<ixx>{mass*(sy*sy+sz*sz)/12:.6f}</ixx><iyy>{mass*(sx*sx+sz*sz)/12:.6f}</iyy><izz>{mass*(sx*sx+sy*sy)/12:.6f}</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")
    out.append(f"""
    <model name='{name}'>
      <static>{'true' if static else 'false'}</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <link name='link'>{inertia}
        {col}
        <visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry><material>{color(*rgb)}</material></visual>
      </link>
    </model>""")

def cyl(name, x, y, r, h, rgb, static=False, mass=0.3):
    inertia = "" if static else (
        f"<inertial><mass>{mass}</mass><inertia><ixx>{mass*(3*r*r+h*h)/12:.6f}</ixx><iyy>{mass*(3*r*r+h*h)/12:.6f}</iyy>"
        f"<izz>{mass*r*r/2:.6f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")
    out.append(f"""
    <model name='{name}'>
      <static>{'true' if static else 'false'}</static>
      <pose>{x:.3f} {y:.3f} {h/2:.3f} 0 0 0</pose>
      <link name='link'>{inertia}
        <collision name='c'><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
        <visual name='v'><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry><material>{color(*rgb)}</material></visual>
      </link>
    </model>""")

def wall(name, x1, y1, x2, y2):
    length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)
    box(name, (x1 + x2) / 2, (y1 + y2) / 2, WALL_H / 2, length + WALL_T, WALL_T, WALL_H, (0.72, 0.53, 0.20), yaw=yaw)

def patch(name, x, y, sx, sy, rgb):          # flat coloured floor area (drive over it)
    box(name, x, y, 0.002, sx, sy, 0.004, rgb, collide=False)

def ramp(name, x, y, length, width, height, rgb):   # up-slope, flat top, down-slope along x
    slope = length / 3
    ang = math.atan2(height, slope)
    hyp = math.hypot(slope, height)
    box(name + "_up",   x - slope, y, height / 2, hyp, width, 0.01, rgb, pitch=-ang)
    box(name + "_top",  x,         y, height,     slope, width, 0.01, rgb)
    box(name + "_down", x + slope, y, height / 2, hyp, width, 0.01, rgb, pitch=ang)


# ======== Layout from Figure 1 (top view, mm). Origin = bottom-left corner. ========
# x: along the 6703.56 mm side, y: along the 4868 mm side (back wall at y = W)
def Y(from_top): return W - from_top          # convert "distance from back wall" to y

X172, X662, X740 = 0.700, 5.250, 5.968        # from 5249.56 / 4549.56 / 5267.56 dims
Y210, Y290, Y367 = Y(1.278), Y(2.019), Y(2.714)   # 2714 dim -> long middle walls

box("field_floor", L / 2, W / 2, -0.005, L, W, 0.01, FLOOR)

# outer walls
wall("wall_front", 0, 0, L, 0); wall("wall_back", 0, W, L, W)
wall("wall_left", 0, 0, 0, W);  wall("wall_right", L, 0, L, W)

# long corridor walls (serpentine path)
wall("wall_a", X172, Y210, X740, Y210)        # under the top band (ramp strip side)
wall("wall_b", 0.0,  Y290, X662, Y290)        # from the left wall
wall("wall_c", X172, Y367, X740, Y367)        # above the bottom area
wall("wall_right_column", X740, W, X740, Y367)

# teeth (grater) on the middle corridor walls
for i, x in enumerate([1.407, 1.843, 4.352, 4.787, 5.204]):
    wall(f"tooth_b{i}", x, Y290, x, Y290 - 0.28)
for i, x in enumerate([1.250, 1.667, 4.185, 4.611, 5.037]):
    wall(f"tooth_c{i}", x, Y367, x, Y367 + 0.30)

# top band: short stub on the left, three bays against the back wall
wall("stub_top_left", 1.435, W, 1.435, Y(0.444))
BAY_X = [2.685, 3.398, 4.111, 4.852]
for i, x in enumerate(BAY_X):
    wall(f"bay_wall_{i}", x, W, x, Y(0.722))

# bottom area: thin grey lines in Figure 1 = painted floor lines (robot can cross)
patch("line_room_1", 0.806, Y367 / 2, 0.03, Y367, (0.35, 0.35, 0.35))
patch("line_room_2", 2.611, Y367 / 2, 0.03, Y367, (0.35, 0.35, 0.35))

# ---------- 4. loading zone (bottom middle): two walls + green bar on top ----------
wall("loading_wall_top",    3.093, 1.435, 4.852, 1.435)
wall("loading_wall_bottom", 3.093, 0.722, 4.852, 0.722)
patch("loading_zone", 3.97, 1.08, 1.70, 0.66, (0.55, 0.80, 0.50))
box("loading_bar", 3.782, 1.08, WALL_H + 0.01, 0.21, 1.27, 0.02, (0.55, 0.85, 0.55))
for i, y in enumerate([0.60, 1.08, 1.56]):
    cyl(f"loading_bar_hole_{i}", 3.782, y, 0.035, 0.001, (0.35, 0.55, 0.35), static=True)
for i, rgb in enumerate([(0.85, 0.15, 0.15), (0.15, 0.15, 0.15), (0.15, 0.35, 0.85)]):
    box(f"load_{i+1}", 3.35 + i * 0.25, 1.08, 0.03, 0.06, 0.06, 0.06, rgb, static=False, mass=0.1)

# ---------- 5. colour-coded offloading zones + 6. start (bays on the back wall) ----------
bay_cy = Y(0.36)
patch("offload_black", (BAY_X[0] + BAY_X[1]) / 2, bay_cy, 0.65, 0.66, (0.35, 0.35, 0.38))
patch("offload_red",   (BAY_X[1] + BAY_X[2]) / 2, bay_cy, 0.65, 0.66, (0.85, 0.30, 0.30))
patch("start_zone",    (BAY_X[2] + BAY_X[3]) / 2, bay_cy, 0.65, 0.66, (0.95, 0.92, 0.82))
patch("offload_blue",  6.34, Y(2.0), 0.60, 0.60, (0.25, 0.40, 0.85))

# ---------- 1. terrain ----------
# ramp + gravel: 420 mm strip along wall_a (top band)
ramp("ramp_gravel", 2.0, Y(1.068), 1.2, 0.40, 0.05, (0.45, 0.45, 0.48))
for i, gx in enumerate([3.0, 3.15, 3.3, 3.45, 3.6, 3.75]):
    box(f"gravel_{i}", gx, Y(1.068) + (0.1 if i % 2 else -0.1), 0.008, 0.03, 0.03, 0.016, (0.4, 0.4, 0.4))
# ramp + sand: corridor between wall_a and wall_b
ramp("ramp_sand", 3.0, (Y210 + Y290) / 2, 1.2, 0.60, 0.05, (0.50, 0.50, 0.52))
patch("sand", 4.1, (Y210 + Y290) / 2, 0.8, 0.65, (0.93, 0.85, 0.60))
# grater: thin raised strips across the toothed corridor
for i in range(8):
    box(f"grater_{i}", 2.6 + i * 0.15, (Y290 + Y367) / 2, 0.006, 0.02, 0.60, 0.012, (0.35, 0.35, 0.35))
# grass
patch("grass", 1.7, 1.2, 1.5, 1.6, (0.40, 0.65, 0.30))

# ---------- 2. dynamic obstacle (barrier in the top band; static for now) ----------
box("dynamic_obstacle_post", 2.45, Y(0.85), 0.15, 0.06, 0.06, 0.30, (0.9, 0.9, 0.9))
box("dynamic_obstacle_arm",  2.45, Y(1.0), 0.25, 0.04, 0.42, 0.04, (0.85, 0.20, 0.20))

# ---------- 3. cylinders (top-left area, on outlined squares) ----------
for name, x, y in [("cylinder_1", 0.45, Y(0.45)), ("cylinder_2", 0.95, Y(0.45)), ("cylinder_judges", 0.70, Y(1.0))]:
    patch(name + "_pad", x, y, 0.30, 0.30, (0.70, 0.52, 0.25))
    cyl(name, x, y, 0.075, 0.25, (0.82, 0.82, 0.82))

sdf = f"""<?xml version="1.0"?>
<!--
  RoboDojo competition field, walls from Figure 1 (top view with dimensions).
  Field {L} m x {W} m, walls {WALL_H} m high. Origin = front-left corner.
  ADJUST the numbers once you have the real measurements.
    Walls/corridors/bays/loading walls: measured from Figure 1.
    Zone colours and terrain placement: matched to Figure 2 (adjust if needed).
-->
<sdf version="1.9">
  <world name="robodojo_field">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>
{''.join(out)}
  </world>
</sdf>
"""
import sys
open(sys.argv[1], "w").write(sdf)
print("models:", len(out))
