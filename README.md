# JNNP RoboDojo Robot – Autonomous Waste Management Robot

Software, firmware and design files for the JNNP Robotics Innovators Club robot (Team 6, RoboDojo 2026):
a four-wheel skid-steer robot that reads the waste type (metal, organic, plastic) from a screen with a
camera and AI, drives to the matching loading station, crosses the field (grass, obstacles, ramp) and
dumps the waste at the matching offloading station – by itself, with ROS 2 Nav2 on a LiDAR map.

**Team:** Jane Kabuchi Maina (Team Leader, software), Munyao Nthenya (hardware), Ephantus Kariuki Kihara
(electronics), Francisco Mbugua Ngigi (Trainer, technical documentation) –
Jeremiah Nyaga National Polytechnic (JNNP), Kenya.

Design description: [`docs/JNNP_RoboDojo_Software_Architecture_Rev2.docx`](docs/).

## System at a glance

| Layer | Hardware | Software |
| --- | --- | --- |
| Operator station | Laptop, Windows 11 + WSL Ubuntu 24.04 | ROS 2 Jazzy, Gazebo Harmonic, RViz 2, waste AI (ONNX) |
| High-level controller | Raspberry Pi 4, Ubuntu Server 24.04 | ROS 2 Jazzy (Zenoh), SLAM Toolbox, Nav2, LiDAR driver, camera stream |
| Low-level controller | Arduino Mega 2560 + Monster Moto shield (VNH2SP30) | C++ firmware: wheel-speed control, encoders, MPU6050 gyro, dump servo |

Sensors: RPLIDAR A1M8, Pi Camera Module 3, wheel encoders, MPU6050 IMU.
Actuators: 4 x GA25-370 gearmotors (two per side), MG995 dump-bin servo.
Power: one 4S LiPo; the motor driver runs directly from the battery (the firmware scales motor power to
a 12 V equivalent), the Pi from its own 5 V buck converter.

## Repository layout

```
arduino/
  jnnp_drive_v18/       CURRENT firmware (see "Firmware versions")
  jnnp_drive_v15/       fallback (proven on the robot)
  jnnp_drive_v11/ v10/  older versions;  tests/  motor, encoder, servo test sketches
pi/                     runs on the Raspberry Pi
  robodojo_ros.py       ROS 2 robot node: /scan, /odom, /imu, /cmd_vel, /dump_cmd, camera switch
  add_wall_guard.py     adds the wall guard + corner guard to robodojo_ros.py (run once)
  make_nav2_params.py   writes the Nav2 settings for this robot (speed, footprint, AMCL, recovery)
  make_slam_params.py   SLAM Toolbox settings (IMU heading, strict loop closing)
  vision/               camera stream (port 8000) + screen finder + ONNX waste detector
  lidar_a1.py, lidar_view.py, robot_teleop_ubuntu.py, systemd/zenoh-router.service
laptop/                 runs on the laptop (WSL Ubuntu)
  install_robodojo.sh   installs everything (laptop + Gazebo + Pi) in one command
  robodojo_shortcuts.sh one-word commands (robotrun, robotcompete, robotmark, gzrun ...)
  robot_compete.py      THE COMPETITION MISSION (see below)
  robot_points.py       mark competition places by clicking in RViz
  robot_localize.py     finds the robot on the map from the LiDAR ("ALIGNED OK")
  robot_turntest.py     measures gyro drift and turn accuracy against the LiDAR
  arrow_teleop.py       arrow-key teleop; also marks places (M / K / B keys)
  waste_laptop.py, screen_focus.py   waste AI on the laptop (fast) with screen detection
  fix_gazebo_nav_self.py  Gazebo copy of the real robot's navigation settings
ros2_ws/src/jnnp_robodojo_robot/   ROS 2 description package (URDF, STL meshes, launch, worlds)
ros2/                   Gazebo simulation helpers and game field
maps/                   saved maps: jane1 = JNNP practice field (.pgm/.yaml/.png), Gazebo field
cad/step/               STEP design files of the robot (editable CAD)
docs/                   software architecture document
legacy/                 earlier programs, kept for reference
```

## Controls (teleop)

| Key | Action |
| --- | --- |
| ↑ / ↓ | forward / backward |
| ← / → | turn left / right |
| Space | stop |
| + / - | faster / slower |
| U / N | dump bin up / down |
| C | camera + waste AI on / off |
| M / K / B | mark this place / skip it / undo last mark (marking mode) |
| Q | quit |

## Install

1. **Laptop (once):** copy `laptop/*` to `~`, then `bash ~/install_robodojo.sh` – it copies the laptop files,
   updates the Gazebo settings and, when the robot is reachable, installs the Pi files and runs
   `make_nav2_params.py`, `make_slam_params.py` and `add_wall_guard.py` there.
   Put `export PI="<user>@<pi-address>"` (and optionally `export PI_MAC=<pi-wifi-mac>`, so the laptop can
   find the Pi by itself on any network) in `~/.bashrc`.
2. **Pi (once):** ROS 2 Jazzy, `ros-jazzy-rmw-zenoh-cpp`, `ros-jazzy-slam-toolbox`,
   `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `python3-serial`;
   `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`; install `pi/systemd/zenoh-router.service`.
3. **Arduino:** upload `arduino/jnnp_drive_v18`. Check: the Mega prints `READY v18`.
4. The trained detector `best.onnx` (36 MB) is not stored here; place it in `~/waste_ai/`.

## Using the robot

| Task | Window 1 | Window 2 |
| --- | --- | --- |
| Map the field | `robotmap` | `robotdrive`, then `robotsavemap NAME` |
| Mark the competition places (once per map) | `robotrun NAME` (wait for ALIGNED OK) | `robotmark NAME` – M at each place |
| Check a mark | `robotrun NAME` | drive onto it, `robotlocalize`, `robotwhere NAME` |
| Fix one mark | `robotrun NAME` | `robotremark NAME PLACE` |
| **Competition run** | `robotrun NAME` (robot on its start marks, ALIGNED OK) | `robotcompete NAME`, Enter at GO |
| Short safe run (no grass/obstacles/ramp) | `robotrun NAME` | `robotcompete NAME --short` |
| Measure turns | `robotrun NAME` | `robotturntest` |
| Simulation | `gzrun fast` (`gzview` = 3D window) | `gzmark`, `gzcompete plastic` |

After a run: `tail -60 ~/compete_log.txt` (every stage + the robot's position every second) and
`robotnavlog` (Nav2 errors).

### The mission (`robot_compete.py`)

Screen → camera + AI vote (2-of-3, camera switched off afterwards to save Pi power) → matching loading
station (wait 5 s) → route point by point: grass → obstacle zone → **ramp** (Nav2 to a run-up spot, then a
straight gyro-held drive over the ramp, then re-localisation) → matching offloading station → bin up 3 s,
bin down. Before GO it checks Nav2 is active, the robot's position, and that no mark sits on a wall.
While driving it re-finds the robot from the LiDAR when navigation reports it lost, and skips a route
point that Nav2 can only reach the "long way round".

### Safety layers

| Layer | Where | What it does |
| --- | --- | --- |
| Wall guard | robot node (Pi) | refuses to drive into anything within 10 cm in front/behind (raw LiDAR, works even when lost) |
| Corner guard | robot node (Pi) | can block turns on the spot that would swing a corner into a wall (`CORNER_GAP`, set 0 = off) |
| Slow back-up | Nav2 recovery | reverses only 15 cm at 8 cm/s when blocked |
| No surges | firmware v18 | max 170/255 wheel power in speed mode, 90/255 when backing up, anti-wind-up |
| Watchdog | firmware | motors stop if no command for 0.5 s |

## Firmware versions

| Version | Change |
| --- | --- |
| v11 | MPU6050 gyro for accurate turns |
| v12 | stronger push when a wheel is held back (grass) |
| v14 | motors fed from the 4S battery directly; power scaled to 12 V equivalent |
| v15 | speed-loop anti-wind-up (no sudden fast moves) – fallback version |
| v16–v17 | limits on reversing power; slow straight back-up |
| **v18** | **no surges in any direction (max 170/255 in speed mode)** – current |

## Lessons from RoboDojo 2026 testing (Nov 2026)

- Pi under-voltage (seen with `vcgencmd get_throttled`) made Nav2 restart mid-run – power the Pi with
  short, thick wires at 5.2 V.
- A loose Arduino USB cable stops the robot; tape it.
- Marks must be saved only when the robot is aligned (`robotlocalize` just before pressing M).
- In long straight corridors (the grass lane) the LiDAR cannot see how far along the robot is, and the
  wheels slip on grass – AMCL is set to trust the LiDAR more than wheel distance.
- The ramp slope looks like a wall to a 2-D LiDAR – hence the straight ramp drive.
- Obstacles lower than the LiDAR plane are invisible; they must be added to the map if their places are fixed.

## Status (November 2026)

- Working on the real robot: screen reading + AI (confident votes), loading-station navigation,
  marks verified on the jane1 map, slow/safe driving with wall guard, Pi power fix.
- Reached in testing: grass lane, obstacle zone and ramp area – but too slowly for the 3-minute limit.
- Next work: faster and more reliable route (localisation along the grass lane, obstacle-zone passage),
  a wireless communication centre, and fully autonomous mapping / navigation to a full bin.
