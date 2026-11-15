# JNNP RoboDojo Robot – Autonomous Waste Management Robot

Software for the JNNP Robotics Innovators Club robot for the RoboDojo 2026 competition: a four-wheel
skid-steer robot that collects waste, sorts it by type (metal, organic, plastic) with a camera and AI,
maps the field with a LiDAR and drives itself with ROS 2 Nav2.

**Team:** Jane Maina (Team Leader), Munyao Nthenya, Ephantus Kariuki Kihara,
Francisco Mbugua Ngigi (Trainer) – Jeremiah Nyaga National Polytechnic (JNNP), Kenya.

Full design description: [`docs/JNNP_RoboDojo_Software_Architecture_Rev2.docx`](docs/).

## System at a glance

| Layer | Hardware | Software |
| --- | --- | --- |
| Operator station | Laptop, Windows 11 + WSL Ubuntu 24.04 | ROS 2 Jazzy, Gazebo Harmonic, RViz 2 |
| High-level controller | Raspberry Pi 4, Ubuntu Server 24.04 | ROS 2 Jazzy (Zenoh), SLAM Toolbox, Nav2, YOLO11s (ONNX), LiDAR driver |
| Low-level controller | Arduino Mega 2560 + Monster Moto shield (VNH2SP30) | C++ firmware: drive, wheel-speed control, encoders, servo, MPU6050 gyro |

Sensors: RPLIDAR A1M8 (USB), Pi Camera Module 3 (CSI), wheel encoders, MPU6050 IMU.
Actuators: 4 x GA25-370 motors (two per side), TowerPro MG995 dump-bin servo.

## Repository layout

```
arduino/            Arduino Mega firmware
  jnnp_drive_v11/     current: v10 + MPU6050 gyro (SDA 20, SCL 21)
  jnnp_drive_v10/     ROS speed command (V), closed-loop wheel speed
  jnnp_keyboard_drive_v9/  keyboard-only drive (older)
  tests/              motor, encoder, servo test sketches
pi/                 programs that run on the Raspberry Pi
  robodojo_ros.py     ROS 2 robot node: /scan, /odom, /imu, /cmd_vel, /dump_cmd, camera switch
  robot_teleop_ubuntu.py  direct arrow-key teleop (no ROS), LiDAR safety stop, reactive autonomy
  lidar_a1.py         RPLIDAR A1 driver (pyserial only)
  lidar_view.py       live LiDAR top view in the browser (port 8001)
  robodojo_mission.py Nav2 mission: home -> load -> offload (dump) -> home
  make_nav2_params.py writes the Nav2 settings for this robot
  vision/             waste detector (ONNX) + live stream (port 8000)
  systemd/            Zenoh router service
laptop/             programs that run on the laptop (WSL Ubuntu)
  robodojo_shortcuts.sh  one-word commands (robotengine, robotdrive, gazebo, nav ...)
  arrow_teleop.py     ROS arrow teleop (real robot and Gazebo)
  robot_calibrate.py, robot_turn_calibrate.py, robot_localize.py  calibration + auto-localisation
  robodojo_map2png.py turns a saved map into a picture
ros2/               Gazebo simulation helpers, launch file and game field
maps/               saved maps (.pgm + .yaml) and mission regions files
docs/               software architecture document
legacy/             earlier programs, kept for reference
```

The ROS 2 robot description package (`jnnp_robodojo_robot`: URDF, meshes, launch) lives in the
laptop's `~/ros2_ws/src/` and is added to this repository under `ros2_ws/src/`.

## Controls

The robot is always driven with the arrow keys:

| Key | Action |
| --- | --- |
| ↑ / ↓ | forward / backward |
| ← / → | turn left / right |
| Space | stop |
| + / - | faster / slower |
| U / N | dump bin up / down |
| C | camera + waste AI on / off |
| Q | quit |

## Quick start

**Laptop (once):** add `export PI="<pi-username>@<pi-address>"` and
`source ~/robodojo_shortcuts.sh` to `~/.bashrc` (copy `laptop/robodojo_shortcuts.sh` to `~`).

**Pi (once):** ROS 2 Jazzy, `ros-jazzy-rmw-zenoh-cpp`, `ros-jazzy-slam-toolbox`,
`ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `python3-serial`; set
`export RMW_IMPLEMENTATION=rmw_zenoh_cpp` in `~/.bashrc`; install `pi/systemd/zenoh-router.service`;
run `python3 ~/make_nav2_params.py`. Upload `arduino/jnnp_drive_v11` to the Mega.

| Task | Commands (one per terminal) |
| --- | --- |
| Drive directly | `robotteleop` |
| Map the real field | `robotengine`, `robotdrive`, `robotslam`, `robotrviz`, then `robotsavemap` |
| Navigate on a map | `robotengine`, `robotnav <map>`, `robotrviz`, `robotlocalize <map>` |
| Run a mission | the navigation set + `robotmission <map>` |
| Simulation | `gazebo`, `slam`, `rviz`, `gzteleop`, `savemap`, then `nav` |

The trained detector `best.onnx` (36 MB) is not stored here; place it in `~/waste_ai/` on the Pi.

## Status (November 2026)

- Working: firmware v10, both arrow teleops, waste detection, LiDAR driver, ROS 2 link over Zenoh,
  SLAM mapping (real robot and Gazebo), Gazebo simulation.
- Being tested: firmware v11 with the MPU6050 gyro, Nav2 on the real field, missions,
  calibration and auto-localisation tools.
