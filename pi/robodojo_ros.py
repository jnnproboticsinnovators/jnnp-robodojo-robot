#!/usr/bin/env python3
"""
JNNP RoboDojo - the REAL robot as a ROS 2 robot (runs on the Raspberry Pi).

    python3 ~/robodojo_ros.py

What it does
  * LiDAR (RPLIDAR A1, /dev/ttyUSB0)  ->  /scan          (sensor_msgs/LaserScan, frame "laser")
  * Arduino encoders + MPU6050 gyro   ->  /odom + TF odom -> base_footprint
                                          (distance from the encoders, TURNING from the gyro = no slip)
  * MPU6050 (Arduino v11)             ->  /imu
  * /cmd_vel (from teleop or Nav2)    ->  Arduino "V <left> <right>" wheel speeds (needs Arduino v10)
  * /dump_cmd (Float64, radians)      ->  Arduino 'u' (up) / 'n' (down)
  * /camera_cmd (Bool)                ->  camera + waste AI on/off (service waste-detect)
  * /camera_on  (Bool, every 1 s)     <-  is the camera + waste AI running?
  * fixed TF base_footprint -> base_link -> laser

Needs in the same folder: lidar_a1.py
Do NOT run it together with robot_teleop_ubuntu.py (both want the Arduino and the LiDAR).
Ctrl+C to stop (the robot stops too).
"""
import glob
import json
import math
import os
import subprocess
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from lidar_a1 import A1Lidar

# ------------------------------------------------------------------ robot measurements
TICKS_PER_METER = 1400.0   # encoder ticks for 1 m of travel  (CALIBRATE: Step 5)
TRACK_WIDTH = 0.371        # m, left wheels to right wheels   (CALIBRATE: Step 5, skid-steer = a bit more)
LEFT_ENC_SIGN = -1         # left encoder counts down when driving forward
RIGHT_ENC_SIGN = +1
MAX_WHEEL_SPEED = 0.45     # m/s - never ask a wheel for more than this
USE_IMU = True             # take the turning from the MPU6050 gyro (Arduino v11) when it is working
IMU_SIGN = +1              # +1: left turn = gyro yaw goes up (chip side up). -1 if mounted upside down
IMU_TIMEOUT = 0.5          # s without IMU data -> fall back to the wheels

# ------------------------------------------------------------------ LiDAR mounting
LIDAR_FRONT_ANGLE = 0.0    # deg: LiDAR angle that points to the robot's FRONT (CHECK: Step 5)
LIDAR_CLOCKWISE = True     # the A1 counts angles clockwise
LASER_X, LASER_Y, LASER_Z = 0.0, 0.0, 0.15   # m, LiDAR centre from the robot centre (on the floor)
IGNORE_CLOSER = 0.18       # m, ignore hits on the robot's own body
RANGE_MAX = 12.0

# Measured values saved by the calibration tools override the guesses above
CALIB_FILE = os.path.expanduser('~/robodojo_calib.json')
try:
    with open(CALIB_FILE) as _f:
        _cal = json.load(_f)
    TICKS_PER_METER = float(_cal.get('TICKS_PER_METER', TICKS_PER_METER))
    TRACK_WIDTH = float(_cal.get('TRACK_WIDTH', TRACK_WIDTH))
    LIDAR_CLOCKWISE = bool(_cal.get('LIDAR_CLOCKWISE', LIDAR_CLOCKWISE))
    LIDAR_FRONT_ANGLE = float(_cal.get('LIDAR_FRONT_ANGLE', LIDAR_FRONT_ANGLE))
    IMU_SIGN = int(_cal.get('IMU_SIGN', IMU_SIGN))
except (OSError, ValueError):
    pass

ARDUINO_PORT = None        # None = first /dev/ttyACM*
CMD_TIMEOUT = 0.5          # s without /cmd_vel -> stop


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class RoboDojo(Node):
    def __init__(self):
        super().__init__('robodojo_robot')
        port = ARDUINO_PORT or (sorted(glob.glob('/dev/ttyACM*')) or ['/dev/ttyACM0'])[0]
        self.ser = serial.Serial(port, 115200, timeout=0)
        time.sleep(2.0)                         # Arduino restarts when the port opens
        self.get_logger().info(f'Arduino on {port}')
        self.get_logger().info(f'Robot values: TICKS_PER_METER={TICKS_PER_METER:.0f}  TRACK_WIDTH={TRACK_WIDTH:.3f} m  '
                               f'LIDAR_CLOCKWISE={LIDAR_CLOCKWISE}  LIDAR_FRONT_ANGLE={LIDAR_FRONT_ANGLE:.0f}')
        self.ser_lock = threading.Lock()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.imu_yaw = None          # radians, from the Arduino gyro
        self.imu_rate = 0.0
        self.imu_time = 0.0
        self.prev_imu_yaw = None
        self.imu_state = "waiting"
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Float64, 'dump_cmd', self.on_dump, 10)
        self.create_subscription(Bool, 'camera_cmd', self.on_camera, 10)
        self.camera_pub = self.create_publisher(Bool, 'camera_on', 10)
        self.create_timer(1.0, self.report_camera)

        self.target = (0, 0)
        self.last_cmd = 0.0
        self.x = self.y = self.th = 0.0
        self.prev_ticks = None
        self.prev_time = None
        self.rx = ''
        self.version_seen = False

        self.create_timer(0.05, self.send_speed)       # 20 Hz to the Arduino
        self.create_timer(0.01, self.read_arduino)

        threading.Thread(target=self.lidar_loop, daemon=True).start()

    # ---------------------------------------------------------------- fixed frames
    def publish_static_tf(self):
        now = self.get_clock().now().to_msg()
        a = TransformStamped()
        a.header.stamp = now
        a.header.frame_id = 'base_footprint'
        a.child_frame_id = 'base_link'
        a.transform.rotation.w = 1.0
        b = TransformStamped()
        b.header.stamp = now
        b.header.frame_id = 'base_link'
        b.child_frame_id = 'laser'
        b.transform.translation.x = LASER_X
        b.transform.translation.y = LASER_Y
        b.transform.translation.z = LASER_Z
        b.transform.rotation.w = 1.0
        self.static_tf.sendTransform([a, b])

    # ---------------------------------------------------------------- driving
    def on_cmd_vel(self, msg):
        v, w = msg.linear.x, msg.angular.z
        vl = v - w * TRACK_WIDTH / 2
        vr = v + w * TRACK_WIDTH / 2
        biggest = max(abs(vl), abs(vr))
        if biggest > MAX_WHEEL_SPEED:                  # keep the turn shape, lower the speed
            vl, vr = vl * MAX_WHEEL_SPEED / biggest, vr * MAX_WHEEL_SPEED / biggest
        self.target = (int(vl * TICKS_PER_METER), int(vr * TICKS_PER_METER))
        self.last_cmd = time.time()

    def send_speed(self):
        l, r = self.target if time.time() - self.last_cmd < CMD_TIMEOUT else (0, 0)
        self.write(f'V {l} {r}\n')

    def on_dump(self, msg):
        self.write('u' if msg.data > 0.2 else 'n')

    # ---------------------------------------------------------------- camera (waste AI)
    def on_camera(self, msg):
        action = 'start' if msg.data else 'stop'
        r = subprocess.run(['sudo', '-n', 'systemctl', action, 'waste-detect'],
                           capture_output=True, text=True)
        if r.returncode != 0:
            self.get_logger().error(f'camera {action} failed: {r.stderr.strip()}')
        else:
            self.get_logger().info(f'camera + waste AI {"ON" if msg.data else "OFF"}')
        self.report_camera()

    def report_camera(self):
        r = subprocess.run(['systemctl', 'is-active', '--quiet', 'waste-detect'])
        self.camera_pub.publish(Bool(data=(r.returncode == 0)))

    def write(self, text):
        with self.ser_lock:
            try:
                self.ser.write(text.encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Arduino write failed: {e}')

    # ---------------------------------------------------------------- odometry
    def read_arduino(self):
        try:
            self.rx += self.ser.read(self.ser.in_waiting or 1).decode(errors='ignore')
        except serial.SerialException as e:
            self.get_logger().error(f'Arduino read failed: {e}')
            return
        while '\n' in self.rx:
            line, self.rx = self.rx.split('\n', 1)
            p = line.split()
            if p[:1] == ['READY']:
                self.version_seen = True
                self.get_logger().info(line.strip())
                if 'v10' not in line and 'v11' not in line:
                    self.get_logger().error('Upload Arduino code v11 - this version cannot follow ROS speeds!')
                elif 'v11' not in line:
                    self.get_logger().warn('Arduino v10: no IMU. Upload v11 to use the MPU6050.')
            elif p[:1] == ['IMU']:
                self.on_imu_line(p)
            elif len(p) == 5 and p[0] == 'ENC':
                try:
                    self.update_odom(int(p[1]), int(p[2]))
                except ValueError:
                    pass

    def on_imu_line(self, p):
        if len(p) == 2 and p[1] == 'none':
            if self.imu_state != 'none':
                self.imu_state = 'none'
                self.get_logger().warn('No MPU6050 found on the Arduino (pins 20/21) - turning from the wheels')
            return
        if len(p) != 3:
            return
        try:
            yaw = IMU_SIGN * math.radians(int(p[1]) / 1000.0)
            rate = IMU_SIGN * math.radians(int(p[2]) / 1000.0)
        except ValueError:
            return
        if self.imu_state != 'ok':
            self.imu_state = 'ok'
            self.get_logger().info('MPU6050 gyro OK - turning is measured by the IMU')
        self.imu_yaw, self.imu_rate, self.imu_time = yaw, rate, time.time()
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.orientation.z, m.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.01]
        m.angular_velocity.z = rate
        m.angular_velocity_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.001]
        m.linear_acceleration_covariance[0] = -1.0       # no accelerometer data sent
        self.imu_pub.publish(m)

    def update_odom(self, left_raw, right_raw):
        now = self.get_clock().now()
        ticks = (LEFT_ENC_SIGN * left_raw, RIGHT_ENC_SIGN * right_raw)
        if self.prev_ticks is None:
            self.prev_ticks, self.prev_time = ticks, now
            return
        dl = (ticks[0] - self.prev_ticks[0]) / TICKS_PER_METER
        dr = (ticks[1] - self.prev_ticks[1]) / TICKS_PER_METER
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_ticks, self.prev_time = ticks, now
        if abs(dl) > 1.0 or abs(dr) > 1.0:          # encoder reset ('r') or glitch
            return
        ds, dth = (dl + dr) / 2, (dr - dl) / TRACK_WIDTH
        imu_fresh = USE_IMU and self.imu_yaw is not None and time.time() - self.imu_time < IMU_TIMEOUT
        if imu_fresh:
            if self.prev_imu_yaw is not None:
                dth = (self.imu_yaw - self.prev_imu_yaw + math.pi) % (2 * math.pi) - math.pi
            self.prev_imu_yaw = self.imu_yaw
        else:
            self.prev_imu_yaw = None
        self.x += ds * math.cos(self.th + dth / 2)
        self.y += ds * math.sin(self.th + dth / 2)
        self.th = (self.th + dth + math.pi) % (2 * math.pi) - math.pi

        stamp = now.to_msg()
        q = yaw_to_quat(self.th)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        if dt > 0:
            o.twist.twist.linear.x = ds / dt
            o.twist.twist.angular.z = self.imu_rate if imu_fresh else dth / dt
        self.odom_pub.publish(o)

    # ---------------------------------------------------------------- LiDAR
    def lidar_loop(self):
        n_bins = 360
        inc = 2 * math.pi / n_bins
        while rclpy.ok():
            lidar = A1Lidar()
            try:
                if not lidar.start():
                    raise IOError('LiDAR did not start (cable / power?)')
                self.get_logger().info('LiDAR scanning')
                t_prev = time.time()
                for turn in lidar.scans():
                    t_now = time.time()
                    ranges = [float('inf')] * n_bins
                    for a, d in turn:
                        if d < IGNORE_CLOSER or d > RANGE_MAX:
                            continue
                        rel = a - LIDAR_FRONT_ANGLE
                        ang = math.radians(-rel if LIDAR_CLOCKWISE else rel)
                        i = int(round((ang + math.pi) / inc)) % n_bins
                        if d < ranges[i]:
                            ranges[i] = d
                    s = LaserScan()
                    s.header.stamp = self.get_clock().now().to_msg()
                    s.header.frame_id = 'laser'
                    s.angle_min = -math.pi
                    s.angle_max = math.pi - inc
                    s.angle_increment = inc
                    s.scan_time = max(0.05, t_now - t_prev)
                    s.time_increment = s.scan_time / n_bins
                    s.range_min = IGNORE_CLOSER
                    s.range_max = RANGE_MAX
                    s.ranges = ranges
                    self.scan_pub.publish(s)
                    t_prev = t_now
                    if not rclpy.ok():
                        break
            except Exception as e:
                self.get_logger().warn(f'LiDAR: {e} - retrying')
                time.sleep(1)
            finally:
                lidar.close()

    def stop(self):
        self.target = (0, 0)
        self.write('V 0 0\n')
        self.write('x')


def main():
    rclpy.init()
    node = RoboDojo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
