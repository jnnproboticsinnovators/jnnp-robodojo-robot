#!/usr/bin/env python3
"""
RoboDojo /cmd_vel -> Arduino bridge (ROS 2 Jazzy).

teleop_twist_keyboard publishes /cmd_vel (geometry_msgs/Twist).
This node turns it into the one-letter commands the Arduino code already
understands (w forward, s back, a turn left, d turn right, x stop) and sends
them 10 times a second, so the Arduino's 0.5 s safety stop stays happy.
It also publishes the Arduino's encoder counts on /wheel_ticks.
"""
import glob
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray

LINEAR_DEADBAND = 0.05    # m/s  below this counts as "not moving"
ANGULAR_DEADBAND = 0.10   # rad/s below this counts as "not turning"
CMD_TIMEOUT = 1.0         # s    stop if no key for 1 s (covers the keyboard repeat delay)


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        ports = sorted(glob.glob('/dev/ttyACM*'))
        port = self.declare_parameter('port', ports[0] if ports else '/dev/ttyACM0').value
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.get_logger().info(f'Connected to Arduino on {port}')

        self.last_twist = Twist()
        self.last_time = self.get_clock().now()
        self.last_cmd = None
        self.buf = ''

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.ticks_pub = self.create_publisher(Int64MultiArray, '/wheel_ticks', 10)
        self.create_timer(0.1, self.tick)

    def on_cmd_vel(self, msg):
        self.last_twist = msg
        self.last_time = self.get_clock().now()

    def choose_command(self):
        age = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        if age > CMD_TIMEOUT:
            return 'x'
        lin = self.last_twist.linear.x
        ang = self.last_twist.angular.z
        if abs(ang) > ANGULAR_DEADBAND:          # any turning request -> turn
            return 'a' if ang > 0 else 'd'       # ROS: positive z = turn left
        if lin > LINEAR_DEADBAND:
            return 'w'
        if lin < -LINEAR_DEADBAND:
            return 's'
        return 'x'

    def tick(self):
        cmd = self.choose_command()
        try:
            self.ser.write(cmd.encode())
            if cmd != self.last_cmd:
                self.get_logger().info(f'command: {cmd}')
                self.last_cmd = cmd
            self.buf += self.ser.read(self.ser.in_waiting or 1).decode(errors='ignore')
        except serial.SerialException as e:
            self.get_logger().error(f'Arduino disconnected: {e}')
            return
        while '\n' in self.buf:
            line, self.buf = self.buf.split('\n', 1)
            p = line.split()
            if len(p) == 5 and p[0] == 'ENC':
                msg = Int64MultiArray()
                msg.data = [int(p[1]), int(p[2])]
                self.ticks_pub.publish(msg)

    def stop(self):
        try:
            self.ser.write(b'x')
            self.ser.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
