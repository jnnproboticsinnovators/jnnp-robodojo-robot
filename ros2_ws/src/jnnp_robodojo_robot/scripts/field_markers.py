#!/usr/bin/env python3
"""Publishes the game field (walls, zones, terrain, cylinders and numbered labels)
as an RViz MarkerArray on /field_markers, in the 'field' frame."""
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class FieldMarkers(Node):
    def __init__(self):
        super().__init__('field_markers')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('cylinders', [0.0])
        self.declare_parameter('frame_id', 'field')
        with open(self.get_parameter('layout_file').value) as f:
            self.layout = yaml.safe_load(f)
        cyl = list(self.get_parameter('cylinders').value)
        self.cylinders = [(cyl[i], cyl[i + 1]) for i in range(0, len(cyl) - 1, 2)]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'field_markers', qos)
        self.msg = self.build()
        self.create_timer(2.0, self.publish)
        self.publish()

    def marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def build(self):
        L, arr = self.layout, MarkerArray()
        # walls
        w = self.marker('walls', 0, Marker.LINE_LIST)
        w.scale.x = 0.04
        w.color.r = w.color.g = w.color.b = 0.75
        w.color.a = 1.0
        for x1, y1, x2, y2 in L['walls']:
            w.points += [Point(x=float(x1), y=float(y1), z=0.05), Point(x=float(x2), y=float(y2), z=0.05)]
        arr.markers.append(w)
        # zones + terrain as flat coloured tiles
        for i, z in enumerate(L['zones'] + L['terrain']):
            m = self.marker('zones', i, Marker.CUBE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(z['cx']), float(z['cy']), 0.005
            m.scale.x, m.scale.y, m.scale.z = float(z['lx']), float(z['ly']), 0.01
            m.color.r, m.color.g, m.color.b = (float(c) for c in z['rgba'][:3])
            m.color.a = 0.6
            arr.markers.append(m)
        # barrier
        b = L['barrier']
        m = self.marker('barrier', 0, Marker.CUBE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(b['x']), float(b['y'] + b['length'] / 2), 0.2
        m.scale.x, m.scale.y, m.scale.z = 0.05, float(b['length']), 0.07
        m.color.r, m.color.a = 1.0, 0.8
        arr.markers.append(m)
        # cylinders
        for i, (x, y) in enumerate(self.cylinders):
            m = self.marker('cylinders', i, Marker.CYLINDER)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.21
            m.scale.x = m.scale.y = 0.28
            m.scale.z = 0.42
            m.color.r = m.color.g = m.color.b = 0.95
            m.color.a = 1.0
            arr.markers.append(m)
        # labels
        for i, lab in enumerate(L['labels']):
            m = self.marker('labels', i, Marker.TEXT_VIEW_FACING)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(lab['x']), float(lab['y']), 0.45
            m.scale.z = 0.16
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = f"{lab['num']}. {lab['text']}" if lab['num'] else lab['text']
            arr.markers.append(m)
        return arr

    def publish(self):
        now = self.get_clock().now().to_msg()
        for m in self.msg.markers:
            m.header.stamp = now
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = FieldMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
